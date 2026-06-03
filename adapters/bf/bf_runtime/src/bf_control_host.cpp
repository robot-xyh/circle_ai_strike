#include "circle/bf/runtime/bf_control_host.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

#include "circle/bf/bf_fc_adapters.hpp"
#include "circle/bf/logger.hpp"
#include "circle/bf/runtime/bf_gating.hpp"
#include "circle/debug/pipeline_perf.hpp"
#include "circle/debug/preview_overlay.hpp"
#include "circle/ipc/debug_preview_shm.hpp"
#include "circle/ipc/param_block_shm.hpp"
#include "circle/ipc/shm_contract.hpp"
#include "circle/ipc/strike_params_snapshot_shm.hpp"
#include "circle/ipc/strike_telemetry_shm.hpp"
#include "circle/perception/camera_config_loader.hpp"
#include "circle/perception/camera_info_loader.hpp"
#include "circle/perception/mpp_rga_pipeline.hpp"
#include "circle/perception/pipeline_timing.hpp"
#include "circle/perception/vision_pipeline.hpp"
#include "circle/perception/zero_copy_perception_runtime.hpp"
#include "circle/types/time.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace circle::bf::runtime {

namespace {

struct PerceptionShared {
  std::mutex mu;
  std::condition_variable cv;
  circle::perception::FrameReady latest_frame{};
  bool frame_ready{false};
  uint64_t frame_seq{0};

  circle::types::FrameDetection latest_detection{};
  uint64_t detection_seq{0};

  std::vector<uint8_t> preview_bgr;
  std::vector<uint8_t> preview_rgb;
  uint32_t preview_w{0};
  uint32_t preview_h{0};
  uint64_t preview_seq{0};
  int64_t preview_stamp_ns{0};

  std::vector<uint8_t> latest_infer_bgr;
  uint32_t latest_infer_w{0};
  uint32_t latest_infer_h{0};
  uint64_t latest_infer_frame_seq{0};
  uint64_t latest_infer_result_seq{0};
  int64_t latest_infer_stamp_ns{0};
  float latest_capture_time_ms{0.0F};
  float latest_decode_time_ms{0.0F};
  circle::perception::VisionPipelineTiming latest_vision_timing{};

  struct MatchedDebugFrame {
    std::vector<uint8_t> bgr;
    uint32_t w{0};
    uint32_t h{0};
    uint64_t frame_seq{0};
    uint64_t debug_seq{0};
    int64_t stamp_ns{0};
    circle::debug::PreviewOverlayContext overlay{};
  };
  std::deque<MatchedDebugFrame> matched_debug_queue;
  uint64_t matched_debug_seq{0};
  uint64_t matched_debug_drops{0};
  circle::debug::PreviewOverlayContext latest_control_overlay{};
  circle::debug::PipelinePerfMeter perf;
  uint64_t camera_grab_ok{0};
  uint64_t latest_pipeline_seq{0};

  std::string source{"bf"};
  std::string mode_tag{"target_strike"};
};

/** Latest batched MSP poll result, published by the dedicated I/O thread. */
struct MspPollShared {
  std::mutex mu;
  circle::types::FcState vehicle{};
  circle::bf::MspPollStats stats{};
  std::chrono::steady_clock::time_point last_update{};
  bool have{false};
};

circle::debug::PipelinePerfMeter::SteadyTimePoint steadyFromPipelineNs(int64_t ns) {
  return circle::debug::PipelinePerfMeter::SteadyTimePoint(
      std::chrono::steady_clock::duration(ns));
}

bool notePipelineTiming(circle::debug::PipelinePerfMeter& meter, uint64_t seq,
                        const circle::perception::FramePipelineTiming& timing) {
  if (timing.grab_done_steady_ns <= 0) {
    return false;
  }
  meter.noteProducerStages(
      seq, steadyFromPipelineNs(timing.grab_start_steady_ns),
      steadyFromPipelineNs(timing.grab_done_steady_ns),
      steadyFromPipelineNs(timing.producer_done_steady_ns));
  if (timing.infer_start_steady_ns > 0) {
    meter.noteInferStart(seq, steadyFromPipelineNs(timing.infer_start_steady_ns));
  }
  if (timing.infer_done_steady_ns > 0) {
    meter.noteInferDone(seq, steadyFromPipelineNs(timing.infer_done_steady_ns));
  }
  meter.noteInfEvent();
  return true;
}

void noteFallbackPipelineTiming(circle::debug::PipelinePerfMeter& meter,
                                uint64_t seq, float wait_grab_ms,
                                float producer_ms, float cnn_ms) {
  const auto infer_done = std::chrono::steady_clock::now();
  const auto infer_start =
      infer_done - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<float, std::milli>(cnn_ms));
  const auto producer_done = infer_start;
  const auto grab_done =
      producer_done - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<float, std::milli>(producer_ms));
  const auto grab_start =
      grab_done - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<float, std::milli>(wait_grab_ms));
  meter.noteProducerStages(seq, grab_start, grab_done, producer_done);
  meter.noteInferStart(seq, infer_start);
  meter.noteInferDone(seq, infer_done);
  meter.noteInfEvent();
}

uint32_t sampledHash(const std::vector<uint8_t>& data) {
  uint32_t h = 2166136261u;
  if (data.empty()) {
    return h;
  }
  const size_t step = std::max<size_t>(1, data.size() / 4096);
  for (size_t i = 0; i < data.size(); i += step) {
    h ^= data[i];
    h *= 16777619u;
  }
  return h;
}

void pushMatchedDebugFrame(PerceptionShared& shared, std::vector<uint8_t> bgr,
                           uint32_t w, uint32_t h, uint64_t frame_seq,
                           int64_t stamp_ns,
                           const circle::debug::PreviewOverlayContext& overlay) {
  constexpr size_t kMaxMatchedDebugQueue = 8;
  if (shared.matched_debug_queue.size() >= kMaxMatchedDebugQueue) {
    shared.matched_debug_drops +=
        shared.matched_debug_queue.size() - kMaxMatchedDebugQueue + 1;
    while (shared.matched_debug_queue.size() >= kMaxMatchedDebugQueue) {
      shared.matched_debug_queue.pop_front();
    }
  }
  PerceptionShared::MatchedDebugFrame debug_frame;
  debug_frame.bgr = std::move(bgr);
  debug_frame.w = w;
  debug_frame.h = h;
  debug_frame.frame_seq = frame_seq;
  debug_frame.debug_seq = ++shared.matched_debug_seq;
  debug_frame.stamp_ns = stamp_ns;
  debug_frame.overlay = overlay;
  shared.matched_debug_queue.push_back(std::move(debug_frame));
}

void applyZeroCopyInferResult(PerceptionShared& shared,
                              circle::perception::ZeroCopyInferResult&& result) {
  std::lock_guard<std::mutex> lk(shared.mu);
  const bool pipeline_ok =
      notePipelineTiming(shared.perf, result.frame_seq, result.pipeline);
  if (!pipeline_ok) {
    shared.cv.notify_all();
    return;
  }

  result.detection.seq = result.frame_seq;
  shared.latest_pipeline_seq = result.frame_seq;
  shared.latest_detection = result.detection;
  (void)shared.perf.tryNoteControlInputReady(result.frame_seq,
                                             std::chrono::steady_clock::now());
  shared.latest_infer_w = result.preview_w;
  shared.latest_infer_h = result.preview_h;
  shared.latest_infer_frame_seq = result.frame_seq;
  shared.latest_infer_stamp_ns = result.stamp_ns;
  shared.latest_capture_time_ms = result.capture_time_ms;
  shared.latest_decode_time_ms = result.decode_time_ms;
  shared.latest_vision_timing = result.timing;
  ++shared.latest_infer_result_seq;
  ++shared.detection_seq;
  shared.frame_ready = true;
  ++shared.frame_seq;

  circle::debug::PreviewOverlayContext overlay = shared.latest_control_overlay;
  overlay.source = overlay.source.empty() ? shared.source : overlay.source;
  overlay.mode_tag = overlay.mode_tag.empty() ? shared.mode_tag : overlay.mode_tag;
  overlay.frame_seq = result.frame_seq;
  overlay.stamp_ns = result.stamp_ns;
  overlay.detection.valid = result.detection.valid;
  overlay.detection.cx = result.detection.detection.cx;
  overlay.detection.cy = result.detection.detection.cy;
  overlay.detection.width = result.detection.detection.width;
  overlay.detection.height = result.detection.detection.height;
  overlay.detection.score = result.detection.detection.score;
  overlay.detection.class_id = result.detection.detection.class_id;
  overlay.detection.class_name = result.detection.detection.class_name;
  overlay.detection.seq = result.frame_seq;
  overlay.detection.capture_ns = static_cast<int64_t>(result.detection.capture_ns);
  overlay.capture_time_ms = result.capture_time_ms;
  overlay.decode_time_ms = result.decode_time_ms;
  overlay.preprocess_time_ms = result.timing.preprocess_time_ms;
  overlay.inference_time_ms = result.timing.inference_time_ms;
  overlay.postprocess_time_ms = result.timing.postprocess_time_ms;
  overlay.cam_pipeline_ms = result.timing.total_time_ms;

  if (!result.preview_bgr.empty() && result.preview_w > 0 && result.preview_h > 0) {
    auto preview_bgr = std::move(result.preview_bgr);
    shared.preview_bgr = preview_bgr;
    shared.preview_w = result.preview_w;
    shared.preview_h = result.preview_h;
    shared.preview_stamp_ns = result.stamp_ns;
    ++shared.preview_seq;
    pushMatchedDebugFrame(shared, std::move(preview_bgr), result.preview_w,
                          result.preview_h, result.frame_seq, result.stamp_ns,
                          overlay);
  }
  shared.cv.notify_all();
}

bool mppRgaLoggerEnabled(circle::perception::MppRgaLogLevel level) {
  switch (level) {
    case circle::perception::MppRgaLogLevel::Debug:
      return circle::bf::logEnabled(circle::bf::LogLevel::Debug);
    case circle::perception::MppRgaLogLevel::Info:
      return circle::bf::logEnabled(circle::bf::LogLevel::Info);
    case circle::perception::MppRgaLogLevel::Warn:
      return circle::bf::logEnabled(circle::bf::LogLevel::Warn);
    case circle::perception::MppRgaLogLevel::Error:
      return circle::bf::logEnabled(circle::bf::LogLevel::Error);
  }
  return false;
}

bool rknnLoggerEnabled(circle::perception::RknnLogLevel level) {
  switch (level) {
    case circle::perception::RknnLogLevel::Debug:
      return circle::bf::logEnabled(circle::bf::LogLevel::Debug);
    case circle::perception::RknnLogLevel::Info:
      return circle::bf::logEnabled(circle::bf::LogLevel::Info);
    case circle::perception::RknnLogLevel::Warn:
      return circle::bf::logEnabled(circle::bf::LogLevel::Warn);
    case circle::perception::RknnLogLevel::Error:
      return circle::bf::logEnabled(circle::bf::LogLevel::Error);
  }
  return false;
}

void mppRgaLogger(circle::perception::MppRgaLogLevel level, const char* message) {
  switch (level) {
    case circle::perception::MppRgaLogLevel::Debug:
      circle::bf::logDebug(message);
      break;
    case circle::perception::MppRgaLogLevel::Info:
      circle::bf::logInfo(message);
      break;
    case circle::perception::MppRgaLogLevel::Warn:
      circle::bf::logWarn(message);
      break;
    case circle::perception::MppRgaLogLevel::Error:
      circle::bf::logError(message);
      break;
  }
}

void rknnLogger(circle::perception::RknnLogLevel level, const char* message) {
  switch (level) {
    case circle::perception::RknnLogLevel::Debug:
      circle::bf::logDebug(message);
      break;
    case circle::perception::RknnLogLevel::Info:
      circle::bf::logInfo(message);
      break;
    case circle::perception::RknnLogLevel::Warn:
      circle::bf::logWarn(message);
      break;
    case circle::perception::RknnLogLevel::Error:
      circle::bf::logError(message);
      break;
  }
}

void cameraThread(circle::perception::MppCameraSource& camera,
                  PerceptionShared& shared, uint32_t fps,
                  std::atomic<bool>& running) {
  const auto period = std::chrono::microseconds(
      fps > 0 ? static_cast<int64_t>(1'000'000 / fps) : 16'667);
  auto next = std::chrono::steady_clock::now();
  auto last_log = std::chrono::steady_clock::now();
  uint64_t grab_ok = 0;
  uint64_t grab_fail = 0;
  uint64_t hw_rgb_ok = 0;
  uint64_t preview_updates = 0;
  uint64_t last_frame_seq = 0;
  uint64_t last_preview_seq = 0;
  uint32_t last_w = 0;
  uint32_t last_h = 0;
  int64_t last_stamp = 0;
  while (running.load()) {
    circle::perception::FrameReady frame;
    if (camera.grab(frame)) {
      ++grab_ok;
      std::lock_guard<std::mutex> lk(shared.mu);
      ++shared.camera_grab_ok;
      shared.latest_frame = frame;
      shared.frame_ready = true;
      ++shared.frame_seq;
      last_frame_seq = shared.frame_seq;
      if (frame.hw_rgb && frame.data && frame.width > 0 && frame.height > 0) {
        ++hw_rgb_ok;
        const size_t bytes = static_cast<size_t>(frame.width) * frame.height * 3u;
        if (shared.preview_bgr.size() != bytes) {
          shared.preview_bgr.resize(bytes);
        }
        if (shared.preview_rgb.size() != bytes) {
          shared.preview_rgb.resize(bytes);
        }
        const auto* rgb = frame.data;
        auto* bgr = shared.preview_bgr.data();
        std::memcpy(shared.preview_rgb.data(), rgb, bytes);
        for (size_t i = 0; i < bytes; i += 3) {
          bgr[i] = rgb[i + 2];
          bgr[i + 1] = rgb[i + 1];
          bgr[i + 2] = rgb[i];
        }
        shared.preview_w = frame.width;
        shared.preview_h = frame.height;
        shared.preview_stamp_ns = static_cast<int64_t>(frame.capture_ns);
        ++shared.preview_seq;
        ++preview_updates;
        last_preview_seq = shared.preview_seq;
        last_w = frame.width;
        last_h = frame.height;
        last_stamp = shared.preview_stamp_ns;
      }
      shared.cv.notify_all();
    } else {
      ++grab_fail;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(5)) {
      circle::bf::logInfo("bf_flight/camera: grab_ok=", grab_ok,
                          " grab_fail=", grab_fail, " hw_rgb=", hw_rgb_ok,
                          " frame_seq=", last_frame_seq,
                          " preview_updates=", preview_updates,
                          " preview_seq=", last_preview_seq, " size=", last_w,
                          'x', last_h, " stamp_ns=", last_stamp);
      last_log = now;
    }
    next += period;
    std::this_thread::sleep_until(next);
  }
}

void inferThread(circle::perception::VisionPipeline& pipeline,
                 PerceptionShared& shared, uint32_t max_infer_fps,
                 std::atomic<bool>& running) {
  uint64_t last_frame_seq = 0;
  auto last_infer = std::chrono::steady_clock::time_point{};
  auto last_log = std::chrono::steady_clock::now();
  uint64_t frames_seen = 0;
  uint64_t infer_calls = 0;
  uint64_t detection_updates = 0;
  uint64_t valid_detections = 0;
  uint64_t skipped_fps = 0;
  while (running.load()) {
    circle::perception::FrameReady frame;
    std::vector<uint8_t> bgr;
    std::vector<uint8_t> rgb;
    uint32_t bgr_w = 0;
    uint32_t bgr_h = 0;
    int64_t bgr_stamp = 0;
    {
      std::unique_lock<std::mutex> lk(shared.mu);
      shared.cv.wait_for(lk, std::chrono::milliseconds(50), [&]() {
        return !running.load() ||
               (shared.frame_ready && shared.frame_seq != last_frame_seq);
      });
      if (!running.load()) {
        break;
      }
      if (!shared.frame_ready || shared.frame_seq == last_frame_seq) {
        continue;
      }
      frame = shared.latest_frame;
      bgr = shared.preview_bgr;
      rgb = shared.preview_rgb;
      bgr_w = shared.preview_w;
      bgr_h = shared.preview_h;
      bgr_stamp = shared.preview_stamp_ns;
      last_frame_seq = shared.frame_seq;
    }
    ++frames_seen;

    if (bgr.empty() || rgb.empty() || bgr_w == 0 || bgr_h == 0 ||
        bgr_stamp != static_cast<int64_t>(frame.capture_ns)) {
      continue;
    }
    frame.data = rgb.data();
    frame.data_size = rgb.size();
    frame.width = bgr_w;
    frame.height = bgr_h;
    frame.hw_rgb = true;

    if (max_infer_fps > 0) {
      const auto min_dt = std::chrono::duration<double>(1.0 / max_infer_fps);
      const auto now = std::chrono::steady_clock::now();
      if (last_infer.time_since_epoch().count() > 0 && now - last_infer < min_dt) {
        ++skipped_fps;
        continue;
      }
      last_infer = now;
    }

    circle::types::FrameDetection det;
    circle::perception::VisionPipelineTiming timing;
    ++infer_calls;
    const bool valid_detection = pipeline.processFrame(frame, det, &timing);
    {
      std::lock_guard<std::mutex> lk(shared.mu);
      shared.latest_pipeline_seq = frame.seq;
      noteFallbackPipelineTiming(shared.perf, frame.seq, frame.capture_time_ms,
                                 frame.decode_time_ms, timing.total_time_ms);
      det.seq = frame.seq;
      shared.latest_detection = det;
      (void)shared.perf.tryNoteControlInputReady(
          frame.seq, std::chrono::steady_clock::now());
      shared.latest_infer_w = bgr_w;
      shared.latest_infer_h = bgr_h;
      shared.latest_infer_frame_seq = frame.seq;
      shared.latest_infer_stamp_ns = static_cast<int64_t>(frame.capture_ns);
      shared.latest_capture_time_ms = frame.capture_time_ms;
      shared.latest_decode_time_ms = frame.decode_time_ms;
      shared.latest_vision_timing = timing;
      ++shared.latest_infer_result_seq;
      ++shared.detection_seq;
      ++detection_updates;
      if (valid_detection) {
        ++valid_detections;
      }

      circle::debug::PreviewOverlayContext overlay = shared.latest_control_overlay;
      overlay.source = overlay.source.empty() ? shared.source : overlay.source;
      overlay.mode_tag =
          overlay.mode_tag.empty() ? shared.mode_tag : overlay.mode_tag;
      overlay.frame_seq = frame.seq;
      overlay.stamp_ns = static_cast<int64_t>(frame.capture_ns);
      overlay.detection.valid = det.valid;
      overlay.detection.cx = det.detection.cx;
      overlay.detection.cy = det.detection.cy;
      overlay.detection.width = det.detection.width;
      overlay.detection.height = det.detection.height;
      overlay.detection.score = det.detection.score;
      overlay.detection.class_id = det.detection.class_id;
      overlay.detection.class_name = det.detection.class_name;
      overlay.detection.seq = frame.seq;
      overlay.detection.capture_ns = static_cast<int64_t>(det.capture_ns);
      overlay.capture_time_ms = frame.capture_time_ms;
      overlay.decode_time_ms = frame.decode_time_ms;
      overlay.preprocess_time_ms = timing.preprocess_time_ms;
      overlay.inference_time_ms = timing.inference_time_ms;
      overlay.postprocess_time_ms = timing.postprocess_time_ms;
      overlay.cam_pipeline_ms = timing.total_time_ms;
      pushMatchedDebugFrame(shared, std::move(bgr), bgr_w, bgr_h, frame.seq,
                            static_cast<int64_t>(frame.capture_ns), overlay);
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(5)) {
      circle::bf::logInfo("bf_flight/infer: frames_seen=", frames_seen,
                          " infer_calls=", infer_calls,
                          " updates=", detection_updates,
                          " valid=", valid_detections,
                          " skipped_fps=", skipped_fps,
                          " last_frame_seq=", last_frame_seq,
                          " cap_ms=", frame.capture_time_ms,
                          " dec_ms=", frame.decode_time_ms,
                          " pre_ms=", timing.preprocess_time_ms,
                          " inf_ms=", timing.inference_time_ms,
                          " post_ms=", timing.postprocess_time_ms,
                          " total_ms=", timing.total_time_ms,
                          " raw_det=", timing.raw_detections,
                          " accepted_idx=", timing.accepted_index);
      last_log = now;
    }
  }
}

void previewThread(circle::ipc::DebugPreviewWriter& writer,
                   PerceptionShared& shared, uint32_t max_fps,
                   std::atomic<bool>& running) {
  const auto min_dt = std::chrono::microseconds(
      max_fps > 0 ? static_cast<int64_t>(1'000'000 / max_fps) : 66'666);
  auto next = std::chrono::steady_clock::now();
  uint64_t publish_ok = 0;
  uint64_t publish_fail = 0;
  uint64_t no_frame = 0;
  uint64_t last_published_seq = 0;
  uint32_t last_hash = 0;
  uint32_t last_w = 0;
  uint32_t last_h = 0;
  int64_t last_stamp = 0;
  uint64_t stale_dropped = 0;
  uint64_t matched_drops = 0;

  while (running.load()) {
    PerceptionShared::MatchedDebugFrame frame;
    bool got_frame = false;
    {
      std::lock_guard<std::mutex> lk(shared.mu);
      matched_drops = shared.matched_debug_drops;
      if (!shared.matched_debug_queue.empty()) {
        if (shared.matched_debug_queue.size() > 1) {
          stale_dropped += shared.matched_debug_queue.size() - 1;
        }
        frame = std::move(shared.matched_debug_queue.back());
        shared.matched_debug_queue.clear();
        got_frame = true;
      }
    }

    if (got_frame && !frame.bgr.empty()) {
      {
        std::lock_guard<std::mutex> lk(shared.mu);
        circle::debug::mergePipelinePerfOverlay(frame.overlay,
                                                shared.latest_control_overlay);
      }
      frame.overlay.frame_seq =
          frame.overlay.frame_seq == 0 ? frame.frame_seq : frame.overlay.frame_seq;
      frame.overlay.stamp_ns = frame.stamp_ns;
      circle::debug::PreviewOverlayShmData overlay_shm =
          circle::debug::fromPreviewOverlayContext(frame.overlay);
      if (writer.publishWithOverlay(frame.bgr.data(), frame.w, frame.h,
                                    frame.stamp_ns, overlay_shm)) {
        ++publish_ok;
        last_published_seq = frame.frame_seq;
        last_hash = sampledHash(frame.bgr);
        last_w = frame.w;
        last_h = frame.h;
        last_stamp = frame.stamp_ns;
      } else {
        ++publish_fail;
      }
    } else {
      ++no_frame;
    }

    next += min_dt;
    const auto now = std::chrono::steady_clock::now();
    if (next < now) {
      next = now;
    }
    std::this_thread::sleep_until(next);
  }
  (void)publish_ok;
  (void)publish_fail;
  (void)no_frame;
  (void)last_published_seq;
  (void)last_hash;
  (void)last_w;
  (void)last_h;
  (void)last_stamp;
  (void)stale_dropped;
  (void)matched_drops;
}

void mspIoThread(std::shared_ptr<circle::bf::MspClient> msp,
                 circle::bf::BfStateSource& state_source,
                 circle::bf::BfRateSink& rate_sink,
                 std::chrono::steady_clock::duration period,
                 uint32_t attitude_divisor, uint32_t status_divisor,
                 bool atomic_poll_mode, MspPollShared& shared,
                 std::atomic<bool>& running) {
  auto next = std::chrono::steady_clock::now();
  uint64_t cycle = 0;
  const auto drain_budget = std::min<std::chrono::steady_clock::duration>(
      period / 3, std::chrono::milliseconds(3));
  while (running.load()) {
    circle::bf::MspPollResult res;
    if (atomic_poll_mode) {
      const bool want_rc = rate_sink.needsRcPoll();
      msp->pollState(want_rc, res);
    } else {
      rate_sink.flushStagedWire();

      const bool want_status = (cycle % status_divisor) == 0;
      const bool want_attitude = (cycle % attitude_divisor) == 0;
      const bool want_rc = rate_sink.needsRcPoll() && want_attitude;
      if (want_status) {
        msp->writeFrameRaw(circle::bf::MspCommand::kStatus, nullptr, 0);
      }
      if (want_attitude) {
        msp->writeFrameRaw(circle::bf::MspCommand::kAttitude, nullptr, 0);
      }
      if (want_rc) {
        msp->writeFrameRaw(circle::bf::MspCommand::kRc, nullptr, 0);
      }

      res.status_requested = want_status;
      res.attitude_requested = want_attitude;
      res.rc_requested = want_rc;
      msp->drainResponses(drain_budget, res);
    }

    rate_sink.ingestPoll(res);
    const circle::types::FcState fc = state_source.ingestPoll(res);
    const auto stats = state_source.pollStats();
    {
      std::lock_guard<std::mutex> lk(shared.mu);
      shared.vehicle = fc;
      shared.stats = stats;
      shared.last_update = std::chrono::steady_clock::now();
      shared.have = true;
    }

    ++cycle;
    next += period;
    const auto now = std::chrono::steady_clock::now();
    if (next < now) {
      next = now;
    }
    std::this_thread::sleep_until(next);
  }
}

void logMspIoStats(const std::shared_ptr<circle::bf::MspClient>& msp_client,
                   const circle::debug::PipelinePerfSnapshot& perf,
                   double configured_poll_hz, double configured_send_hz) {
  const auto w = msp_client->snapshotIoWindow();
  if (w.window_s <= 0.0) {
    return;
  }
  const double poll_hz = static_cast<double>(w.poll_ops) / w.window_s;
  const double send_hz = static_cast<double>(w.send_ops) / w.window_s;
  const uint64_t io_ops = w.poll_ops + w.send_ops;
  const double lock_wait_ms =
      io_ops > 0 ? (static_cast<double>(w.lock_wait_us) /
                    static_cast<double>(io_ops)) / 1000.0
                 : 0.0;
  const double avg_poll_ms =
      w.poll_ops > 0 ? static_cast<double>(w.poll_round_trip_us) /
                           static_cast<double>(w.poll_ops) / 1000.0
                     : 0.0;
  const double avg_send_ms =
      w.send_ops > 0 ? static_cast<double>(w.send_round_trip_us) /
                           static_cast<double>(w.send_ops) / 1000.0
                     : 0.0;
  const double poll_wire_pct =
      w.poll_round_trip_us > 0
          ? std::min(100.0, 100.0 * static_cast<double>(w.poll_wire_us) /
                                static_cast<double>(w.poll_round_trip_us))
          : 0.0;
  const double send_wire_pct =
      w.send_round_trip_us > 0
          ? std::min(100.0, 100.0 * static_cast<double>(w.send_wire_us) /
                                static_cast<double>(w.send_round_trip_us))
          : 0.0;
  const uint64_t total_bytes = w.tx_bytes + w.rx_bytes;
  const int baud = msp_client->baudrate();
  const double bus_util_pct =
      baud > 0 ? 100.0 * static_cast<double>(total_bytes) * 10.0 /
                     (static_cast<double>(baud) * w.window_s)
               : 0.0;
  const double avg_cycle_bytes =
      io_ops > 0 ? static_cast<double>(total_bytes) / static_cast<double>(io_ops)
                 : 0.0;
  const double est_max_hz =
      avg_cycle_bytes > 0.0 && baud > 0
          ? static_cast<double>(baud) / (avg_cycle_bytes * 10.0)
          : 0.0;
  const double serial_share_pct =
      100.0 * static_cast<double>(w.poll_round_trip_us + w.send_round_trip_us) /
      (w.window_s * 1'000'000.0);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1);
  oss << "bf_flight/msp/io: baud=" << baud << " window_s=" << w.window_s
      << " poll_hz=" << poll_hz << "/" << configured_poll_hz
      << " send_hz=" << send_hz << "/" << configured_send_hz;
  if (perf.ctrl_fps.has_value()) {
    oss << " ctrl_fps=" << *perf.ctrl_fps;
  }
  if (perf.msp_fps.has_value()) {
    oss << " msp_fps=" << *perf.msp_fps;
  }
  oss << " poll_rt_ms=" << avg_poll_ms << " send_rt_ms=" << avg_send_ms
      << " lock_wait_ms=" << lock_wait_ms << " poll_wire_pct=" << poll_wire_pct
      << " send_wire_pct=" << send_wire_pct << " bus_util_pct=" << bus_util_pct
      << " serial_share_pct=" << serial_share_pct
      << " max_poll_ms=" << (static_cast<double>(w.max_poll_round_trip_us) / 1000.0)
      << " max_send_ms=" << (static_cast<double>(w.max_send_round_trip_us) / 1000.0)
      << " max_lock_ms=" << (static_cast<double>(w.max_lock_wait_us) / 1000.0)
      << " tx_kBps=" << (static_cast<double>(w.tx_bytes) / w.window_s / 1024.0)
      << " rx_kBps=" << (static_cast<double>(w.rx_bytes) / w.window_s / 1024.0)
      << " est_max_hz=" << est_max_hz;
  if (w.poll_fail + w.send_fail > 0) {
    oss << " fail_poll=" << w.poll_fail << " fail_send=" << w.send_fail;
  }
  if (perf.e2e_wire_ms.has_value()) {
    oss << " e2e_wire_ms=" << *perf.e2e_wire_ms;
  }
  if (perf.msp_gate_ms.has_value()) {
    oss << " msp_gate_ms=" << *perf.msp_gate_ms;
  }
  constexpr double kSendStallWarnMs = 30.0;
  const double max_send_ms = static_cast<double>(w.max_send_round_trip_us) / 1000.0;
  const bool send_stall = max_send_ms > kSendStallWarnMs;
  if (bus_util_pct > 85.0) {
    oss << " **BANDWIDTH_LIMITED**";
  } else if (poll_wire_pct < 25.0 && send_wire_pct < 25.0 &&
             serial_share_pct > 20.0) {
    oss << " **LATENCY_LIMITED**";
  } else if (lock_wait_ms > 2.0) {
    oss << " **MUTEX_CONTENTION**";
  }
  if (send_stall) {
    oss << " **SEND_STALL**";
  }
  circle::bf::logInfo(oss.str());
  if (send_stall) {
    circle::bf::logWarn("bf_flight/msp/io: SET write/ack stall max_send_ms=",
                        max_send_ms, " (>", kSendStallWarnMs,
                        "ms) — serial TX/BF hiccup; send_hz=", send_hz);
  }
}

}  // namespace

struct BfControlHost::Impl {
  BfRuntimeConfig cfg;
  std::shared_ptr<circle::bf::MspClient> msp;
  IBfStrikeController& controller;

  Impl(BfRuntimeConfig c, std::shared_ptr<circle::bf::MspClient> m,
       IBfStrikeController& ctrl)
      : cfg(std::move(c)), msp(std::move(m)), controller(ctrl) {}

  int run(std::atomic<bool>& running, uint64_t max_iterations);
};

BfControlHost::BfControlHost(BfRuntimeConfig config,
                             std::shared_ptr<circle::bf::MspClient> msp,
                             IBfStrikeController& controller)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(msp),
                                   controller)) {}

BfControlHost::~BfControlHost() = default;

int BfControlHost::run(std::atomic<bool>& running, uint64_t max_iterations) {
  return impl_->run(running, max_iterations);
}

int BfControlHost::Impl::run(std::atomic<bool>& running,
                             uint64_t max_iterations) {
  IBfStrikeController& controller = this->controller;

  if (!cfg.camera_info_yaml.empty() && cfg.intrinsics.fx <= 0.0F) {
    circle::bf::logWarn("bf_flight: camera_info not loaded path=",
                        cfg.camera_info_yaml,
                        " — vision ex/ey will be in pixels, not normalized");
  }
  circle::perception::setMppRgaLogHandler(mppRgaLoggerEnabled, mppRgaLogger);
  circle::perception::setRknnLogHandler(rknnLoggerEnabled, rknnLogger);

  if (!msp->openSerial(cfg.msp.device, cfg.msp.baud)) {
    circle::bf::logError("bf_flight: failed to open MSP serial ", cfg.msp.device);
    return 1;
  }

  auto msp_probe = [&]() -> bool {
    circle::bf::MspStatus st{};
    std::vector<uint16_t> rc_probe;
    const bool st_ok = msp->readStatus(st);
    const std::string st_err = msp->lastError();
    const bool rc_ok = msp->readRc(&rc_probe);
    const std::string rc_err = msp->lastError();
    std::ostringstream probe_msg;
    probe_msg << "bf_flight/msp: probe @" << cfg.msp.baud
              << " status=" << (st_ok ? "ok" : "fail")
              << " rc=" << (rc_ok ? "ok" : "fail") << " rc_ch=" << rc_probe.size();
    if (!st_ok) {
      probe_msg << " status_err=" << st_err;
    }
    if (!rc_ok) {
      probe_msg << " rc_err=" << rc_err;
    }
    circle::bf::logInfo(probe_msg.str());
    return st_ok && rc_ok;
  };

  if (!msp_probe()) {
    static constexpr int kFallbackBauds[] = {921600, 57600, 115200};
    for (int baud : kFallbackBauds) {
      if (baud == cfg.msp.baud) {
        continue;
      }
      if (!msp->openSerial(cfg.msp.device, baud)) {
        continue;
      }
      circle::bf::logInfo("bf_flight/msp: retry probe @", baud);
      if (msp_probe()) {
        cfg.msp.baud = baud;
        circle::bf::logInfo("bf_flight/msp: autobaud selected ", baud);
        break;
      }
    }
  }

  if (cfg.msp.override_mode_flag_auto) {
    std::vector<uint8_t> box_ids;
    if (msp->readBoxIds(&box_ids)) {
      int override_bit = -1;
      for (size_t i = 0; i < box_ids.size(); ++i) {
        if (box_ids[i] == circle::bf::kBoxPermanentIdMspOverride) {
          override_bit = static_cast<int>(i);
          break;
        }
      }
      if (override_bit >= 0) {
        const uint32_t detected = 1U << static_cast<uint32_t>(override_bit);
        std::ostringstream auto_msg;
        auto_msg << "bf_flight/msp: override_mode_flag auto-detected bit="
                 << override_bit << " mask=0x" << std::hex << detected
                 << " (was yaml=0x" << cfg.msp.override_mode_flag << std::dec
                 << ", nboxes=" << box_ids.size() << ")";
        circle::bf::logInfo(auto_msg.str());
        cfg.msp.override_mode_flag = detected;
      } else {
        std::ostringstream warn_msg;
        warn_msg << "bf_flight/msp: MSP OVERRIDE (permId 50) not in BOXIDS"
                    " (nboxes="
                 << box_ids.size()
                 << "); enable it in BF Modes tab. Falling back to yaml mask=0x"
                 << std::hex << cfg.msp.override_mode_flag << std::dec;
        circle::bf::logWarn(warn_msg.str());
      }
    } else {
      circle::bf::logWarn("bf_flight/msp: MSP_BOXIDS query failed (",
                          msp->lastError(),
                          "); falling back to yaml msp_override_mode_flag");
    }
  }

  circle::bf::MspPassthroughDebugConfig passthrough_dbg;
  passthrough_dbg.log_enabled = cfg.msp.passthrough_log;
  passthrough_dbg.log_interval_s = cfg.msp.passthrough_log_interval_s;
  passthrough_dbg.throttle_jump_pwm = cfg.msp.passthrough_throttle_jump_pwm;
  passthrough_dbg.override_mode_flag = cfg.msp.override_mode_flag;
  passthrough_dbg.override_channels_mask = cfg.msp.override_channels_mask;
  passthrough_dbg.passthrough_channel_count = cfg.msp.passthrough_channel_count;
  passthrough_dbg.live_publish_hz = cfg.msp.live_publish_hz > 0.0
                                        ? cfg.msp.live_publish_hz
                                        : cfg.msp.passthrough_hz;
  passthrough_dbg.override_grace_hold_s = cfg.msp.override_grace_hold_s;
  const double msp_set_hz = std::max(1.0, passthrough_dbg.live_publish_hz);
  const auto msp_io_period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / msp_set_hz));
  const uint32_t attitude_divisor =
      std::max<uint32_t>(1, cfg.msp.attitude_poll_divisor);
  const uint32_t status_divisor =
      std::max<uint32_t>(1, cfg.msp.status_poll_divisor);
  const double msp_poll_hz = msp_set_hz / static_cast<double>(attitude_divisor);
  circle::bf::BfRateSink rate_sink(msp, circle::bf::BfRcMapper(cfg.rc),
                                   passthrough_dbg);
  circle::bf::BfStateSource state_source(msp, cfg.rc, cfg.msp.override_mode_flag,
                                         cfg.msp.override_channels_mask);
  const circle::bf::BfRcMapper rc_mapper(cfg.rc);
  auto throttleCmdNorm = [&rc_mapper, this](const circle::types::RateCommand& rates,
                                            bool armed) -> float {
    const auto ch = rc_mapper.mapRates(rates, armed);
    const float span = static_cast<float>(cfg.rc.rc_max - cfg.rc.rc_min);
    if (span <= 0.0F || cfg.rc.throttle_channel >= ch.size()) {
      return rates.thrust_z;
    }
    return (static_cast<float>(ch[cfg.rc.throttle_channel]) -
            static_cast<float>(cfg.rc.rc_min)) /
           span;
  };

  circle::ipc::StrikeTelemetryWriter telemetry;
  telemetry.open(circle::ipc::kStrikeTelemetryShmName);
  circle::ipc::ParamBlockReader param_reader;
  bool param_reader_open = false;
  circle::ipc::StrikeParamsSnapshotWriter params_snapshot;
  params_snapshot.open(circle::ipc::kStrikeParamsSnapshotShmName);
  const auto publishParamsSnapshot = [&params_snapshot, &controller]() {
    if (!params_snapshot.writeJson(controller.paramsSnapshotJson())) {
      circle::bf::logWarn("bf_flight: strike params snapshot write failed");
    }
  };
  publishParamsSnapshot();

  PerceptionShared perception_shared;
  perception_shared.source = "bf";
  perception_shared.mode_tag = cfg.mode_tag;
  circle::perception::MppCameraSource camera_source(cfg.camera);
  circle::perception::VisionPipelineConfig perception_cfg;
  perception_cfg.filter = cfg.filter;
  perception_cfg.intrinsics = cfg.intrinsics;
  perception_cfg.model_path = cfg.model_path;
  perception_cfg.detection_file = cfg.detection_file;
  perception_cfg.pipeline_out_w = static_cast<int>(cfg.camera.output_width);
  perception_cfg.pipeline_out_h = static_cast<int>(cfg.camera.output_height);
  perception_cfg.conf_threshold = cfg.conf_threshold;
  perception_cfg.iou_threshold = cfg.iou_threshold;
  perception_cfg.max_det = cfg.max_det;
  circle::perception::VisionPipeline perception(std::move(perception_cfg));

  std::thread camera_thread;
  std::thread infer_thread;
  std::thread preview_thread;
  circle::ipc::DebugPreviewWriter preview_writer;
  std::unique_ptr<circle::perception::ZeroCopyPerceptionRuntime> zero_copy_runtime;
  bool use_zero_copy_pipeline = cfg.zero_copy_pipeline_enabled &&
                                cfg.detection_file.empty() &&
                                cfg.zero_copy_slot_count > 0 &&
                                !cfg.model_path.empty();

  if (use_zero_copy_pipeline) {
    circle::perception::ZeroCopyPerceptionConfig zcfg;
    zcfg.camera = cfg.camera;
    zcfg.vision.filter = cfg.filter;
    zcfg.vision.intrinsics = cfg.intrinsics;
    zcfg.vision.model_path = cfg.model_path;
    zcfg.vision.pipeline_out_w = static_cast<int>(cfg.camera.output_width);
    zcfg.vision.pipeline_out_h = static_cast<int>(cfg.camera.output_height);
    zcfg.vision.conf_threshold = cfg.conf_threshold;
    zcfg.vision.iou_threshold = cfg.iou_threshold;
    zcfg.vision.max_det = cfg.max_det;
    zcfg.slot_count = std::clamp(cfg.zero_copy_slot_count, 1, 4);
    zcfg.infer_worker_count = std::clamp(cfg.infer_worker_count, 1, zcfg.slot_count);
    zcfg.vision.rknn_core_masks = cfg.rknn_core_masks;
    zcfg.max_infer_fps = cfg.max_infer_fps;
    zero_copy_runtime =
        std::make_unique<circle::perception::ZeroCopyPerceptionRuntime>(
            std::move(zcfg));
    if (!zero_copy_runtime->initialize()) {
      circle::bf::logWarn("bf_flight: zero-copy pipeline init failed; falling back "
                          "to single-slot capture");
      zero_copy_runtime.reset();
      use_zero_copy_pipeline = false;
    }
  }

  const bool use_camera_pipeline =
      cfg.detection_file.empty() &&
      (use_zero_copy_pipeline || perception.initialize());
  if (use_zero_copy_pipeline && zero_copy_runtime) {
    zero_copy_runtime->start(
        [&](circle::perception::ZeroCopyInferResult&& result) {
          applyZeroCopyInferResult(perception_shared, std::move(result));
        },
        &running);
    std::ostringstream zc_msg;
    zc_msg << "bf_flight: zero-copy pipeline camera=" << cfg.camera.device << " @ "
           << cfg.camera.fps << " fps slots=" << zero_copy_runtime->slotCount()
           << " infer_workers="
           << std::clamp(cfg.infer_worker_count, 1, cfg.zero_copy_slot_count)
           << " rknn_core_masks=[";
    for (size_t i = 0; i < cfg.rknn_core_masks.size(); ++i) {
      if (i > 0) {
        zc_msg << ',';
      }
      zc_msg << cfg.rknn_core_masks[i];
    }
    zc_msg << ']';
    circle::bf::logInfo(zc_msg.str());
    if (cfg.preview_shm_enabled) {
      circle::ipc::DebugPreviewWriterConfig pwcfg;
      pwcfg.shm_name = circle::ipc::kDebugPreviewShmName;
      pwcfg.width = cfg.camera.output_width;
      pwcfg.height = cfg.camera.output_height;
      pwcfg.encoding = 2;
      if (preview_writer.open(pwcfg)) {
        preview_thread = std::thread(previewThread, std::ref(preview_writer),
                                     std::ref(perception_shared),
                                     cfg.preview_max_fps, std::ref(running));
        circle::bf::logInfo("bf_flight: debug_preview SHM enabled @ ",
                            cfg.preview_max_fps, " fps");
      }
    }
  } else if (use_camera_pipeline) {
    if (!camera_source.start()) {
      circle::bf::logError("bf_flight: camera start failed on ", cfg.camera.device);
    } else {
      circle::bf::logInfo("bf_flight: camera ", cfg.camera.device, " @ ",
                          cfg.camera.fps, " fps, pipeline ",
                          cfg.camera.output_width, 'x', cfg.camera.output_height,
                          " (single-slot fallback)");
      camera_thread = std::thread(cameraThread, std::ref(camera_source),
                                  std::ref(perception_shared), cfg.camera.fps,
                                  std::ref(running));
      infer_thread = std::thread(inferThread, std::ref(perception),
                                 std::ref(perception_shared), cfg.max_infer_fps,
                                 std::ref(running));
      if (cfg.preview_shm_enabled) {
        circle::ipc::DebugPreviewWriterConfig pwcfg;
        pwcfg.shm_name = circle::ipc::kDebugPreviewShmName;
        pwcfg.width = cfg.camera.output_width;
        pwcfg.height = cfg.camera.output_height;
        pwcfg.encoding = 2;
        if (preview_writer.open(pwcfg)) {
          preview_thread =
              std::thread(previewThread, std::ref(preview_writer),
                          std::ref(perception_shared), cfg.preview_max_fps,
                          std::ref(running));
          circle::bf::logInfo("bf_flight: debug_preview SHM enabled @ ",
                              cfg.preview_max_fps, " fps");
        }
      }
    }
  }

  const bool msp_passthrough_active =
      cfg.dry_run && cfg.msp.passthrough_in_dry_run;

  std::ostringstream started_msg;
  started_msg << "bf_flight started, MSP=" << cfg.msp.device;
  if (msp_passthrough_active) {
    started_msg << " dry_run=MSP_RC_passthrough"
                << " set_hz=" << cfg.msp.passthrough_hz << " poll_hz=" << msp_poll_hz
                << " rc_map=R" << cfg.rc.roll_channel << "/P" << cfg.rc.pitch_channel
                << "/T" << cfg.rc.throttle_channel << "/Y" << cfg.rc.yaw_channel
                << "/A" << cfg.rc.aux_arm_channel
                << " passthrough_log=" << (cfg.msp.passthrough_log ? "yes" : "no");
  } else if (cfg.dry_run) {
    started_msg << " dry_run=MSP_silent";
  } else {
    started_msg << " live_cmd=mapRates set_hz=" << msp_set_hz
                << " poll_hz=" << msp_poll_hz
                << " (requires BF MSP_OVERRIDE mode active)";
  }
  circle::bf::logInfo(started_msg.str());
  circle::bf::logInfo("bf_flight/vision: conf_threshold=", cfg.conf_threshold,
                      " iou_threshold=", cfg.iou_threshold, " max_det=", cfg.max_det,
                      " filter.min_score=", cfg.filter.min_score,
                      " filter.min_bbox_area=", cfg.filter.min_bbox_area,
                      " filter.max_aspect=", cfg.filter.max_bbox_aspect_ratio,
                      " model=", cfg.model_path);

  // Control-loop cadence is independent of the MSP wire rate (msp_set_hz). Clamp
  // to a sane band so a bad config can't spin the loop too fast (CPU) or too
  // slow (sluggish guidance). Default 200Hz preserves prior hardcoded behavior.
  constexpr double kControlLoopHzMin = 20.0;
  constexpr double kControlLoopHzMax = 500.0;
  const double requested_control_hz =
      cfg.control_loop_hz > 0.0 ? cfg.control_loop_hz : 200.0;
  const double control_hz =
      std::clamp(requested_control_hz, kControlLoopHzMin, kControlLoopHzMax);
  if (std::abs(control_hz - requested_control_hz) > 1e-6) {
    circle::bf::logWarn("control_loop_hz=", requested_control_hz,
                        " out of range [", kControlLoopHzMin, ",",
                        kControlLoopHzMax, "]; clamped to ", control_hz);
  }
  if (control_hz < msp_set_hz) {
    circle::bf::logWarn("control_loop_hz=", control_hz,
                        " is slower than msp live_publish_hz=", msp_set_hz,
                        "; staged commands will repeat on the MSP wire");
  }
  const auto control_period = std::chrono::microseconds(
      static_cast<int64_t>(std::llround(1'000'000.0 / control_hz)));
  circle::bf::logInfo("control loop ", control_hz, "Hz (period=",
                      control_period.count(), "us)");
  const auto passthrough_period = std::chrono::microseconds(
      static_cast<int64_t>(1'000'000.0 / std::max(1.0, cfg.msp.passthrough_hz)));
  auto next_control = std::chrono::steady_clock::now();
  auto next_passthrough = next_control;
  auto last_msp_poll_log = std::chrono::steady_clock::time_point{};
  auto last_cmd_diag_log = std::chrono::steady_clock::time_point{};
  circle::types::FcState cached_vehicle{};
  auto last_passthrough_warn = std::chrono::steady_clock::time_point{};
  uint64_t passthrough_ok = 0;
  uint64_t passthrough_fail = 0;
  circle::types::FrameDetection last_valid_control_detection{};
  uint64_t detection_coast_hits = 0;
  bool prev_strike_engaged = false;
  bool engaged_fresh_seen = false;
  auto last_state_ok_time = std::chrono::steady_clock::now();
  auto last_loop_time = std::chrono::steady_clock::time_point{};
  auto last_watchdog_warn = std::chrono::steady_clock::time_point{};
  uint64_t watchdog_overrun_count = 0;
  uint64_t watchdog_state_trip_count = 0;
  bool prev_algo_owns_throttle = false;
  bool throttle_handover_active = false;
  circle::types::TimestampNs throttle_handover_start_ns = 0;
  float throttle_handover_from_norm = 0.0F;

  MspPollShared msp_poll_shared;
  std::thread msp_poll_thread(mspIoThread, msp, std::ref(state_source),
                              std::ref(rate_sink), msp_io_period, attitude_divisor,
                              status_divisor, msp_passthrough_active,
                              std::ref(msp_poll_shared), std::ref(running));

  uint64_t iterations = 0;
  while (running.load()) {
    const auto loop_now = std::chrono::steady_clock::now();
    if (cfg.watchdog_enabled && last_loop_time.time_since_epoch().count() != 0) {
      const double iter_gap_s =
          std::chrono::duration<double>(loop_now - last_loop_time).count();
      if (iter_gap_s > cfg.watchdog_overrun_warn_s) {
        ++watchdog_overrun_count;
        if (last_watchdog_warn.time_since_epoch().count() == 0 ||
            loop_now - last_watchdog_warn >= std::chrono::seconds(1)) {
          last_watchdog_warn = loop_now;
          circle::bf::logWarn("bf_flight/watchdog: control loop overrun ",
                              iter_gap_s, "s (count=", watchdog_overrun_count, ")");
        }
      }
    }
    last_loop_time = loop_now;
    if (!param_reader_open) {
      param_reader_open = param_reader.open(circle::ipc::kParamBlockShmName);
    } else {
      std::string update_json;
      if (param_reader.readLatestJson(update_json)) {
        try {
          if (controller.applyParamUpdateJson(update_json)) {
            publishParamsSnapshot();
            circle::bf::logInfo("bf_flight: applied param update ", update_json);
          } else {
            circle::bf::logWarn("bf_flight: param update ignored (unknown key) ",
                                update_json);
          }
        } catch (const std::exception& ex) {
          circle::bf::logWarn("bf_flight: rejected param update: ", ex.what());
        }
      }
    }

    const uint64_t now_ns = circle::types::monotonicNowNs();
    circle::types::FrameDetection control_detection;
    float capture_time_ms = 0.0F;
    float decode_time_ms = 0.0F;
    circle::perception::VisionPipelineTiming vision_timing;
    uint64_t pipeline_seq = 0;
    {
      std::lock_guard<std::mutex> lk(perception_shared.mu);
      control_detection = perception_shared.latest_detection;
      capture_time_ms = perception_shared.latest_capture_time_ms;
      decode_time_ms = perception_shared.latest_decode_time_ms;
      vision_timing = perception_shared.latest_vision_timing;
      pipeline_seq = perception_shared.latest_pipeline_seq;
    }
    if (control_detection.valid) {
      last_valid_control_detection = control_detection;
    } else if (cfg.detection_coast_s > 0.0 && last_valid_control_detection.valid) {
      const double gap_s = circle::types::secondsBetween(
          last_valid_control_detection.capture_ns, control_detection.capture_ns);
      if (gap_s >= 0.0 && gap_s <= cfg.detection_coast_s) {
        control_detection = last_valid_control_detection;
        ++detection_coast_hits;
      }
    }

    {
      const auto now = std::chrono::steady_clock::now();
      circle::bf::MspPollStats poll;
      bool poll_fresh = false;
      {
        std::lock_guard<std::mutex> lk(msp_poll_shared.mu);
        if (msp_poll_shared.have) {
          cached_vehicle = msp_poll_shared.vehicle;
        }
        poll = msp_poll_shared.stats;
        poll_fresh =
            msp_poll_shared.have &&
            (now - msp_poll_shared.last_update) < std::chrono::milliseconds(200);
      }
      if (poll_fresh && poll.status_age_ms >= 0.0 && poll.status_age_ms < 250.0) {
        last_state_ok_time = now;
      }
      if (cfg.msp.passthrough_log &&
          (last_msp_poll_log.time_since_epoch().count() == 0 ||
           now - last_msp_poll_log >=
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(cfg.msp.passthrough_log_interval_s)))) {
        last_msp_poll_log = now;
        const auto fmtFrame = [](bool ok, double age_ms) {
          std::ostringstream f;
          f << (ok ? 1 : 0) << "(age=";
          if (age_ms < 0.0) {
            f << "never)";
          } else {
            f << std::fixed << std::setprecision(0) << age_ms << "ms)";
          }
          return f.str();
        };
        const std::string rc_field =
            rate_sink.needsRcPoll()
                ? fmtFrame(poll.last_rc_ok, poll.rc_age_ms)
                : "frozen(override)";
        std::ostringstream oss;
        oss << "bf_flight/msp/poll: ok=" << poll.ok << " fail=" << poll.fail
            << " status=" << fmtFrame(poll.last_status_ok, poll.status_age_ms)
            << " att=" << fmtFrame(poll.last_attitude_ok, poll.attitude_age_ms)
            << " rc=" << rc_field << " mode_flags=0x" << std::hex << poll.mode_flags
            << " want_mask=0x" << poll.override_mode_flag << std::dec
            << " override=" << (poll.override_active ? 1 : 0)
            << " override_eff=" << (rate_sink.mspOverrideActive() ? 1 : 0)
            << " override_raw=" << (rate_sink.mspOverrideRawActive() ? 1 : 0)
            << " override_grace=" << (rate_sink.mspOverrideGraceHoldActive() ? 1 : 0);
        if (!poll.last_error.empty()) {
          oss << " err=" << poll.last_error;
        }
        circle::bf::logInfo(oss.str());

        const auto perf_snap = perception_shared.perf.snapshot(
            pipeline_seq, rate_sink.mspOverrideActive() || cfg.dry_run);
        logMspIoStats(msp, perf_snap, msp_poll_hz, msp_set_hz);
      }
    }

    const bool override_active = rate_sink.mspOverrideActive();
    const bool override_raw = rate_sink.mspOverrideRawActive();
    const bool override_grace = rate_sink.mspOverrideGraceHoldActive();
    const bool strike_engaged = override_active || cfg.dry_run;

    BfControlContext ctx;
    ctx.now_ns = now_ns;
    ctx.detection = control_detection;
    ctx.vehicle = cached_vehicle;
    ctx.mode_active = strike_engaged;
    ctx.override_active = override_active;
    ctx.intrinsics = cfg.intrinsics;
    ctx.image_width = cfg.camera.output_width;
    ctx.image_height = cfg.camera.output_height;

    if (strike_engaged && !prev_strike_engaged) {
      const bool had_latch = rate_sink.lastLatchedPhysicalThrottle().valid;
      if (!had_latch) {
        controller.onEngageRisingEdge(ctx);
        engaged_fresh_seen = false;
        circle::bf::logInfo(
            "bf_flight: strike engaged (override/dry_run rising edge) ->"
            " controller.reset()");
      } else {
        circle::bf::logInfo(
            "bf_flight: strike re-engaged with existing OVERRIDE latch ->"
            " skip controller.reset()");
      }
    }
    prev_strike_engaged = strike_engaged;

    if (strike_engaged && !engaged_fresh_seen && control_detection.valid) {
      const double det_age_s =
          circle::types::secondsBetween(control_detection.capture_ns, now_ns);
      if (det_age_s >= 0.0 && det_age_s <= cfg.engage_detection_fresh_timeout_s) {
        engaged_fresh_seen = true;
      }
    }

    uint64_t grab_ok_total = 0;
    if (use_zero_copy_pipeline && zero_copy_runtime) {
      grab_ok_total = zero_copy_runtime->stats().grab_ok;
      const auto zstats = zero_copy_runtime->stats();
      circle::debug::PipelinePipeStats pipe;
      pipe.zero_copy = true;
      pipe.slot_count = zero_copy_runtime->slotCount();
      pipe.slot_busy = zstats.slot_busy_drops;
      pipe.ready_drop = zstats.ready_drops;
      perception_shared.perf.setPipeStats(pipe);
    } else {
      circle::debug::PipelinePipeStats pipe;
      pipe.zero_copy = false;
      perception_shared.perf.setPipeStats(pipe);
      std::lock_guard<std::mutex> lk(perception_shared.mu);
      grab_ok_total = perception_shared.camera_grab_ok;
    }
    perception_shared.perf.updateProdFpsFromTotal(grab_ok_total,
                                                  std::chrono::steady_clock::now());

    const auto exec_start = std::chrono::steady_clock::now();
    const uint64_t wire_count_before = rate_sink.wirePublishCount();
    const BfControlResult result = controller.update(ctx);
    const auto exec_end = std::chrono::steady_clock::now();

    const bool state_watchdog_tripped =
        cfg.watchdog_enabled &&
        std::chrono::duration<double>(exec_end - last_state_ok_time).count() >
            cfg.watchdog_state_timeout_s;
    if (state_watchdog_tripped) {
      ++watchdog_state_trip_count;
    }
    const bool may_command =
        strike_engaged &&
        (!cfg.require_armed_to_command || result.safety.armed);
    const BfPublishMode publish_mode = decideBfPublish(
        result.command, may_command, override_active, cfg.dry_run,
        state_watchdog_tripped, cfg.engage_require_fresh_detection,
        engaged_fresh_seen);
    const bool algorithm_command_active =
        publish_mode == BfPublishMode::Algorithm;
    const bool force_level_command_active =
        publish_mode == BfPublishMode::LevelOnly;
    const bool base_command_gate =
        may_command && !cfg.dry_run && override_active && !state_watchdog_tripped;

    const bool algo_owns_throttle = algorithm_command_active;
    if (algo_owns_throttle != prev_algo_owns_throttle && base_command_gate &&
        cfg.throttle_handover_s > 0.0) {
      const float hover_norm = controller.hoverThrottleNorm();
      const auto last_thr = rate_sink.lastSentThrottle();
      float from = 0.0F;
      const char* from_src = "none";
      bool from_ok = false;
      if (algo_owns_throttle) {
        // pilot/physical-hold -> algorithm takeover. Pick the blend START with
        // priority: reliable pre-OVERRIDE latched physical throttle, else hover,
        // else last sent wire throttle. lastSentThrottle is unreliable here (it
        // can be stale or a masked-channel garbage near rc_min => norm~0), so it
        // is only the last resort. Then floor at hover so the handover never
        // starts below hover (prevents the throttle drop-to-0 at the switch).
        const auto latched = rate_sink.lastLatchedPhysicalThrottle();
        if (latched.valid && std::isfinite(latched.norm)) {
          from = latched.norm;
          from_src = "latched_physical";
          from_ok = true;
        } else if (hover_norm > 0.0F) {
          from = hover_norm;
          from_src = "hover";
          from_ok = true;
        } else if (last_thr.valid && std::isfinite(last_thr.norm)) {
          from = last_thr.norm;
          from_src = "last_sent";
          from_ok = true;
        }
      } else {
        // algorithm -> pilot/physical-hold. Ramp down FROM the algorithm's last
        // sent throttle so it decays smoothly toward the physical/latched value.
        if (last_thr.valid && std::isfinite(last_thr.norm)) {
          from = last_thr.norm;
          from_src = "last_sent_release";
          from_ok = true;
        }
      }
      if (from_ok) {
        // Single clamp does both jobs: floor + range. On ENGAGE the lower bound
        // is hover (the blend start never drops below hover -> no throttle
        // drop-to-0 at the switch); on RELEASE it is 0 since we ramp back down
        // toward the pilot/physical throttle.
        const float from_lo = algo_owns_throttle ? hover_norm : 0.0F;
        throttle_handover_active = true;
        throttle_handover_start_ns = now_ns;
        throttle_handover_from_norm = std::clamp(from, from_lo, 1.0F);
        circle::bf::logInfo(
            "bf_flight/throttle_handover: ",
            (algo_owns_throttle ? "ENGAGE" : "RELEASE"),
            " from_norm=", throttle_handover_from_norm, " src=", from_src,
            " hover=", hover_norm, " latched_valid=",
            (rate_sink.lastLatchedPhysicalThrottle().valid ? 1 : 0),
            " last_sent_valid=", (last_thr.valid ? 1 : 0),
            " dur_s=", cfg.throttle_handover_s);
      }
    }
    prev_algo_owns_throttle = algo_owns_throttle;

    float blend_alpha = 1.0F;
    if (throttle_handover_active && base_command_gate) {
      const double elapsed = circle::types::secondsBetween(
          throttle_handover_start_ns, now_ns);
      if (elapsed >= cfg.throttle_handover_s || elapsed < 0.0) {
        throttle_handover_active = false;
      } else {
        blend_alpha = static_cast<float>(elapsed / cfg.throttle_handover_s);
      }
    } else {
      throttle_handover_active = false;
    }
    const bool handover_throttle_active = throttle_handover_active;
    auto blendThrust = [&](float target_norm) {
      return blendThrottleHandover(throttle_handover_from_norm, target_norm,
                                   blend_alpha);
    };

    if (algorithm_command_active) {
      circle::types::RateCommand cmd = result.rates;
      if (handover_throttle_active) {
        cmd.thrust_z = blendThrust(result.rates.thrust_z);
      }
      rate_sink.publishRates(cmd, result.safety);
    } else if (force_level_command_active) {
      circle::types::RateCommand cmd = result.rates;
      if (handover_throttle_active) {
        const auto latched = rate_sink.lastLatchedPhysicalThrottle();
        cmd.thrust_z = latched.valid ? blendThrust(latched.norm) : 0.0F;
      } else {
        cmd.thrust_z = 0.0F;
      }
      rate_sink.publishRates(cmd, result.safety);
    } else if (handover_throttle_active) {
      const auto latched = rate_sink.lastLatchedPhysicalThrottle();
      if (latched.valid) {
        circle::types::RateCommand cmd{};
        cmd.thrust_z = blendThrust(latched.norm);
        rate_sink.publishRates(cmd, result.safety);
      } else if (!cfg.dry_run) {
        rate_sink.publishOverridePhysicalHold();
      }
    } else if (!cfg.dry_run) {
      rate_sink.publishOverridePhysicalHold();
    }
    const auto publish_end = std::chrono::steady_clock::now();
    const uint64_t wire_count_after = rate_sink.wirePublishCount();

    const uint64_t det_seq = control_detection.seq;
    const bool seq_aligned =
        det_seq > 0 && pipeline_seq > 0 && det_seq == pipeline_seq;
    if (seq_aligned) {
      (void)perception_shared.perf.tryNoteControlComplete(det_seq, exec_start,
                                                          exec_end);
    }
    if (algorithm_command_active) {
      if (seq_aligned && wire_count_after > wire_count_before) {
        if (perception_shared.perf.tryNoteWireComplete(det_seq, exec_start,
                                                       publish_end)) {
          perception_shared.perf.noteMspEvent();
        }
      }
    }

    if (msp_passthrough_active && exec_end >= next_passthrough) {
      if (rate_sink.publishRcPassthrough()) {
        ++passthrough_ok;
      } else {
        ++passthrough_fail;
        const auto now = exec_end;
        if (last_passthrough_warn.time_since_epoch().count() == 0 ||
            now - last_passthrough_warn >= std::chrono::seconds(5)) {
          circle::bf::logWarn("bf_flight/msp: RC passthrough failed (ok=",
                              passthrough_ok, " fail=", passthrough_fail, ") ",
                              rate_sink.lastPassthroughError());
          last_passthrough_warn = now;
        }
      }
      next_passthrough = exec_end + passthrough_period;
    }

    {
      const auto diag_now = exec_end;
      if (diag_now - last_cmd_diag_log >= std::chrono::seconds(1)) {
        last_cmd_diag_log = diag_now;
        constexpr float kRateEps = 1.0e-4F;
        const bool rates_active =
            std::fabs(result.rates.roll_rate_rad_s) > kRateEps ||
            std::fabs(result.rates.pitch_rate_rad_s) > kRateEps ||
            std::fabs(result.rates.yaw_rate_rad_s) > kRateEps;
        const auto wire = rate_sink.lastSentWire();
        if (!cfg.dry_run && (rates_active || !may_command || wire.valid)) {
          std::ostringstream oss;
          oss << "bf_flight/cmd: state=" << result.state_code
              << " armed=" << (result.safety.armed ? 1 : 0)
              << " may_send=" << (may_command ? 1 : 0)
              << " override=" << (override_active ? 1 : 0)
              << " override_raw=" << (override_raw ? 1 : 0)
              << " override_grace=" << (override_grace ? 1 : 0)
              << " algo_active=" << (algorithm_command_active ? 1 : 0)
              << " lvl_only=" << (force_level_command_active ? 1 : 0)
              << " thr_handover=" << (handover_throttle_active ? 1 : 0)
              << " roll_sp=" << result.rates.roll_rate_rad_s
              << " pitch_sp=" << result.rates.pitch_rate_rad_s
              << " yaw_sp=" << result.rates.yaw_rate_rad_s
              << " thrust_z=" << result.rates.thrust_z
              << " det_valid=" << (control_detection.valid ? 1 : 0)
              << " coast_hits=" << detection_coast_hits
              << " engaged=" << (strike_engaged ? 1 : 0)
              << " fresh_seen=" << (engaged_fresh_seen ? 1 : 0)
              << " wdog=" << (state_watchdog_tripped ? 1 : 0)
              << " wire_valid=" << (wire.valid ? 1 : 0);
          if (wire.valid) {
            oss << " wireR=" << wire.roll_pwm << " wireP=" << wire.pitch_pwm
                << " wireT=" << wire.throttle_pwm << " wireY=" << wire.yaw_pwm;
          }
          if (rates_active && !result.safety.armed) {
            oss << " **RATES_TELEMETRY_ONLY(disarmed)**";
          }
          if (result.safety.armed && !override_active) {
            const uint32_t mode_flags = rate_sink.cachedModeFlags();
            const uint32_t want_mask = rate_sink.overrideModeFlag();
            oss << " **NEED_MSP_OVERRIDE** mode_flags=0x" << std::hex << mode_flags
                << " want_mask=0x" << want_mask << std::dec
                << " match=" << ((mode_flags & want_mask) != 0U ? 1 : 0);
          }
          if (state_watchdog_tripped) {
            oss << " **STATE_WATCHDOG_TRIPPED(stale_msp)** trips="
                << watchdog_state_trip_count;
          }
          if (override_active && cfg.engage_require_fresh_detection &&
              !engaged_fresh_seen) {
            oss << " **WAIT_FRESH_DETECTION(physical_hold)**";
          }
          circle::bf::logInfo(oss.str());
        }
      }
    }

    const float executor_ms =
        std::chrono::duration<float, std::milli>(exec_end - exec_start).count();

    circle::debug::PreviewOverlayContext overlay;
    overlay.source = "bf";
    overlay.mode_tag = controller.modeTag();
    overlay.state = controller.stateName(result);
    overlay.armed = result.safety.armed;
    overlay.has_target = result.has_target;
    overlay.detection.valid = control_detection.valid;
    overlay.detection.cx = control_detection.detection.cx;
    overlay.detection.cy = control_detection.detection.cy;
    overlay.detection.width = control_detection.detection.width;
    overlay.detection.height = control_detection.detection.height;
    overlay.detection.score = control_detection.detection.score;
    overlay.detection.class_id = control_detection.detection.class_id;
    overlay.detection.class_name = control_detection.detection.class_name;
    overlay.detection.seq = control_detection.seq;
    overlay.detection.capture_ns = static_cast<int64_t>(control_detection.capture_ns);
    overlay.image_ex = result.image_ex;
    overlay.image_ey = result.image_ey;
    overlay.roll_rate_rad_s = result.rates.roll_rate_rad_s;
    overlay.pitch_rate_rad_s = result.rates.pitch_rate_rad_s;
    overlay.yaw_rate_rad_s = result.rates.yaw_rate_rad_s;
    const float throttle_algo = throttleCmdNorm(result.rates, result.safety.armed);
    const auto wire_throttle = rate_sink.lastSentThrottle();
    const auto latched_throttle = rate_sink.lastLatchedPhysicalThrottle();
    // T_cmd = the throttle actually written to the MSP wire. Show it for the
    // whole OVERRIDE session (algo, handover blend AND physical hold all flush a
    // wire frame every cycle) so the handover throttle curve is continuous and
    // the blend start/floor is visible. Hidden only when not overriding (no wire
    // is sent, so lastSentThrottle would be stale).
    const bool wire_cmd_shown = wire_throttle.valid && override_active;
    overlay.thrust_z = throttle_algo;
    overlay.throttle_algo_norm = throttle_algo;
    if (wire_cmd_shown) {
      overlay.throttle_cmd_norm = wire_throttle.norm;
    }
    if (ctx.vehicle.valid) {
      overlay.vehicle_roll_rad = ctx.vehicle.roll_rad;
      overlay.vehicle_pitch_rad = ctx.vehicle.pitch_rad;
      overlay.vehicle_yaw_rad = ctx.vehicle.yaw_rad;
      if (std::isfinite(ctx.vehicle.throttle_norm)) {
        overlay.vehicle_throttle_pwm = ctx.vehicle.throttle_pwm;
        overlay.vehicle_throttle_norm = ctx.vehicle.throttle_norm;
      }
    }
    if (algorithm_command_active && wire_throttle.valid) {
      overlay.vehicle_throttle_pwm = wire_throttle.pwm;
      overlay.vehicle_throttle_norm = wire_throttle.norm;
    } else if (latched_throttle.valid && override_active) {
      overlay.vehicle_throttle_pwm = latched_throttle.pwm;
      overlay.vehicle_throttle_norm = latched_throttle.norm;
    }
    overlay.capture_time_ms = capture_time_ms;
    overlay.decode_time_ms = decode_time_ms;
    overlay.preprocess_time_ms = vision_timing.preprocess_time_ms;
    overlay.inference_time_ms = vision_timing.inference_time_ms;
    overlay.postprocess_time_ms = vision_timing.postprocess_time_ms;
    overlay.cam_pipeline_ms = vision_timing.total_time_ms;
    overlay.executor_ms = executor_ms;
    controller.fillOverlay(overlay, result);
    {
      const bool perf_wire = algorithm_command_active;
      const uint64_t perf_seq =
          (det_seq > 0 && pipeline_seq > 0 && det_seq == pipeline_seq) ? det_seq
                                                                       : pipeline_seq;
      const auto perf_snap = perception_shared.perf.snapshot(
          perf_seq > 0 ? perf_seq : det_seq, perf_wire);
      circle::debug::applyPipelinePerfSnapshot(overlay, perf_snap);
      std::lock_guard<std::mutex> lk(perception_shared.mu);
      perception_shared.latest_control_overlay = overlay;
    }

    circle::ipc::StrikeTelemetrySample sample;
    sample.stamp_ns = now_ns;
    sample.roll_rate_sp = result.rates.roll_rate_rad_s;
    sample.pitch_rate_sp = result.rates.pitch_rate_rad_s;
    sample.yaw_rate_sp = result.rates.yaw_rate_rad_s;
    sample.thrust_z = throttle_algo;
    sample.throttle_algo_norm = throttle_algo;
    if (wire_cmd_shown) {
      sample.throttle_cmd_norm = wire_throttle.norm;
      sample.throttle_cmd_valid = 1;
    } else {
      sample.throttle_cmd_norm = std::numeric_limits<float>::quiet_NaN();
      sample.throttle_cmd_valid = 0;
    }
    sample.dry_run_passthrough = msp_passthrough_active ? 1 : 0;
    sample.ex = result.image_ex;
    sample.ey = result.image_ey;
    sample.state = result.state_code;
    sample.has_target = result.has_target ? 1 : 0;
    sample.armed = result.safety.armed ? 1 : 0;
    if (ctx.vehicle.valid) {
      sample.vehicle_valid = 1;
      sample.vehicle_roll_rad = ctx.vehicle.roll_rad;
      sample.vehicle_pitch_rad = ctx.vehicle.pitch_rad;
      sample.vehicle_yaw_rad = ctx.vehicle.yaw_rad;
      if (std::isfinite(ctx.vehicle.throttle_norm)) {
        sample.throttle_pwm = ctx.vehicle.throttle_pwm;
        sample.throttle_norm = ctx.vehicle.throttle_norm;
      }
    }
    if (algorithm_command_active && wire_throttle.valid) {
      sample.throttle_pwm = wire_throttle.pwm;
      sample.throttle_norm = wire_throttle.norm;
    } else if (latched_throttle.valid && override_active) {
      sample.throttle_pwm = latched_throttle.pwm;
      sample.throttle_norm = latched_throttle.norm;
    }
    sample.detection_valid = control_detection.valid ? 1 : 0;
    sample.msp_override_active = override_active ? 1 : 0;
    sample.controller_kind =
        (std::strcmp(controller.modeTag(), "target_strike_png") == 0) ? 1 : 0;
    // Note: vehicle_*_rate_rad_s are intentionally left unset on BF; BfStateSource
    // provides attitude only (no measured body rates), so seriesJson emits null.
    controller.fillTelemetry(sample, result);

    telemetry.publish(sample);

    ++iterations;
    if (max_iterations > 0 && iterations >= max_iterations) {
      running.store(false);
      break;
    }
    next_control += control_period;
    std::this_thread::sleep_until(next_control);
  }

  running.store(false);
  perception_shared.cv.notify_all();
  if (zero_copy_runtime) {
    zero_copy_runtime->stop();
  }
  if (camera_thread.joinable()) {
    camera_thread.join();
  }
  if (infer_thread.joinable()) {
    infer_thread.join();
  }
  if (preview_thread.joinable()) {
    preview_thread.join();
  }
  if (msp_poll_thread.joinable()) {
    msp_poll_thread.join();
  }
  preview_writer.close();
  if (!zero_copy_runtime) {
    camera_source.stop();
  }
  return 0;
}

}  // namespace circle::bf::runtime
