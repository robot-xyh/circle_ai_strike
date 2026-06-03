#include "circle/perception/parallel_vision_pipeline.hpp"

#include <chrono>
#include <iostream>
#include <mutex>

#include "circle/types/time.hpp"
#include "circle/vision/yolo_postprocess.hpp"

namespace circle::perception {

namespace {

#if CIRCLE_PERCEPTION_USE_RKNN
int coreMaskForSlot(const VisionPipelineConfig& config, int slot_index) {
  if (!config.rknn_core_masks.empty()) {
    const size_t idx =
        static_cast<size_t>(slot_index) % config.rknn_core_masks.size();
    return config.rknn_core_masks[idx];
  }
  return config.rknn_core_mask;
}
#endif

float detectionArea(const circle::types::Detection& d) {
  return std::max(0.0F, d.width) * std::max(0.0F, d.height);
}

float detectionAspect(const circle::types::Detection& d) {
  const float w = std::max(1.0F, d.width);
  const float h = std::max(1.0F, d.height);
  return std::max(w, h) / std::min(w, h);
}

void logYoloAndTargetSelection(
    const std::vector<circle::vision::YoloDetection>& yolo_dets,
    const std::vector<circle::types::Detection>& typed,
    const circle::vision::DetectionFilterOutput& filtered,
    const circle::vision::DetectionFilterParams& params) {
  static std::mutex mu;
  static auto last_log = std::chrono::steady_clock::time_point{};
  const auto now = std::chrono::steady_clock::now();
  if (last_log.time_since_epoch().count() != 0 &&
      now - last_log < std::chrono::seconds(1)) {
    return;
  }
  std::lock_guard<std::mutex> lk(mu);
  if (last_log.time_since_epoch().count() != 0 &&
      now - last_log < std::chrono::seconds(1)) {
    return;
  }
  last_log = now;

  rknnLog(RknnLogLevel::Info,
          "zero_copy/yolo: raw=%zu min_score=%.3f target_class=%s "
          "min_area=%.1f max_ratio=%.3f",
          yolo_dets.size(), params.min_score, params.target_class_name.c_str(),
          params.min_bbox_area, params.max_bbox_aspect_ratio);
  const size_t raw_limit = std::min<size_t>(yolo_dets.size(), 5);
  for (size_t i = 0; i < raw_limit; ++i) {
    const auto& y = yolo_dets[i];
    rknnLog(RknnLogLevel::Info,
            "zero_copy/yolo/det: i=%zu cls=%d score=%.3f "
            "x1=%.1f y1=%.1f x2=%.1f y2=%.1f w=%.1f h=%.1f",
            i, y.class_id, y.score, y.x1, y.y1, y.x2, y.y2, y.x2 - y.x1,
            y.y2 - y.y1);
  }

  rknnLog(RknnLogLevel::Info, "zero_copy/target_select: candidates=%zu best=%d",
          typed.size(), filtered.best_index);
  const size_t cand_limit = std::min<size_t>(typed.size(), 8);
  for (size_t i = 0; i < cand_limit && i < filtered.results.size(); ++i) {
    const auto& d = typed[i];
    const auto& r = filtered.results[i];
    rknnLog(RknnLogLevel::Info,
            "zero_copy/target_select/cand: i=%zu status=%s cls=%s(%d) "
            "score=%.3f cx=%.1f cy=%.1f w=%.1f h=%.1f area=%.1f "
            "ratio=%.3f%s",
            i, circle::vision::filterStatusToString(r.status),
            d.class_name.c_str(), d.class_id, d.score, d.cx, d.cy, d.width,
            d.height, detectionArea(d), detectionAspect(d),
            static_cast<int>(i) == filtered.best_index ? " **BEST**" : "");
  }
}

}  // namespace

ParallelVisionPipeline::ParallelVisionPipeline(VisionPipelineConfig config)
    : config_(std::move(config)) {}

bool ParallelVisionPipeline::initialize(int slot_count) {
  engines_.clear();
  initialized_ = false;
  if (slot_count <= 0) {
    return false;
  }
  if (!config_.detection_file.empty()) {
    initialized_ = true;
    return true;
  }
#if CIRCLE_PERCEPTION_USE_RKNN
  if (config_.model_path.empty()) {
    return false;
  }
  letterbox_ = computeLetterbox(config_.pipeline_out_w, config_.pipeline_out_h,
                                config_.letterbox_w, config_.letterbox_h);
  for (int i = 0; i < slot_count; ++i) {
    const int core_mask = coreMaskForSlot(config_, i);
    auto engine = std::make_unique<RknnEngine>();
    if (!engine->init(config_.model_path, core_mask,
                      /*zero_copy_input=*/true, /*zero_copy_output=*/false)) {
      engines_.clear();
      return false;
    }
    std::cerr << "zero_copy/rknn: slot=" << i << " core_mask=" << core_mask
              << '\n';
    engines_.push_back(std::move(engine));
  }
  initialized_ = true;
  return true;
#else
  (void)slot_count;
  return false;
#endif
}

RknnEngine& ParallelVisionPipeline::engine(int slot_id) {
  return *engines_.at(static_cast<size_t>(slot_id));
}

const RknnEngine& ParallelVisionPipeline::engine(int slot_id) const {
  return *engines_.at(static_cast<size_t>(slot_id));
}

int ParallelVisionPipeline::slotDmaFd(int slot_id) const {
#if CIRCLE_PERCEPTION_USE_RKNN
  return engine(slot_id).getInputDmaFd();
#else
  (void)slot_id;
  return -1;
#endif
}

uint32_t ParallelVisionPipeline::inputBufferSize() const {
#if CIRCLE_PERCEPTION_USE_RKNN
  if (engines_.empty()) {
    return 0;
  }
  return engines_.front()->getInputBufferSize();
#else
  return 0;
#endif
}

circle::vision::DetectionTrackHint ParallelVisionPipeline::trackHintSnapshot()
    const {
  std::lock_guard<std::mutex> lk(track_mu_);
  return track_hint_;
}

void ParallelVisionPipeline::updateTrackHint(
    const circle::types::Detection* best) {
  std::lock_guard<std::mutex> lk(track_mu_);
  if (best != nullptr) {
    track_hint_.valid = true;
    track_hint_.cx = static_cast<double>(best->cx);
    track_hint_.cy = static_cast<double>(best->cy);
    track_hint_.area = std::max(1.0F, best->width) * std::max(1.0F, best->height);
    track_hint_misses_ = 0;
    return;
  }
  if (track_hint_.valid &&
      ++track_hint_misses_ > config_.track_hint_max_misses) {
    track_hint_ = circle::vision::DetectionTrackHint{};
    track_hint_misses_ = 0;
  }
}

circle::types::Detection ParallelVisionPipeline::yoloToDetection(
    const circle::vision::YoloDetection& y) {
  circle::types::Detection d;
  d.cx = (y.x1 + y.x2) * 0.5F;
  d.cy = (y.y1 + y.y2) * 0.5F;
  d.width = y.x2 - y.x1;
  d.height = y.y2 - y.y1;
  d.score = y.score;
  d.class_id = y.class_id;
  d.class_name = "UAV";
  return d;
}

bool ParallelVisionPipeline::postprocessEngine(
    RknnEngine& engine,
    int src_w,
    int src_h,
    VisionPipelineTiming* timing,
    circle::types::FrameDetection& out,
    std::chrono::steady_clock::time_point total_start) {
#if !CIRCLE_PERCEPTION_USE_RKNN
  (void)engine;
  (void)src_w;
  (void)src_h;
  (void)timing;
  (void)out;
  (void)total_start;
  return false;
#else
  const int n_out = engine.numOutputs();
  if (n_out <= 0) {
    out.valid = false;
    return false;
  }

  const auto post_start = std::chrono::steady_clock::now();
  std::vector<circle::vision::YoloDetection> yolo_dets;
  if (n_out == 1) {
    yolo_dets = circle::vision::yoloPostprocess(
        engine.getOutputData(0), engine.getOutputShape(0), engine.getOutputDims(0),
        src_w, src_h, letterbox_.scale, letterbox_.top_pad, letterbox_.left_pad,
        config_.conf_threshold, config_.iou_threshold, config_.max_det);
  } else {
    std::vector<const float*> outputs(static_cast<size_t>(n_out));
    std::vector<const uint32_t*> shapes(static_cast<size_t>(n_out));
    std::vector<int> dims(static_cast<size_t>(n_out));
    std::unique_ptr<bool[]> is_nhwc(new bool[static_cast<size_t>(n_out)]);
    for (int i = 0; i < n_out; ++i) {
      outputs[static_cast<size_t>(i)] = engine.getOutputData(i);
      shapes[static_cast<size_t>(i)] = engine.getOutputShape(i);
      dims[static_cast<size_t>(i)] = engine.getOutputDims(i);
      is_nhwc[static_cast<size_t>(i)] = engine.getOutputIsNHWC(i);
    }
    yolo_dets = circle::vision::yoloPostprocessMultihead(
        outputs.data(), shapes.data(), dims.data(), is_nhwc.get(), n_out, src_w,
        src_h, letterbox_.scale, letterbox_.top_pad, letterbox_.left_pad,
        config_.conf_threshold, config_.iou_threshold, config_.max_det,
        config_.letterbox_w, config_.letterbox_h);
  }

  std::vector<circle::types::Detection> typed;
  typed.reserve(yolo_dets.size());
  for (const auto& y : yolo_dets) {
    typed.push_back(yoloToDetection(y));
  }
  circle::vision::DetectionTrackHint hint;
  if (config_.filter.temporal_gating_enabled) {
    hint = trackHintSnapshot();
  }
  const auto filtered = circle::vision::filterDetections(
      typed, config_.filter,
      config_.filter.temporal_gating_enabled ? &hint : nullptr);
  updateTrackHint(filtered.best_index >= 0
                      ? &typed[static_cast<size_t>(filtered.best_index)]
                      : nullptr);
  logYoloAndTargetSelection(yolo_dets, typed, filtered, config_.filter);
  const auto post_end = std::chrono::steady_clock::now();
  if (timing) {
    timing->postprocess_time_ms =
        std::chrono::duration<float, std::milli>(post_end - post_start).count();
    timing->total_time_ms =
        std::chrono::duration<float, std::milli>(post_end - total_start).count();
    timing->raw_detections = static_cast<int>(typed.size());
    timing->accepted_index = filtered.best_index;
  }
  if (filtered.best_index < 0) {
    out.valid = false;
    return false;
  }
  out.detection = typed[static_cast<size_t>(filtered.best_index)];
  out.valid = true;
  return true;
#endif
}

InferSlotResult ParallelVisionPipeline::inferPreparedSlot(
    int slot_id,
    int src_w,
    int src_h,
    uint64_t seq,
    circle::types::TimestampNs capture_ns,
    circle::types::FrameDetection& out,
    VisionPipelineTiming* timing) {
  if (timing) {
    *timing = {};
  }
  if (!initialized_ || slot_id < 0 ||
      static_cast<size_t>(slot_id) >= engines_.size()) {
    out.valid = false;
    return InferSlotResult::RknnFailed;
  }
  out.seq = seq;
  out.capture_ns = capture_ns;
  out.infer_ns = circle::types::monotonicNowNs();
  out.receive_ns = out.infer_ns;
  out.intrinsics = config_.intrinsics;

#if !CIRCLE_PERCEPTION_USE_RKNN
  (void)src_w;
  (void)src_h;
  out.valid = false;
  return InferSlotResult::RknnFailed;
#else
  const auto total_start = std::chrono::steady_clock::now();
  auto& engine = *engines_[static_cast<size_t>(slot_id)];
  const auto inf_start = std::chrono::steady_clock::now();
  if (!engine.runZeroCopy(/*skip_input_sync=*/true)) {
    out.valid = false;
    return InferSlotResult::RknnFailed;
  }
  const auto inf_end = std::chrono::steady_clock::now();
  if (timing) {
    timing->preprocess_time_ms = 0.0F;
    timing->inference_time_ms =
        std::chrono::duration<float, std::milli>(inf_end - inf_start).count();
  }
  if (postprocessEngine(engine, src_w, src_h, timing, out, total_start)) {
    return InferSlotResult::Detection;
  }
  return InferSlotResult::NoDetection;
#endif
}

}  // namespace circle::perception
