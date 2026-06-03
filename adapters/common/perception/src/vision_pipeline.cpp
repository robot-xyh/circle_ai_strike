#include "circle/perception/vision_pipeline.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "circle/perception/rknn_engine.hpp"
#include "circle/types/time.hpp"
#include "circle/vision/yolo_postprocess.hpp"

namespace circle::perception {

namespace {

std::unordered_map<std::string, std::string> readKeyValueFile(const std::string& path) {
  std::ifstream in(path);
  std::unordered_map<std::string, std::string> kv;
  std::string line;
  while (std::getline(in, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.resize(comment);
    }
    const auto sep = line.find_first_of(":=");
    if (sep == std::string::npos) {
      continue;
    }
    auto key = line.substr(0, sep);
    auto value = line.substr(sep + 1);
    const auto trim = [](std::string s) {
      const auto begin = s.find_first_not_of(" \t\r\n");
      if (begin == std::string::npos) {
        return std::string{};
      }
      const auto end = s.find_last_not_of(" \t\r\n");
      return s.substr(begin, end - begin + 1);
    };
    kv[trim(key)] = trim(value);
  }
  return kv;
}

float getFloat(const std::unordered_map<std::string, std::string>& kv,
               const std::string& key,
               float fallback) {
  const auto it = kv.find(key);
  if (it == kv.end()) {
    return fallback;
  }
  try {
    return std::stof(it->second);
  } catch (...) {
    return fallback;
  }
}

int getInt(const std::unordered_map<std::string, std::string>& kv,
           const std::string& key,
           int fallback) {
  const auto it = kv.find(key);
  if (it == kv.end()) {
    return fallback;
  }
  try {
    return std::stoi(it->second);
  } catch (...) {
    return fallback;
  }
}

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
          "vision/yolo: raw=%zu min_score=%.3f target_class=%s "
          "min_area=%.1f max_ratio=%.3f",
          yolo_dets.size(), params.min_score, params.target_class_name.c_str(),
          params.min_bbox_area, params.max_bbox_aspect_ratio);
  const size_t raw_limit = std::min<size_t>(yolo_dets.size(), 5);
  for (size_t i = 0; i < raw_limit; ++i) {
    const auto& y = yolo_dets[i];
    rknnLog(RknnLogLevel::Info,
            "vision/yolo/det: i=%zu cls=%d score=%.3f "
            "x1=%.1f y1=%.1f x2=%.1f y2=%.1f w=%.1f h=%.1f",
            i, y.class_id, y.score, y.x1, y.y1, y.x2, y.y2, y.x2 - y.x1,
            y.y2 - y.y1);
  }

  rknnLog(RknnLogLevel::Info, "vision/target_select: candidates=%zu best=%d",
          typed.size(), filtered.best_index);
  const size_t cand_limit = std::min<size_t>(typed.size(), 8);
  for (size_t i = 0; i < cand_limit && i < filtered.results.size(); ++i) {
    const auto& d = typed[i];
    const auto& r = filtered.results[i];
    rknnLog(RknnLogLevel::Info,
            "vision/target_select/cand: i=%zu status=%s cls=%s(%d) "
            "score=%.3f cx=%.1f cy=%.1f w=%.1f h=%.1f area=%.1f "
            "ratio=%.3f%s",
            i, circle::vision::filterStatusToString(r.status),
            d.class_name.c_str(), d.class_id, d.score, d.cx, d.cy, d.width,
            d.height, detectionArea(d), detectionAspect(d),
            static_cast<int>(i) == filtered.best_index ? " **BEST**" : "");
  }
}

}  // namespace

VisionPipeline::VisionPipeline(VisionPipelineConfig config)
    : config_(std::move(config)) {}

bool VisionPipeline::initialize() {
  if (!config_.detection_file.empty()) {
    rknnLog(RknnLogLevel::Info, "[VisionPipeline] detection_file: %s",
            config_.detection_file.c_str());
    initialized_ = true;
    return true;
  }
#if CIRCLE_PERCEPTION_USE_RKNN
  if (config_.model_path.empty()) {
    rknnLog(RknnLogLevel::Error,
            "[VisionPipeline] model_path required for RKNN");
    return false;
  }
  if (!rknn_.init(config_.model_path, config_.rknn_core_mask,
                  /*zero_copy_input=*/true, /*zero_copy_output=*/false)) {
    rknnLog(RknnLogLevel::Error, "[VisionPipeline] RKNN init failed: %s",
            config_.model_path.c_str());
    return false;
  }
  letterbox_buf_.resize(static_cast<size_t>(config_.letterbox_w) *
                        static_cast<size_t>(config_.letterbox_h) * 3u);
  letterbox_ = computeLetterbox(config_.pipeline_out_w, config_.pipeline_out_h,
                               config_.letterbox_w, config_.letterbox_h);
  rknnLog(RknnLogLevel::Info,
          "[VisionPipeline] RKNN ready %dx%d letterbox scale=%.6g",
          config_.letterbox_w, config_.letterbox_h, letterbox_.scale);
  initialized_ = true;
  return true;
#else
  rknnLog(RknnLogLevel::Error,
          "[VisionPipeline] RKNN not built; set detection_file for bench");
  initialized_ = !config_.model_path.empty() ? false : true;
  return initialized_;
#endif
}

bool VisionPipeline::processFrame(const FrameReady& frame,
                                  circle::types::FrameDetection& out) {
  return processFrame(frame, out, nullptr);
}

bool VisionPipeline::processFrame(const FrameReady& frame,
                                  circle::types::FrameDetection& out,
                                  VisionPipelineTiming* timing) {
  if (timing) {
    *timing = {};
  }
  if (!initialized_) {
    return false;
  }
  out.seq = frame.seq;
  out.capture_ns = frame.capture_ns;
  out.infer_ns = circle::types::monotonicNowNs();
  out.receive_ns = out.infer_ns;
  out.intrinsics = config_.intrinsics;

  if (!config_.detection_file.empty()) {
    return processDetectionFile(out);
  }
  return processRknnFrame(frame, out, timing);
}

bool VisionPipeline::processDetectionFile(circle::types::FrameDetection& out) {
  const auto kv = readKeyValueFile(config_.detection_file);
  if (kv.empty()) {
    out.valid = false;
    return false;
  }
  circle::types::Detection raw{};
  out.valid = getInt(kv, "valid", 1) != 0;
  raw.cx = getFloat(kv, "cx", 0.0F);
  raw.cy = getFloat(kv, "cy", 0.0F);
  raw.width = getFloat(kv, "width", 0.0F);
  raw.height = getFloat(kv, "height", 0.0F);
  raw.score = getFloat(kv, "score", 1.0F);
  raw.class_id = getInt(kv, "class_id", 0);
  const auto cls = kv.find("class_name");
  raw.class_name = cls == kv.end() ? "UAV" : cls->second;

  const auto detections = std::vector<circle::types::Detection>{raw};
  const auto filtered =
      circle::vision::filterDetections(detections, config_.filter);
  if (filtered.best_index < 0) {
    out.valid = false;
    return false;
  }
  out.detection = detections[static_cast<size_t>(filtered.best_index)];
  if (out.intrinsics.fx <= 0.0F) {
    out.intrinsics.fx = getFloat(kv, "fx", 640.0F);
    out.intrinsics.fy = getFloat(kv, "fy", 640.0F);
    out.intrinsics.cx = getFloat(kv, "camera_cx", 320.0F);
    out.intrinsics.cy = getFloat(kv, "camera_cy", 240.0F);
  }
  return out.valid;
}

void VisionPipeline::updateTrackHint(const circle::types::Detection* best) {
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

circle::types::Detection VisionPipeline::yoloToDetection(
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

bool VisionPipeline::processRknnFrame(const FrameReady& frame,
                                      circle::types::FrameDetection& out,
                                      VisionPipelineTiming* timing) {
#if !CIRCLE_PERCEPTION_USE_RKNN
  (void)frame;
  (void)timing;
  out.valid = false;
  return false;
#else
  const auto total_start = std::chrono::steady_clock::now();
  if (!frame.hw_rgb || !frame.data || frame.width == 0 || frame.height == 0) {
    out.valid = false;
    return false;
  }
  const int src_w = static_cast<int>(frame.width);
  const int src_h = static_cast<int>(frame.height);
  letterbox_ = computeLetterbox(src_w, src_h, config_.letterbox_w, config_.letterbox_h);

  uint8_t* input = rknn_.getInputBuffer();
  const uint32_t input_size = rknn_.getInputBufferSize();
  bool infer_ok = false;
  const auto pre_start = std::chrono::steady_clock::now();
  if (input && input_size >= letterbox_buf_.size()) {
    letterboxRgbToBuffer(frame.data, src_w, src_h, letterbox_, config_.letterbox_w,
                         config_.letterbox_h, input);
    const auto pre_end = std::chrono::steady_clock::now();
    const auto inf_start = pre_end;
    infer_ok = rknn_.runZeroCopy(/*skip_input_sync=*/false);
    const auto inf_end = std::chrono::steady_clock::now();
    if (timing) {
      timing->preprocess_time_ms =
          std::chrono::duration<float, std::milli>(pre_end - pre_start).count();
      timing->inference_time_ms =
          std::chrono::duration<float, std::milli>(inf_end - inf_start).count();
    }
  } else {
    letterboxRgbToBuffer(frame.data, src_w, src_h, letterbox_, config_.letterbox_w,
                         config_.letterbox_h, letterbox_buf_.data());
    const auto pre_end = std::chrono::steady_clock::now();
    const auto inf_start = pre_end;
    infer_ok = rknn_.run(letterbox_buf_.data(),
                         static_cast<uint32_t>(letterbox_buf_.size()));
    const auto inf_end = std::chrono::steady_clock::now();
    if (timing) {
      timing->preprocess_time_ms =
          std::chrono::duration<float, std::milli>(pre_end - pre_start).count();
      timing->inference_time_ms =
          std::chrono::duration<float, std::milli>(inf_end - inf_start).count();
    }
  }
  if (!infer_ok) {
    out.valid = false;
    return false;
  }

  const int n_out = rknn_.numOutputs();
  if (n_out <= 0) {
    out.valid = false;
    return false;
  }

  const auto post_start = std::chrono::steady_clock::now();
  std::vector<circle::vision::YoloDetection> yolo_dets;
  if (n_out == 1) {
    yolo_dets = circle::vision::yoloPostprocess(
        rknn_.getOutputData(0), rknn_.getOutputShape(0), rknn_.getOutputDims(0),
        src_w, src_h, letterbox_.scale, letterbox_.top_pad, letterbox_.left_pad,
        config_.conf_threshold, config_.iou_threshold, config_.max_det);
  } else {
    std::vector<const float*> outputs(static_cast<size_t>(n_out));
    std::vector<const uint32_t*> shapes(static_cast<size_t>(n_out));
    std::vector<int> dims(static_cast<size_t>(n_out));
    std::unique_ptr<bool[]> is_nhwc(new bool[static_cast<size_t>(n_out)]);
    for (int i = 0; i < n_out; ++i) {
      outputs[static_cast<size_t>(i)] = rknn_.getOutputData(i);
      shapes[static_cast<size_t>(i)] = rknn_.getOutputShape(i);
      dims[static_cast<size_t>(i)] = rknn_.getOutputDims(i);
      is_nhwc[static_cast<size_t>(i)] = rknn_.getOutputIsNHWC(i);
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
  const auto filtered = circle::vision::filterDetections(
      typed, config_.filter,
      config_.filter.temporal_gating_enabled ? &track_hint_ : nullptr);
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

}  // namespace circle::perception
