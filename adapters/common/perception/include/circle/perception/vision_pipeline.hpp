#pragma once

#include <memory>
#include <string>

#include "circle/perception/camera_source.hpp"
#include "circle/perception/letterbox.hpp"
#include "circle/perception/rknn_engine.hpp"
#include "circle/types/detection.hpp"
#include "circle/vision/detection_filter.hpp"
#include "circle/vision/yolo_postprocess.hpp"

namespace circle::perception {

struct VisionPipelineConfig {
  circle::vision::DetectionFilterParams filter{};
  circle::types::CameraIntrinsics intrinsics{};
  std::string model_path;
  std::string detection_file;
  int pipeline_out_w{640};
  int pipeline_out_h{512};
  int letterbox_w{640};
  int letterbox_h{640};
  /** Fallback when rknn_core_masks is empty (7 = all RK3588 NPU cores). */
  int rknn_core_mask{7};
  /** Per-slot NPU core mask (e.g. [1, 2, 4] for triple parallel infer). */
  std::vector<int> rknn_core_masks{};
  float conf_threshold{0.25F};
  float iou_threshold{0.45F};
  int max_det{300};
  /** Frames to keep the temporal-gating anchor alive across misses (raw=0 /
   *  best=-1) before resetting to cold-start largest-area selection. At ~90fps
   *  30 ≈ 0.33s, enough to bridge brief detection gaps without anchoring stale. */
  int track_hint_max_misses{30};
};

struct VisionPipelineTiming {
  float preprocess_time_ms{0.0F};
  float inference_time_ms{0.0F};
  float postprocess_time_ms{0.0F};
  float total_time_ms{0.0F};
  int raw_detections{0};
  int accepted_index{-1};
};

class VisionPipeline {
 public:
  explicit VisionPipeline(VisionPipelineConfig config);

  bool initialize();
  bool processFrame(const FrameReady& frame, circle::types::FrameDetection& out);
  bool processFrame(const FrameReady& frame, circle::types::FrameDetection& out,
                    VisionPipelineTiming* timing);

 private:
  bool processDetectionFile(circle::types::FrameDetection& out);
  bool processRknnFrame(const FrameReady& frame, circle::types::FrameDetection& out,
                        VisionPipelineTiming* timing);
  static circle::types::Detection yoloToDetection(const circle::vision::YoloDetection& y);
  /** Advance the temporal-gating anchor: refresh on a hit, age out on misses. */
  void updateTrackHint(const circle::types::Detection* best);

  VisionPipelineConfig config_;
  bool initialized_{false};
  LetterboxParams letterbox_{};
  circle::vision::DetectionTrackHint track_hint_{};
  int track_hint_misses_{0};
#if CIRCLE_PERCEPTION_USE_RKNN
  RknnEngine rknn_;
  std::vector<uint8_t> letterbox_buf_;
#endif
};

}  // namespace circle::perception
