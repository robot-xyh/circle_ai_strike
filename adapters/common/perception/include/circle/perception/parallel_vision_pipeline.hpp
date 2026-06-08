#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "circle/perception/letterbox.hpp"
#include "circle/perception/rknn_engine.hpp"
#include "circle/perception/vision_pipeline.hpp"
#include "circle/types/detection.hpp"
#include "circle/vision/detection_filter.hpp"
#include "circle/vision/sot_byte_track.hpp"

namespace circle::perception {

enum class InferSlotResult {
  Detection,
  NoDetection,
  RknnFailed,
};

/** Multi-slot RKNN pool: one engine per slot for pipeline-overlap / parallel infer. */
class ParallelVisionPipeline {
 public:
  explicit ParallelVisionPipeline(VisionPipelineConfig config);

  bool initialize(int slot_count);
  int slotCount() const { return static_cast<int>(engines_.size()); }

  RknnEngine& engine(int slot_id);
  const RknnEngine& engine(int slot_id) const;
  int slotDmaFd(int slot_id) const;

  const LetterboxParams& letterbox() const { return letterbox_; }
  uint32_t inputBufferSize() const;

  /** RGA already wrote letterboxed RGB into slot DMA; run NPU + postprocess. */
  InferSlotResult inferPreparedSlot(int slot_id,
                                    int src_w,
                                    int src_h,
                                    uint64_t seq,
                                    circle::types::TimestampNs capture_ns,
                                    circle::types::FrameDetection& out,
                                    VisionPipelineTiming* timing);

 private:
  circle::types::Detection yoloToDetection(const circle::vision::YoloDetection& y);
  bool postprocessEngine(RknnEngine& engine,
                         int src_w,
                         int src_h,
                         VisionPipelineTiming* timing,
                         circle::types::FrameDetection& out,
                         std::chrono::steady_clock::time_point total_start);
  /** Snapshot the gating anchor (mutex-guarded; slots run concurrently). */
  circle::vision::DetectionTrackHint trackHintSnapshot() const;
  void updateTrackHint(const circle::types::Detection* best);

  VisionPipelineConfig config_;
  LetterboxParams letterbox_{};
  std::vector<std::unique_ptr<RknnEngine>> engines_;
  bool initialized_{false};
  mutable std::mutex track_mu_;
  circle::vision::DetectionTrackHint track_hint_{};
  int track_hint_misses_{0};
  circle::vision::SotByteTrack byte_track_;
};

}  // namespace circle::perception
