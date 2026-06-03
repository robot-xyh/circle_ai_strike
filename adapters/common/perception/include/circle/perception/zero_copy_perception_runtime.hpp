#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "circle/perception/camera_source.hpp"
#include "circle/perception/mpp_rga_pipeline.hpp"
#include "circle/perception/parallel_vision_pipeline.hpp"
#include "circle/perception/zero_copy_frame_slot.hpp"
#include "circle/types/detection.hpp"

#include "circle/perception/pipeline_timing.hpp"

namespace circle::perception {

struct ZeroCopyPerceptionConfig {
  MppCameraSourceConfig camera;
  VisionPipelineConfig vision;
  int slot_count{3};
  /** Infer worker threads (1 = pipeline overlap; >1 = parallel slots). */
  int infer_worker_count{1};
  uint32_t max_infer_fps{0};
};

struct ZeroCopyRuntimeStats {
  uint64_t grab_ok{0};
  /** RKNN ok + detection passed filter (legacy infer_ok). */
  uint64_t infer_det{0};
  /** RKNN ok but no detection passed filter. */
  uint64_t infer_no_det{0};
  /** runZeroCopy / RKNN runtime failure. */
  uint64_t infer_rknn_fail{0};
  uint64_t slot_busy_drops{0};
  uint64_t ready_drops{0};
};

struct ZeroCopyInferResult {
  circle::types::FrameDetection detection{};
  VisionPipelineTiming timing{};
  std::vector<uint8_t> preview_bgr;
  uint32_t preview_w{0};
  uint32_t preview_h{0};
  uint64_t frame_seq{0};
  int64_t stamp_ns{0};
  float capture_time_ms{0.0F};
  float decode_time_ms{0.0F};
  FramePipelineTiming pipeline{};
};

/**
 * In-process multi-slot camera → RGA → RKNN pipeline (same overlap model as
 * mpp_cam + multi_source_yolo zero-copy, without Unix socket IPC).
 */
class ZeroCopyPerceptionRuntime {
 public:
  using ResultCallback = std::function<void(ZeroCopyInferResult&&)>;

  explicit ZeroCopyPerceptionRuntime(ZeroCopyPerceptionConfig config);
  ~ZeroCopyPerceptionRuntime();

  ZeroCopyPerceptionRuntime(const ZeroCopyPerceptionRuntime&) = delete;
  ZeroCopyPerceptionRuntime& operator=(const ZeroCopyPerceptionRuntime&) = delete;

  bool initialize();
  bool start(ResultCallback on_result, std::atomic<bool>* running);
  void stop();

  int slotCount() const { return config_.slot_count; }
  ZeroCopyRuntimeStats stats() const;

 private:
  struct ReadyFrame {
    uint32_t slot_id{0};
    uint64_t seq{0};
    int64_t capture_ns{0};
    float capture_time_ms{0.0F};
    float decode_time_ms{0.0F};
    std::chrono::steady_clock::time_point grab_start{};
    std::chrono::steady_clock::time_point grab_done{};
    std::chrono::steady_clock::time_point producer_done{};
    int src_w{0};
    int src_h{0};
  };

  bool openV4L2();
  void closeV4L2();
  bool captureMjpeg(void** buffer, size_t* size, int* buf_index);
  void requeueV4L2Buffer(int index);
  void copyPreviewBgrFromSlot(uint32_t slot_id, std::vector<uint8_t>* bgr,
                              uint32_t width, uint32_t height);

  void producerLoop();
  void inferWorkerLoop();

  ZeroCopyPerceptionConfig config_;
  ParallelVisionPipeline vision_;
  MppRgaPipeline pipeline_;
  ZeroCopySlotGeometry geometry_{};
  std::vector<ZeroCopyFrameSlot> slots_;

  ResultCallback on_result_;
  std::atomic<bool>* running_{nullptr};

  std::thread producer_thread_;
  std::vector<std::thread> infer_threads_;

  std::mutex ready_mu_;
  std::condition_variable ready_cv_;
  std::deque<ReadyFrame> ready_queue_;
  static constexpr size_t kMaxReadyQueue = 16;

  std::mutex pipeline_mu_;

  int v4l2_fd_{-1};
  std::vector<void*> v4l2_buffers_;
  std::vector<size_t> v4l2_buffer_sizes_;
  int v4l2_buffer_count_{0};
  bool v4l2_streaming_{false};
  int32_t active_buffer_index_{-1};
  uint64_t seq_{0};

  uint64_t grab_ok_{0};
  uint64_t grab_fail_{0};
  uint64_t infer_det_{0};
  uint64_t infer_no_det_{0};
  uint64_t infer_rknn_fail_{0};
  uint64_t slot_busy_drops_{0};
  uint64_t ready_drops_{0};
};

}  // namespace circle::perception
