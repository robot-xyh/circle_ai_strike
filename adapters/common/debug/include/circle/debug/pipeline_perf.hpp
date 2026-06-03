#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <unordered_set>
#include <vector>

#include "circle/perception/pipeline_timing.hpp"

namespace circle::debug {

using FramePipelineTiming = circle::perception::FramePipelineTiming;

struct PipelinePipeStats {
  int slot_count{0};
  uint64_t slot_busy{0};
  uint64_t ready_drop{0};
  bool zero_copy{false};
};

struct PipelinePerfSnapshot {
  std::optional<float> e2e_fps;
  std::optional<float> prod_fps;
  std::optional<float> inf_fps;
  std::optional<float> ctrl_fps;
  std::optional<float> msp_fps;
  std::optional<float> wait_grab_ms;
  std::optional<float> e2e_input_ms;
  std::optional<float> e2e_input_p50_ms;
  std::optional<float> e2e_wire_ms;
  std::optional<float> e2e_wire_p50_ms;
  std::optional<float> e2e_algo_ms;
  std::optional<float> e2e_algo_p50_ms;
  std::optional<float> queue_wait_ms;
  std::optional<float> producer_ms;
  std::optional<float> cnn_ms;
  std::optional<float> ctrl_ms;
  std::optional<float> msp_gate_ms;
  bool wire_path_active{false};
  PipelinePipeStats pipe{};
};

class PipelinePerfMeter {
 public:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  void noteProducerStages(uint64_t seq, SteadyTimePoint grab_start,
                          SteadyTimePoint grab_done,
                          SteadyTimePoint producer_done);
  void noteInferStart(uint64_t seq, SteadyTimePoint infer_start);
  void noteInferDone(uint64_t seq, SteadyTimePoint infer_done);
  /** Vision result has been published to the control-side latest_detection. */
  bool tryNoteControlInputReady(uint64_t seq, SteadyTimePoint ready_time);
  /** grab_done + infer_done + control tick; counts e2e_algo once per seq. */
  bool tryNoteControlComplete(uint64_t seq, SteadyTimePoint exec_start,
                              SteadyTimePoint exec_end);
  /** grab_done + infer_done + MSP wire; counts e2e_wire once per seq. */
  bool tryNoteWireComplete(uint64_t seq, SteadyTimePoint exec_start,
                           SteadyTimePoint msp_sent);

  void noteProdEvent();
  void noteInfEvent();
  void noteMspEvent();

  void updateProdFpsFromTotal(uint64_t grab_ok_total, SteadyTimePoint now);
  void setPipeStats(const PipelinePipeStats& stats);

  PipelinePerfSnapshot snapshot(uint64_t latest_seq, bool wire_path_active) const;

 private:
  struct FrameRecord {
    uint64_t seq{0};
    SteadyTimePoint grab_start{};
    SteadyTimePoint grab_done{};
    SteadyTimePoint producer_done{};
    SteadyTimePoint infer_start{};
    SteadyTimePoint infer_done{};
    SteadyTimePoint control_input_ready{};
    SteadyTimePoint control_tick{};
    SteadyTimePoint control_done{};
    SteadyTimePoint msp_sent{};
    bool input_ready{false};
    bool wired{false};
    bool algo_done{false};
  };

  struct SlidingCounter {
    uint64_t events{0};
    SteadyTimePoint window_start{};

    void tick(SteadyTimePoint now);
    float ratePerSec(SteadyTimePoint now) const;
  };

  static float msBetween(SteadyTimePoint a, SteadyTimePoint b);
  static float percentile50(std::vector<float> values);

  FrameRecord& getOrCreate(uint64_t seq);
  FrameRecord* findMutable(uint64_t seq);
  const FrameRecord* find(uint64_t seq) const;
  void pruneOld(SteadyTimePoint now);

  mutable std::mutex mu_;
  std::deque<FrameRecord> frames_;
  std::unordered_set<uint64_t> input_ready_seqs_;
  std::unordered_set<uint64_t> wired_seqs_;
  std::unordered_set<uint64_t> algo_seqs_;
  SlidingCounter prod_{};
  SlidingCounter inf_{};
  SlidingCounter ctrl_{};
  SlidingCounter msp_{};
  SlidingCounter e2e_input_{};
  SlidingCounter e2e_wire_{};
  SlidingCounter e2e_algo_{};
  std::deque<float> recent_e2e_input_ms_;
  std::deque<float> recent_e2e_wire_ms_;
  std::deque<float> recent_e2e_algo_ms_;
  uint64_t last_grab_ok_total_{0};
  SteadyTimePoint last_grab_sample_at_{};
  PipelinePipeStats pipe_stats_{};
};

void applyPipelinePerfSnapshot(class PreviewOverlayContext& overlay,
                               const PipelinePerfSnapshot& snap);

/** Copy perf HUD fields from src into dst (e.g. control overlay into preview frame). */
void mergePipelinePerfOverlay(PreviewOverlayContext& dst,
                              const PreviewOverlayContext& src);

}  // namespace circle::debug
