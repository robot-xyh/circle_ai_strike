#include "circle/debug/pipeline_perf.hpp"

#include "circle/debug/preview_overlay.hpp"

#include <algorithm>
#include <cmath>

namespace circle::debug {
namespace {

constexpr auto kWindow = std::chrono::seconds(1);
constexpr auto kRecordTtl = std::chrono::seconds(2);
constexpr size_t kMaxRecords = 256;
constexpr size_t kMaxLatencySamples = 128;

bool validTp(PipelinePerfMeter::SteadyTimePoint tp) {
  return tp.time_since_epoch().count() > 0;
}

}  // namespace

void PipelinePerfMeter::SlidingCounter::tick(SteadyTimePoint now) {
  if (window_start.time_since_epoch().count() == 0) {
    window_start = now;
  }
  if (now - window_start >= kWindow) {
    window_start = now;
    events = 0;
  }
  ++events;
}

float PipelinePerfMeter::SlidingCounter::ratePerSec(SteadyTimePoint now) const {
  if (window_start.time_since_epoch().count() == 0) {
    return 0.0F;
  }
  const auto elapsed = now - window_start;
  if (elapsed <= std::chrono::milliseconds(0)) {
    return static_cast<float>(events);
  }
  const double sec =
      std::chrono::duration<double>(elapsed).count();
  if (sec <= 1.0e-6) {
    return static_cast<float>(events);
  }
  return static_cast<float>(static_cast<double>(events) / sec);
}

float PipelinePerfMeter::msBetween(SteadyTimePoint a, SteadyTimePoint b) {
  if (!validTp(a) || !validTp(b)) {
    return 0.0F;
  }
  return std::chrono::duration<float, std::milli>(b - a).count();
}

float PipelinePerfMeter::percentile50(std::vector<float> values) {
  if (values.empty()) {
    return 0.0F;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid),
                   values.end());
  return values[mid];
}

PipelinePerfMeter::FrameRecord& PipelinePerfMeter::getOrCreate(uint64_t seq) {
  for (auto& rec : frames_) {
    if (rec.seq == seq) {
      return rec;
    }
  }
  FrameRecord rec;
  rec.seq = seq;
  frames_.push_back(rec);
  return frames_.back();
}

const PipelinePerfMeter::FrameRecord* PipelinePerfMeter::find(uint64_t seq) const {
  for (const auto& rec : frames_) {
    if (rec.seq == seq) {
      return &rec;
    }
  }
  return nullptr;
}

PipelinePerfMeter::FrameRecord* PipelinePerfMeter::findMutable(uint64_t seq) {
  for (auto& rec : frames_) {
    if (rec.seq == seq) {
      return &rec;
    }
  }
  return nullptr;
}

void PipelinePerfMeter::pruneOld(SteadyTimePoint now) {
  while (frames_.size() > kMaxRecords) {
    input_ready_seqs_.erase(frames_.front().seq);
    wired_seqs_.erase(frames_.front().seq);
    algo_seqs_.erase(frames_.front().seq);
    frames_.pop_front();
  }
  while (!frames_.empty()) {
    const auto& front = frames_.front();
    const SteadyTimePoint newest = front.msp_sent.time_since_epoch().count() > 0
                                       ? front.msp_sent
                                   : front.control_done.time_since_epoch().count() > 0
                                       ? front.control_done
                                   : front.infer_done.time_since_epoch().count() > 0
                                       ? front.infer_done
                                       : front.producer_done;
    if (!validTp(newest) || now - newest <= kRecordTtl) {
      break;
    }
    input_ready_seqs_.erase(front.seq);
    wired_seqs_.erase(front.seq);
    algo_seqs_.erase(front.seq);
    frames_.pop_front();
  }
}

void PipelinePerfMeter::noteProducerStages(uint64_t seq,
                                           SteadyTimePoint grab_start,
                                           SteadyTimePoint grab_done,
                                           SteadyTimePoint producer_done) {
  std::lock_guard<std::mutex> lk(mu_);
  auto& rec = getOrCreate(seq);
  rec.grab_start = grab_start;
  rec.grab_done = grab_done;
  rec.producer_done = producer_done;
  pruneOld(producer_done);
}

void PipelinePerfMeter::noteInferStart(uint64_t seq, SteadyTimePoint infer_start) {
  std::lock_guard<std::mutex> lk(mu_);
  auto& rec = getOrCreate(seq);
  rec.infer_start = infer_start;
  pruneOld(infer_start);
}

void PipelinePerfMeter::noteInferDone(uint64_t seq, SteadyTimePoint infer_done) {
  std::lock_guard<std::mutex> lk(mu_);
  auto& rec = getOrCreate(seq);
  rec.infer_done = infer_done;
  pruneOld(infer_done);
}

bool PipelinePerfMeter::tryNoteControlInputReady(uint64_t seq,
                                                 SteadyTimePoint ready_time) {
  std::lock_guard<std::mutex> lk(mu_);
  if (input_ready_seqs_.count(seq) > 0) {
    return false;
  }
  FrameRecord* rec = findMutable(seq);
  if (rec == nullptr || !validTp(rec->grab_done) || !validTp(rec->infer_done)) {
    return false;
  }
  rec->control_input_ready = ready_time;
  rec->input_ready = true;
  input_ready_seqs_.insert(seq);

  const float input_ms = msBetween(rec->grab_done, ready_time);
  recent_e2e_input_ms_.push_back(input_ms);
  if (recent_e2e_input_ms_.size() > kMaxLatencySamples) {
    recent_e2e_input_ms_.pop_front();
  }
  e2e_input_.tick(ready_time);
  pruneOld(ready_time);
  return true;
}

bool PipelinePerfMeter::tryNoteControlComplete(uint64_t seq,
                                             SteadyTimePoint exec_start,
                                             SteadyTimePoint exec_end) {
  std::lock_guard<std::mutex> lk(mu_);
  if (algo_seqs_.count(seq) > 0) {
    return false;
  }
  FrameRecord* rec = findMutable(seq);
  if (rec == nullptr || !validTp(rec->grab_done) || !validTp(rec->infer_done)) {
    return false;
  }
  rec->control_tick = exec_start;
  rec->control_done = exec_end;
  rec->algo_done = true;
  algo_seqs_.insert(seq);

  const float algo_ms = msBetween(rec->grab_done, exec_end);
  recent_e2e_algo_ms_.push_back(algo_ms);
  if (recent_e2e_algo_ms_.size() > kMaxLatencySamples) {
    recent_e2e_algo_ms_.pop_front();
  }
  ctrl_.tick(exec_end);
  e2e_algo_.tick(exec_end);
  pruneOld(exec_end);
  return true;
}

bool PipelinePerfMeter::tryNoteWireComplete(uint64_t seq,
                                            SteadyTimePoint exec_start,
                                            SteadyTimePoint msp_sent) {
  std::lock_guard<std::mutex> lk(mu_);
  if (wired_seqs_.count(seq) > 0) {
    return false;
  }
  FrameRecord* rec = findMutable(seq);
  if (rec == nullptr || !validTp(rec->grab_done) || !validTp(rec->infer_done)) {
    return false;
  }
  rec->control_tick = exec_start;
  rec->control_done = msp_sent;
  rec->msp_sent = msp_sent;
  rec->wired = true;
  wired_seqs_.insert(seq);

  const float wire_ms = msBetween(rec->grab_done, msp_sent);
  recent_e2e_wire_ms_.push_back(wire_ms);
  if (recent_e2e_wire_ms_.size() > kMaxLatencySamples) {
    recent_e2e_wire_ms_.pop_front();
  }
  e2e_wire_.tick(msp_sent);
  pruneOld(msp_sent);
  return true;
}

void PipelinePerfMeter::noteProdEvent() {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(mu_);
  prod_.tick(now);
}

void PipelinePerfMeter::noteInfEvent() {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(mu_);
  inf_.tick(now);
}

void PipelinePerfMeter::noteMspEvent() {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(mu_);
  msp_.tick(now);
}

void PipelinePerfMeter::updateProdFpsFromTotal(uint64_t grab_ok_total,
                                               SteadyTimePoint now) {
  std::lock_guard<std::mutex> lk(mu_);
  if (last_grab_sample_at_.time_since_epoch().count() == 0) {
    last_grab_ok_total_ = grab_ok_total;
    last_grab_sample_at_ = now;
    return;
  }
  const auto elapsed = now - last_grab_sample_at_;
  if (elapsed < std::chrono::milliseconds(200)) {
    return;
  }
  if (grab_ok_total >= last_grab_ok_total_) {
    const uint64_t delta = grab_ok_total - last_grab_ok_total_;
    for (uint64_t i = 0; i < delta; ++i) {
      prod_.tick(now);
    }
  }
  last_grab_ok_total_ = grab_ok_total;
  last_grab_sample_at_ = now;
}

void PipelinePerfMeter::setPipeStats(const PipelinePipeStats& stats) {
  std::lock_guard<std::mutex> lk(mu_);
  pipe_stats_ = stats;
}

PipelinePerfSnapshot PipelinePerfMeter::snapshot(uint64_t latest_seq,
                                                 bool wire_path_active) const {
  const auto now = std::chrono::steady_clock::now();
  PipelinePerfSnapshot out;
  out.wire_path_active = wire_path_active;
  std::lock_guard<std::mutex> lk(mu_);
  out.pipe = pipe_stats_;
  out.prod_fps = prod_.ratePerSec(now);
  out.inf_fps = inf_.ratePerSec(now);
  out.ctrl_fps = ctrl_.ratePerSec(now);
  out.msp_fps = msp_.ratePerSec(now);
  out.e2e_fps = e2e_input_.ratePerSec(now);

  const FrameRecord* rec = find(latest_seq);
  if (rec != nullptr && validTp(rec->grab_start) && validTp(rec->grab_done)) {
    out.wait_grab_ms = msBetween(rec->grab_start, rec->grab_done);
  }
  if (rec != nullptr) {
    if (validTp(rec->producer_done) && validTp(rec->infer_start)) {
      out.queue_wait_ms = msBetween(rec->producer_done, rec->infer_start);
    }
    if (validTp(rec->grab_done) && validTp(rec->producer_done)) {
      out.producer_ms = msBetween(rec->grab_done, rec->producer_done);
    }
    if (validTp(rec->infer_start) && validTp(rec->infer_done)) {
      out.cnn_ms = msBetween(rec->infer_start, rec->infer_done);
    }
    if (rec->input_ready && validTp(rec->grab_done) &&
        validTp(rec->control_input_ready)) {
      out.e2e_input_ms = msBetween(rec->grab_done, rec->control_input_ready);
    }
    float control_wait = 0.0F;
    float exec = 0.0F;
    if (validTp(rec->infer_done) && validTp(rec->control_tick)) {
      control_wait = msBetween(rec->infer_done, rec->control_tick);
    }
    if (validTp(rec->control_tick) && validTp(rec->control_done)) {
      exec = msBetween(rec->control_tick, rec->control_done);
    }
    if (control_wait > 0.0F || exec > 0.0F) {
      out.ctrl_ms = control_wait + exec;
    }
    if (validTp(rec->control_done) && validTp(rec->msp_sent)) {
      out.msp_gate_ms = msBetween(rec->control_done, rec->msp_sent);
    }
    if (rec->wired && validTp(rec->grab_done) && validTp(rec->msp_sent)) {
      out.e2e_wire_ms = msBetween(rec->grab_done, rec->msp_sent);
    }
    if (rec->algo_done && validTp(rec->grab_done) &&
        validTp(rec->control_done)) {
      out.e2e_algo_ms = msBetween(rec->grab_done, rec->control_done);
    }
  }

  if (!recent_e2e_wire_ms_.empty()) {
    std::vector<float> vals(recent_e2e_wire_ms_.begin(), recent_e2e_wire_ms_.end());
    out.e2e_wire_p50_ms = percentile50(std::move(vals));
  }
  if (!recent_e2e_input_ms_.empty()) {
    std::vector<float> vals(recent_e2e_input_ms_.begin(), recent_e2e_input_ms_.end());
    out.e2e_input_p50_ms = percentile50(std::move(vals));
  }
  if (!recent_e2e_algo_ms_.empty()) {
    std::vector<float> vals(recent_e2e_algo_ms_.begin(), recent_e2e_algo_ms_.end());
    out.e2e_algo_p50_ms = percentile50(std::move(vals));
  }
  return out;
}

void applyPipelinePerfSnapshot(PreviewOverlayContext& overlay,
                               const PipelinePerfSnapshot& snap) {
  overlay.perf_wire_path_active = snap.wire_path_active;
  overlay.perf_e2e_fps = snap.e2e_fps;
  overlay.perf_prod_fps = snap.prod_fps;
  overlay.perf_inf_fps = snap.inf_fps;
  overlay.perf_ctrl_fps = snap.ctrl_fps;
  overlay.perf_msp_fps = snap.msp_fps;
  overlay.perf_wait_grab_ms = snap.wait_grab_ms;
  overlay.perf_e2e_input_ms = snap.e2e_input_ms;
  overlay.perf_e2e_input_p50_ms = snap.e2e_input_p50_ms;
  overlay.perf_e2e_wire_ms = snap.e2e_wire_ms;
  overlay.perf_e2e_wire_p50_ms = snap.e2e_wire_p50_ms;
  overlay.perf_e2e_algo_ms = snap.e2e_algo_ms;
  overlay.perf_e2e_algo_p50_ms = snap.e2e_algo_p50_ms;
  overlay.perf_queue_wait_ms = snap.queue_wait_ms;
  overlay.perf_producer_ms = snap.producer_ms;
  overlay.perf_cnn_ms = snap.cnn_ms;
  overlay.perf_ctrl_ms = snap.ctrl_ms;
  overlay.perf_msp_gate_ms = snap.msp_gate_ms;
  overlay.perf_pipe_slot_count = snap.pipe.slot_count;
  overlay.perf_pipe_slot_busy = snap.pipe.slot_busy;
  overlay.perf_pipe_ready_drop = snap.pipe.ready_drop;
  overlay.perf_pipe_zero_copy = snap.pipe.zero_copy;
}

void mergePipelinePerfOverlay(PreviewOverlayContext& dst,
                              const PreviewOverlayContext& src) {
  dst.perf_wire_path_active = src.perf_wire_path_active;
  dst.perf_pipe_zero_copy = src.perf_pipe_zero_copy;
  dst.perf_e2e_fps = src.perf_e2e_fps;
  dst.perf_prod_fps = src.perf_prod_fps;
  dst.perf_inf_fps = src.perf_inf_fps;
  dst.perf_ctrl_fps = src.perf_ctrl_fps;
  dst.perf_msp_fps = src.perf_msp_fps;
  dst.perf_wait_grab_ms = src.perf_wait_grab_ms;
  dst.perf_e2e_input_ms = src.perf_e2e_input_ms;
  dst.perf_e2e_input_p50_ms = src.perf_e2e_input_p50_ms;
  dst.perf_e2e_wire_ms = src.perf_e2e_wire_ms;
  dst.perf_e2e_wire_p50_ms = src.perf_e2e_wire_p50_ms;
  dst.perf_e2e_algo_ms = src.perf_e2e_algo_ms;
  dst.perf_e2e_algo_p50_ms = src.perf_e2e_algo_p50_ms;
  dst.perf_queue_wait_ms = src.perf_queue_wait_ms;
  dst.perf_producer_ms = src.perf_producer_ms;
  dst.perf_cnn_ms = src.perf_cnn_ms;
  dst.perf_ctrl_ms = src.perf_ctrl_ms;
  dst.perf_msp_gate_ms = src.perf_msp_gate_ms;
  dst.perf_pipe_slot_count = src.perf_pipe_slot_count;
  dst.perf_pipe_slot_busy = src.perf_pipe_slot_busy;
  dst.perf_pipe_ready_drop = src.perf_pipe_ready_drop;
}

}  // namespace circle::debug
