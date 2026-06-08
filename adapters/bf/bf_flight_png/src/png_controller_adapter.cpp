#include "png_controller_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "circle/types/time.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include "circle/debug_common/strike_png_param_tune.hpp"
#endif

namespace circle::bf::png {

namespace {

uint64_t targetLostHoldDelayNs(double delay_s) {
  return static_cast<uint64_t>(std::max(0.0, delay_s) * 1.0e9);
}

constexpr float kNsToS = 1.0e-9F;
constexpr float kRadWrap = static_cast<float>(2.0 * M_PI);
constexpr uint64_t kBodyRateHistoryMaxAgeNs = 2'000'000'000ULL;

}  // namespace

PngControllerAdapter::PngControllerAdapter(circle::strike_png::StrikePngNodeParams params)
    : params_(std::move(params)) {}

void PngControllerAdapter::onEngageRisingEdge(
    const circle::bf::runtime::BfControlContext& ctx) {
  controller_.reset();
  loss_state_.reset();
  entry_snapshot_ = circle::strike_png::EntryHandoffSnapshot{};
  entry_snapshot_.activation_ns =
      ctx.now_ns != 0U ? ctx.now_ns : static_cast<uint64_t>(1);
  entry_snapshot_.thrust_z = params_.entry_handoff.initial_thrust_z;
  entry_snapshot_.roll_rate_rad_s = ctx.vehicle.roll_rate_rad_s;
  entry_snapshot_.pitch_rate_rad_s = ctx.vehicle.pitch_rate_rad_s;
}

circle::bf::runtime::BfControlResult PngControllerAdapter::update(
    const circle::bf::runtime::BfControlContext& ctx) {
  ingestBodyRateSample(ctx.vehicle);

  const auto& intr = ctx.intrinsics;
  const auto& det = ctx.detection.detection;

  const bool camera_valid = intr.fx > 1.0e-6F && intr.fy > 1.0e-6F &&
                            ctx.image_width > 0U && ctx.image_height > 0U;
  const double age_s =
      circle::types::secondsBetween(ctx.detection.capture_ns, ctx.now_ns);
  const bool detection_fresh = ctx.detection.valid && age_s >= 0.0 &&
                               age_s <= params_.detection_stale_s;

  circle::strike_png::StrikePngInput input;
  input.now_ns = ctx.now_ns != 0U ? ctx.now_ns : static_cast<uint64_t>(1);
  input.detection_valid = camera_valid && detection_fresh;
  if (input.detection_valid) {
    input.measurement_ns =
        ctx.detection.capture_ns != 0U ? ctx.detection.capture_ns : input.now_ns;
    input.ex = (det.cx - intr.cx) / intr.fx;
    input.ey = (det.cy - intr.cy) / intr.fy;
    input.fx = intr.fx;
    input.fy = intr.fy;
    const float frame_area = static_cast<float>(ctx.image_width) *
                             static_cast<float>(ctx.image_height);
    const float bbox_area = det.width * det.height;
    input.bbox_area_px = std::max(0.0F, bbox_area);
    input.detection_score = det.score;
    input.bbox_area_ratio =
        frame_area > 1.0F ? std::max(0.0F, bbox_area / frame_area) : 0.0F;
  }
  input.derotate_rate_valid = false;
  last_derotate_lookup_valid_ = false;
  last_derotate_lookup_age_ms_ = 0.0F;
  last_derotate_interp_gap_ms_ = 0.0F;
  last_derotate_roll_rate_rad_s_ = 0.0F;
  last_derotate_pitch_rate_rad_s_ = 0.0F;
  if (params_.derotate_history_enable && input.detection_valid) {
    const uint64_t lookup_ns = subtractNs(
        input.measurement_ns, params_.camera_exposure_midpoint_offset_ns);
    if (input.now_ns >= lookup_ns) {
      last_derotate_lookup_age_ms_ =
          static_cast<float>(input.now_ns - lookup_ns) * 1.0e-6F;
    }
    BodyRateLookup lookup;
    if (lookupBodyRate(lookup_ns, lookup)) {
      input.roll_rate_rad_s = lookup.sample.roll_rate_rad_s;
      input.pitch_rate_rad_s = lookup.sample.pitch_rate_rad_s;
      input.derotate_rate_valid = true;
      last_derotate_lookup_valid_ = true;
      last_derotate_interp_gap_ms_ = lookup.interp_gap_ms;
      last_derotate_roll_rate_rad_s_ = lookup.sample.roll_rate_rad_s;
      last_derotate_pitch_rate_rad_s_ = lookup.sample.pitch_rate_rad_s;
    } else {
      last_derotate_interp_gap_ms_ = lookup.interp_gap_ms;
    }
  } else if (!params_.derotate_history_enable) {
    input.roll_rate_rad_s = ctx.vehicle.roll_rate_rad_s;
    input.pitch_rate_rad_s = ctx.vehicle.pitch_rate_rad_s;
    input.derotate_rate_valid = ctx.vehicle.valid;
    last_derotate_roll_rate_rad_s_ = input.roll_rate_rad_s;
    last_derotate_pitch_rate_rad_s_ = input.pitch_rate_rad_s;
  }
  input.attitude_valid = ctx.vehicle.valid;
  input.vehicle_roll_rad = ctx.vehicle.roll_rad;
  input.vehicle_pitch_rad = ctx.vehicle.pitch_rad;
  if (ctx.vehicle.velocity_xy_valid) {
    input.ownship_forward_speed_valid = true;
    input.ownship_forward_speed_m_s = ctx.vehicle.velocity_ned_y;
  }

  last_output_ = controller_.tick(params_.controller, input);
  last_input_ = input;
  last_detection_score_ = input.detection_valid ? det.score : 0.0F;

  float roll_rate = last_output_.roll_rate_rad_s;
  float pitch_rate = last_output_.pitch_rate_rad_s;
  float thrust_z =
      last_output_.has_target ? params_.strike_thrust_z : params_.hover_thrust_z;
  const auto handoff = circle::strike_png::applyEntryHandoff(
      params_.entry_handoff, entry_snapshot_,
      circle::strike_png::EntryHandoffCommand{roll_rate, pitch_rate, thrust_z},
      input.now_ns);
  if (params_.entry_handoff.enable && entry_snapshot_.activation_ns != 0U &&
      params_.entry_handoff.duration_s > 0.0F &&
      input.now_ns > entry_snapshot_.activation_ns) {
    const float elapsed_s =
        static_cast<float>(input.now_ns - entry_snapshot_.activation_ns) * 1.0e-9F;
    last_handoff_progress_ = circle::strike_png::entryHandoffSmoothstep(
        elapsed_s / std::max(1.0e-3F, params_.entry_handoff.duration_s));
  } else {
    last_handoff_progress_ = handoff.active ? 0.0F : 1.0F;
  }

  circle::bf::runtime::BfControlResult result;
  result.rates.roll_rate_rad_s = handoff.roll_rate_rad_s;
  result.rates.pitch_rate_rad_s = handoff.pitch_rate_rad_s;
  result.rates.yaw_rate_rad_s = 0.0F;
  result.rates.thrust_z = handoff.thrust_z;
  result.safety.dry_run = params_.dry_run;
  result.safety.require_armed_to_command = params_.require_armed_to_command;
  result.safety.armed = ctx.vehicle.armed;
  result.has_target = last_output_.has_target;
  result.image_ex = input.ex;
  result.image_ey = input.ey;
  result.state_code = last_output_.has_target ? 1 : 0;

  circle::strike_png::updateTargetLossHold(
      loss_state_, ctx.mode_active, last_output_.has_target, input.now_ns,
      targetLostHoldDelayNs(params_.target_lost_hold_delay_s),
      params_.target_lost_hold_enable);
  last_loss_hold_latched_ = loss_state_.hold_requested;

  // Keep commanding (PNG continues to publish in PX4) until the loss-hold
  // latches a release request; then drop to physical hold (the BF equivalent
  // of the PX4 LOITER handover).
  result.command = (ctx.mode_active && !loss_state_.hold_requested)
                       ? circle::bf::runtime::BfControlResult::Command::Algorithm
                       : circle::bf::runtime::BfControlResult::Command::None;
  return result;
}

void PngControllerAdapter::resetBodyRateHistory() {
  body_rate_history_ = {};
  body_rate_start_ = 0;
  body_rate_count_ = 0;
  prev_vehicle_stamp_ns_ = 0;
  prev_vehicle_roll_rad_ = 0.0F;
  prev_vehicle_pitch_rad_ = 0.0F;
  prev_vehicle_yaw_rad_ = 0.0F;
  prev_vehicle_valid_ = false;
  last_derotate_lookup_valid_ = false;
  last_derotate_lookup_age_ms_ = 0.0F;
  last_derotate_interp_gap_ms_ = 0.0F;
  last_derotate_roll_rate_rad_s_ = 0.0F;
  last_derotate_pitch_rate_rad_s_ = 0.0F;
}

uint64_t PngControllerAdapter::subtractNs(uint64_t stamp_ns, int64_t offset_ns) {
  if (offset_ns >= 0) {
    const uint64_t off = static_cast<uint64_t>(offset_ns);
    return stamp_ns > off ? stamp_ns - off : 0U;
  }
  return stamp_ns + static_cast<uint64_t>(-offset_ns);
}

float PngControllerAdapter::angleDeltaRad(float newer_rad, float older_rad) {
  float d = newer_rad - older_rad;
  while (d > static_cast<float>(M_PI)) {
    d -= kRadWrap;
  }
  while (d < -static_cast<float>(M_PI)) {
    d += kRadWrap;
  }
  return d;
}

void PngControllerAdapter::ingestBodyRateSample(
    const circle::types::FcState& vehicle) {
  if (!vehicle.valid || vehicle.stamp_ns == 0U) {
    prev_vehicle_valid_ = false;
    return;
  }

  const uint64_t compensated_stamp_ns =
      subtractNs(vehicle.stamp_ns, params_.fc_serial_latency_ns);
  if (prev_vehicle_valid_ && compensated_stamp_ns > prev_vehicle_stamp_ns_) {
    const float dt_s =
        static_cast<float>(compensated_stamp_ns - prev_vehicle_stamp_ns_) *
        kNsToS;
    if (dt_s > 1.0e-4F && dt_s < 1.0F) {
      BodyRateSample sample;
      sample.stamp_ns = compensated_stamp_ns;
      sample.roll_rate_rad_s =
          angleDeltaRad(vehicle.roll_rad, prev_vehicle_roll_rad_) / dt_s;
      sample.pitch_rate_rad_s =
          angleDeltaRad(vehicle.pitch_rad, prev_vehicle_pitch_rad_) / dt_s;
      sample.yaw_rate_rad_s =
          angleDeltaRad(vehicle.yaw_rad, prev_vehicle_yaw_rad_) / dt_s;
      sample.valid = std::isfinite(sample.roll_rate_rad_s) &&
                     std::isfinite(sample.pitch_rate_rad_s) &&
                     std::isfinite(sample.yaw_rate_rad_s);
      if (sample.valid) {
        const size_t insert =
            (body_rate_start_ + body_rate_count_) % body_rate_history_.size();
        body_rate_history_[insert] = sample;
        if (body_rate_count_ < body_rate_history_.size()) {
          ++body_rate_count_;
        } else {
          body_rate_start_ =
              (body_rate_start_ + 1U) % body_rate_history_.size();
        }
        while (body_rate_count_ > 0U) {
          const auto& oldest = body_rate_history_[body_rate_start_];
          if (!oldest.valid ||
              sample.stamp_ns - oldest.stamp_ns <= kBodyRateHistoryMaxAgeNs) {
            break;
          }
          body_rate_start_ =
              (body_rate_start_ + 1U) % body_rate_history_.size();
          --body_rate_count_;
        }
      }
    }
  }

  prev_vehicle_stamp_ns_ = compensated_stamp_ns;
  prev_vehicle_roll_rad_ = vehicle.roll_rad;
  prev_vehicle_pitch_rad_ = vehicle.pitch_rad;
  prev_vehicle_yaw_rad_ = vehicle.yaw_rad;
  prev_vehicle_valid_ = true;
}

bool PngControllerAdapter::lookupBodyRate(uint64_t lookup_ns,
                                          BodyRateLookup& out) const {
  if (lookup_ns == 0U || body_rate_count_ < 2U) {
    return false;
  }

  const auto sample_at = [&](size_t logical_index) -> const BodyRateSample& {
    return body_rate_history_[(body_rate_start_ + logical_index) %
                              body_rate_history_.size()];
  };

  const BodyRateSample& first = sample_at(0);
  const BodyRateSample& last = sample_at(body_rate_count_ - 1U);
  if (!first.valid || !last.valid || lookup_ns < first.stamp_ns ||
      lookup_ns > last.stamp_ns) {
    return false;
  }

  for (size_t i = 0; i + 1U < body_rate_count_; ++i) {
    const BodyRateSample& a = sample_at(i);
    const BodyRateSample& b = sample_at(i + 1U);
    if (!a.valid || !b.valid || lookup_ns < a.stamp_ns ||
        lookup_ns > b.stamp_ns) {
      continue;
    }
    const uint64_t gap_ns = b.stamp_ns - a.stamp_ns;
    out.interp_gap_ms = static_cast<float>(gap_ns) * 1.0e-6F;
    const float max_gap_s =
        std::max(0.001F, params_.max_derotate_interpolation_gap_s);
    if (static_cast<float>(gap_ns) * kNsToS > max_gap_s) {
      return false;
    }
    if (gap_ns == 0U) {
      out.sample = b;
      return true;
    }
    const float u =
        static_cast<float>(lookup_ns - a.stamp_ns) /
        static_cast<float>(gap_ns);
    out.sample.stamp_ns = lookup_ns;
    out.sample.roll_rate_rad_s =
        a.roll_rate_rad_s + u * (b.roll_rate_rad_s - a.roll_rate_rad_s);
    out.sample.pitch_rate_rad_s =
        a.pitch_rate_rad_s + u * (b.pitch_rate_rad_s - a.pitch_rate_rad_s);
    out.sample.yaw_rate_rad_s =
        a.yaw_rate_rad_s + u * (b.yaw_rate_rad_s - a.yaw_rate_rad_s);
    out.sample.valid = true;
    return true;
  }
  return false;
}

bool PngControllerAdapter::applyParamUpdateJson(const std::string& json) {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  const bool ok = circle::debug_common::applyStrikePngParamUpdate(params_, json);
  if (ok) {
    resetBodyRateHistory();
  }
  return ok;
#else
  (void)json;
  return false;
#endif
}

std::string PngControllerAdapter::paramsSnapshotJson() const {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  return circle::debug_common::strikePngParamsJson(params_);
#else
  return R"({"ok":false,"error":"yaml support not built"})";
#endif
}

void PngControllerAdapter::fillTelemetry(
    circle::ipc::StrikeTelemetrySample& sample,
    const circle::bf::runtime::BfControlResult& /*result*/) const {
  const auto& o = last_output_;
  sample.ex_dot_filt = o.ex_dot_filt;
  sample.ey_dot_filt = o.ey_dot_filt;
  sample.bbox_area_ratio = last_input_.bbox_area_ratio;
  sample.detection_score = last_detection_score_;
  sample.final_approach_active =
      (o.terminal_intercept_active || o.terminal_crossing_active) ? 1 : 0;

  // PNG-specific debug signals (controller_kind set to png by the host).
  sample.png_closure_scale = o.png_closure_scale;
  sample.png_ex_dot_inertial = o.png_ex_dot_inertial;
  sample.png_ey_dot_inertial = o.png_ey_dot_inertial;
  sample.png_measurement_age_s = o.measurement_age_s;
  sample.png_ff_roll_rad_s = o.roll_png_ff_rad_s;
  sample.png_ff_pitch_rad_s = o.pitch_png_ff_rad_s;
  sample.png_fov_trim_roll_rad_s = o.roll_fov_trim_rad_s;
  sample.png_fov_trim_pitch_rad_s = o.pitch_fov_trim_rad_s;
  sample.png_edge_guard_roll_rad_s = o.roll_edge_guard_rad_s;
  sample.png_edge_guard_pitch_rad_s = o.pitch_edge_guard_rad_s;
  sample.png_pursuit_roll_rad_s = o.roll_pursuit_fallback_rad_s;
  sample.png_pursuit_pitch_rad_s = o.pitch_pursuit_fallback_rad_s;
  sample.png_stale_trim_roll_rad_s = o.roll_terminal_stale_trim_rad_s;
  sample.png_intercept_roll_rad_s = o.roll_terminal_intercept_rad_s;
  sample.png_intercept_pitch_rad_s = o.pitch_terminal_intercept_rad_s;
  sample.png_crossing_pitch_rad_s = o.pitch_terminal_crossing_rad_s;
  sample.png_future_ex = o.terminal_future_ex;
  sample.png_future_ey = o.terminal_future_ey;
  sample.png_intercept_lead_s = o.terminal_intercept_lead_s;
  sample.png_crossing_weight = o.terminal_crossing_weight;
  sample.png_fwd_guard_scale = o.terminal_forward_speed_guard_scale;
  sample.png_entry_handoff_progress = last_handoff_progress_;
  sample.png_tilt_softcap_roll = o.roll_tilt_softcap_factor;
  sample.png_tilt_softcap_pitch = o.pitch_tilt_softcap_factor;
  sample.png_derotate_lookup_valid = last_derotate_lookup_valid_ ? 1 : 0;
  sample.png_derotate_lookup_age_ms = last_derotate_lookup_age_ms_;
  sample.png_derotate_interp_gap_ms = last_derotate_interp_gap_ms_;
  sample.png_derotate_roll_rate_rad_s = last_derotate_roll_rate_rad_s_;
  sample.png_derotate_pitch_rate_rad_s = last_derotate_pitch_rate_rad_s_;
  sample.png_camera_exposure_midpoint_offset_ns =
      static_cast<float>(params_.camera_exposure_midpoint_offset_ns);
  sample.png_fc_serial_latency_ns =
      static_cast<float>(params_.fc_serial_latency_ns);
  sample.png_intercept_active = o.terminal_intercept_active ? 1 : 0;
  sample.png_crossing_active = o.terminal_crossing_active ? 1 : 0;
  sample.png_fwd_guard_active = o.terminal_forward_speed_guard_active ? 1 : 0;
  sample.png_loss_hold_latched = last_loss_hold_latched_ ? 1 : 0;
  sample.png_tilt_hardcap_active = o.tilt_hardcap_active ? 1 : 0;
}

void PngControllerAdapter::fillOverlay(
    circle::debug::PreviewOverlayContext& /*overlay*/,
    const circle::bf::runtime::BfControlResult& /*result*/) const {}

const char* PngControllerAdapter::stateName(
    const circle::bf::runtime::BfControlResult& result) const {
  return result.has_target ? "PngTracking" : "PngWaiting";
}

}  // namespace circle::bf::png
