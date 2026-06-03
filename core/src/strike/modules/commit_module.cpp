#include "circle/strike/modules/commit_module.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

void CommitModule::reset() {
}

CommitOutput CommitModule::computeHold(
    CommitState& state,
    const FACommitParams& params,
    const ThrustParams& thrust_params,
    float hover_thrust_scalar,
    float lateral_output_sign,
    float longitudinal_output_sign,
    float vehicle_roll_rad,
    float vehicle_pitch_rad,
    float max_roll_angle_rad,
    float max_pitch_angle_rad,
    float tilt_softcap_band_rad,
    bool yaw_lock_enabled,
    double detection_age_s,
    circle::types::TimestampNs now_ns) const {
  CommitOutput out;

  if (!params.enable || !state.snapshot.valid) {
    return out;
  }

  const double command_age_s =
      circle::types::secondsBetween(state.snapshot.command_stamp_ns, now_ns);
  const double snapshot_detection_age_s =
      circle::types::secondsBetween(state.snapshot.detection_stamp_ns, now_ns);
  const double effective_detection_age_s =
      std::isfinite(detection_age_s) ? detection_age_s : snapshot_detection_age_s;

  // Compute latch age for min_latch and recent_centered_hold_through guards
  const double latch_age_s =
      state.latch_start_time_ns.has_value()
          ? circle::types::secondsBetween(*state.latch_start_time_ns, now_ns)
          : std::numeric_limits<double>::infinity();
  const bool min_latch_active =
      params.min_latch_s > 0.0F &&
      latch_age_s < static_cast<double>(params.min_latch_s);
  const bool recent_centered_hold_through_active =
      state.snapshot.recent_centered_terminal &&
      params.recent_centered_hold_through_s > 0.0F &&
      latch_age_s < static_cast<double>(params.recent_centered_hold_through_s);

  const bool blind_terminal_hold =
      state.snapshot.blind_terminal && command_age_s <= params.command_hold_s;

  const bool detection_expired =
      !blind_terminal_hold &&
      (effective_detection_age_s > params.detection_stale_s);

  if (command_age_s < 0.0 || snapshot_detection_age_s < 0.0 ||
      command_age_s > params.command_hold_s ||
      detection_expired) {
    if (!min_latch_active && !recent_centered_hold_through_active) {
      state.active = false;
      state.terminal_ready = false;
      state.snapshot = FinalApproachCommitSnapshot{};
      state.recent_centered_snapshot = FinalApproachCommitSnapshot{};
      state.latch_start_time_ns.reset();
      state.align_since_ns.reset();
      out.expired = true;
      return out;
    }
  }

  if (blind_terminal_hold && std::isfinite(detection_age_s) &&
      detection_age_s <= static_cast<double>(params.detection_stale_s)) {
    state.snapshot.detection_stamp_ns = now_ns;
  }

  out.command_age_s = static_cast<float>(command_age_s);

  float yaw_rate_sp = state.snapshot.yaw_rate_sp_rad_s;
  if (params.yaw_lock_enabled || yaw_lock_enabled) {
    yaw_rate_sp = 0.0F;
  }

  if (!state.terminal_ready) {
    return out;
  }

  const float commit_target_s = std::max(
      hover_thrust_scalar, params.thrust_scalar);
  float eff_s = std::clamp(commit_target_s, 0.0F, 1.0F);

  if (params.thrust_ramp_s > 1.0e-3F) {
    const float ramp_blend = smoothstep01(
        static_cast<float>(command_age_s) / params.thrust_ramp_s);
    const float snapshot_s = std::clamp(
        state.snapshot.thrust_z, 0.0F, 1.0F);
    eff_s = snapshot_s * (1.0F - ramp_blend) + eff_s * ramp_blend;
  }

  // Output positive scalar; adapter negates for PX4 FRD body.z.
  out.thrust_z = clampThrustScalar(eff_s, thrust_params.scalar_min,
                                   thrust_params.scalar_max);
  out.thrust_scalar = eff_s;

  float commit_roll_rate_sp = state.snapshot.roll_rate_sp_rad_s;
  float commit_pitch_rate_sp = state.snapshot.pitch_rate_sp_rad_s;

  if (params.predictive_enable &&
      params.predict_max_rate_rad_s > 0.0F &&
      params.predict_kp_rate > 0.0F) {
    const float command_age_f = static_cast<float>(command_age_s);
    const float predict_t_s = std::clamp(
        command_age_f + params.predict_lead_s, 0.0F, params.command_hold_s);
    const float predict_ex =
        state.snapshot.ex + state.snapshot.ex_dot * predict_t_s;
    const float predict_ey =
        state.snapshot.ey + state.snapshot.ey_dot * predict_t_s;

    float predict_roll_rate_sp =
        lateral_output_sign *
        (params.predict_kp_rate * predict_ex +
         params.predict_kd_rate * state.snapshot.ex_dot);
    float predict_pitch_rate_sp =
        longitudinal_output_sign *
        (params.predict_kp_rate * predict_ey +
         params.predict_kd_rate * state.snapshot.ey_dot);

    predict_roll_rate_sp = std::clamp(
        predict_roll_rate_sp,
        -params.predict_max_rate_rad_s,
        params.predict_max_rate_rad_s);
    predict_pitch_rate_sp = std::clamp(
        predict_pitch_rate_sp,
        -params.predict_max_rate_rad_s,
        params.predict_max_rate_rad_s);

    const float predict_blend =
        params.predict_blend_s > 1.0e-3F
            ? smoothstep01(command_age_f / params.predict_blend_s)
            : 1.0F;

    commit_roll_rate_sp =
        commit_roll_rate_sp * (1.0F - predict_blend) +
        predict_roll_rate_sp * predict_blend;
    commit_pitch_rate_sp =
        commit_pitch_rate_sp * (1.0F - predict_blend) +
        predict_pitch_rate_sp * predict_blend;
  }

  if (params.roll_level_max_rad_s > 0.0F &&
      params.roll_level_kp > 0.0F) {
    const float level_roll_rate_sp = std::clamp(
        -params.roll_level_kp * vehicle_roll_rad,
        -params.roll_level_max_rad_s,
        params.roll_level_max_rad_s);
    commit_roll_rate_sp =
        commit_roll_rate_sp * (1.0F - params.roll_level_blend) +
        level_roll_rate_sp * params.roll_level_blend;
  }

  if (params.pitch_level_max_rad_s > 0.0F &&
      params.pitch_level_kp > 0.0F) {
    const float level_pitch_rate_sp = std::clamp(
        -params.pitch_level_kp * vehicle_pitch_rad,
        -params.pitch_level_max_rad_s,
        params.pitch_level_max_rad_s);
    commit_pitch_rate_sp =
        commit_pitch_rate_sp * (1.0F - params.pitch_level_blend) +
        level_pitch_rate_sp * params.pitch_level_blend;
  }

  if (params.forward_pitch_rate_rad_s > 0.0F) {
    const float forward_pitch_rate = -params.forward_pitch_rate_rad_s;
    commit_pitch_rate_sp = std::min(commit_pitch_rate_sp, forward_pitch_rate);
  }

  const float commit_roll_softcap =
      tiltSoftcapFactor(vehicle_roll_rad, commit_roll_rate_sp,
                        max_roll_angle_rad, tilt_softcap_band_rad);
  const float commit_pitch_softcap =
      tiltSoftcapFactor(vehicle_pitch_rad, commit_pitch_rate_sp,
                        max_pitch_angle_rad, tilt_softcap_band_rad);
  commit_roll_rate_sp *= commit_roll_softcap;
  commit_pitch_rate_sp *= commit_pitch_softcap;

  out.roll_rate_rad_s = commit_roll_rate_sp;
  out.pitch_rate_rad_s = commit_pitch_rate_sp;
  out.yaw_rate_rad_s = yaw_rate_sp;
  out.should_hold = true;

  state.active = true;
  if (!state.latch_start_time_ns.has_value()) {
    state.latch_start_time_ns = now_ns;
  }

  return out;
}

bool CommitModule::shouldCreateSnapshot(
    const CommitState& state,
    const FACommitParams& params,
    bool final_approach_active,
    bool measure_reliable,
    bool,
    float bbox_area_ratio,
    float bbox_margin_x_px,
    float bbox_margin_y_px,
    float align_error_x_px,
    float align_error_y_px,
    float align_rate_x_px_s,
    float align_rate_y_px_s,
    float future_error_x_px,
    float future_error_y_px,
    float vehicle_roll_rad,
    float vehicle_pitch_rad,
    bool edge_protect_active,
    circle::types::TimestampNs now_ns) const {
  if (!params.enable || !final_approach_active || !measure_reliable) {
    return false;
  }

  if (params.freeze_on_edge_protect && edge_protect_active) {
    return false;
  }

  const bool margin_ok =
      bbox_margin_x_px >= params.min_margin_x_px &&
      bbox_margin_y_px >= params.min_margin_y_px;
  if (!margin_ok) {
    return false;
  }

  const bool min_area_ok =
      params.min_area_ratio <= 0.0F ||
      bbox_area_ratio >= params.min_area_ratio;
  if (!min_area_ok) {
    return false;
  }

  const bool area_guard =
      params.snapshot_max_area_ratio > 0.0F &&
      bbox_area_ratio > params.snapshot_max_area_ratio;
  if (area_guard) {
    return false;
  }

  const bool align_in_window =
      std::abs(align_error_x_px) <= params.align_max_error_x_px &&
      std::abs(align_error_y_px) <= params.align_max_error_y_px &&
      (params.align_max_rate_x_px_s <= 0.0F ||
       align_rate_x_px_s <= params.align_max_rate_x_px_s) &&
      (params.align_max_rate_y_px_s <= 0.0F ||
       align_rate_y_px_s <= params.align_max_rate_y_px_s);

  if (params.align_gate_enable && align_in_window) {
    if (!state.align_since_ns.has_value()) {
      const_cast<CommitState&>(state).align_since_ns = now_ns;
    }
    const double align_hold_s =
        circle::types::secondsBetween(*state.align_since_ns, now_ns);
    if (align_hold_s < params.align_hold_s) {
      return false;
    }
  } else if (params.align_gate_enable) {
    const_cast<CommitState&>(state).align_since_ns.reset();
    return false;
  }

  if (params.future_gate_enable) {
    const bool future_x_ok =
        params.future_max_error_x_px <= 0.0F ||
        std::abs(future_error_x_px) <= params.future_max_error_x_px;
    const bool future_y_ok =
        params.future_max_error_y_px <= 0.0F ||
        std::abs(future_error_y_px) <= params.future_max_error_y_px;
    if (!future_x_ok || !future_y_ok) {
      return false;
    }
  }

  const float tilt_rad = std::hypot(vehicle_roll_rad, vehicle_pitch_rad);
  if (params.tilt_gate_enable && tilt_rad > params.max_tilt_rad) {
    return false;
  }

  return true;
}

bool CommitModule::shouldCreateBlindSnapshot(
    const CommitState& /*state*/,
    const FACommitParams& params,
    bool,
    bool blind_commit_trigger,
    float bbox_area_ratio,
    float bbox_margin_x_px,
    float bbox_margin_y_px,
    float align_error_x_px,
    float align_error_y_px,
    float align_rate_x_px_s,
    float,
    float,
    float ey_dot_filt,
    float detection_score,
    circle::types::TimestampNs) const {
  if (!params.enable || !params.blind_commit_enable) {
    return false;
  }

  if (!blind_commit_trigger) {
    return false;
  }

  const bool area_ok =
      params.blind_commit_min_area_ratio <= 0.0F ||
      bbox_area_ratio >= params.blind_commit_min_area_ratio;

  const bool margin_ok =
      bbox_margin_x_px <= params.blind_commit_edge_margin_x_px ||
      bbox_margin_y_px <= params.blind_commit_edge_margin_y_px;

  const float hard_edge_margin_y_px =
      params.blind_commit_edge_margin_y_px > 1.0F
          ? std::min(12.0F, params.blind_commit_edge_margin_y_px * 0.15F)
          : 0.0F;
  const float hard_edge_margin_x_px =
      params.blind_commit_edge_margin_x_px > 1.0F
          ? std::min(12.0F, params.blind_commit_edge_margin_x_px * 0.15F)
          : 0.0F;

  const bool terminal_area_ok =
      params.terminal_min_area_ratio <= 0.0F ||
      bbox_area_ratio >= params.terminal_min_area_ratio;

  const bool hard_x_edge =
      area_ok &&
      params.blind_commit_edge_margin_x_px > 1.0F &&
      bbox_margin_x_px <= hard_edge_margin_x_px &&
      terminal_area_ok &&
      (params.blind_commit_edge_max_error_x_px <= 0.0F ||
       align_error_x_px <= params.blind_commit_edge_max_error_x_px * 2.75F) &&
      (params.blind_commit_edge_max_rate_x_px_s <= 0.0F ||
       align_rate_x_px_s <= params.blind_commit_edge_max_rate_x_px_s * 3.50F);

  const bool hard_y_edge =
      area_ok &&
      params.blind_commit_edge_margin_y_px > 1.0F &&
      bbox_margin_y_px <= hard_edge_margin_y_px &&
      terminal_area_ok &&
      (params.blind_commit_edge_max_error_y_px <= 0.0F ||
       align_error_y_px <= params.blind_commit_edge_max_error_y_px * 1.75F) &&
      (params.blind_commit_edge_max_rate_y_px_s <= 0.0F ||
       std::abs(ey_dot_filt) <= params.blind_commit_edge_max_rate_y_px_s * 1.25F);

  const float large_edge_min_area_ratio = std::max(
      0.020F, params.terminal_min_area_ratio * 12.0F);
  const bool large_hard_edge =
      area_ok &&
      bbox_area_ratio >= large_edge_min_area_ratio &&
      (params.snapshot_max_area_ratio <= 0.0F ||
       bbox_area_ratio <= params.snapshot_max_area_ratio) &&
      ((params.blind_commit_edge_margin_x_px > 1.0F &&
        bbox_margin_x_px <= hard_edge_margin_x_px) ||
       (params.blind_commit_edge_margin_y_px > 1.0F &&
        bbox_margin_y_px <= hard_edge_margin_y_px)) &&
      (params.blind_commit_edge_max_error_x_px <= 0.0F ||
       align_error_x_px <= params.blind_commit_edge_max_error_x_px * 2.75F) &&
      (params.blind_commit_edge_max_error_y_px <= 0.0F ||
       align_error_y_px <= params.blind_commit_edge_max_error_y_px * 1.75F);

  if (!area_ok && !margin_ok && !large_hard_edge) {
    return false;
  }

  if (params.blind_commit_edge_vertical_gate_enable && margin_ok && !hard_y_edge) {
    const float y_rate_px_s = std::abs(ey_dot_filt);
    if (std::abs(align_error_y_px) > params.blind_commit_edge_max_error_y_px ||
        y_rate_px_s > params.blind_commit_edge_max_rate_y_px_s) {
      return false;
    }
  }

  if (params.blind_commit_edge_horizontal_gate_enable && margin_ok && !hard_x_edge) {
    const float x_rate_px_s = std::abs(align_rate_x_px_s);
    if (std::abs(align_error_x_px) > params.blind_commit_edge_max_error_x_px ||
        x_rate_px_s > params.blind_commit_edge_max_rate_x_px_s) {
      return false;
    }
  }

  if (params.blind_commit_min_score > 0.0F &&
      detection_score < params.blind_commit_min_score) {
    return false;
  }

  return true;
}

void CommitModule::updateSnapshot(
    CommitState& state,
    bool blind_terminal,
    float roll_rate_sp_rad_s,
    float pitch_rate_sp_rad_s,
    float yaw_rate_sp_rad_s,
    float thrust_z,
    float bbox_area_ratio,
    float bbox_margin_x_px,
    float bbox_margin_y_px,
    float align_error_x_px,
    float align_error_y_px,
    float ex,
    float ey,
    float ex_dot,
    float ey_dot,
    circle::types::TimestampNs now_ns,
    circle::types::TimestampNs detection_stamp_ns) const {
  state.snapshot.valid = true;
  state.snapshot.blind_terminal = blind_terminal;
  state.snapshot.command_stamp_ns = now_ns;
  state.snapshot.detection_stamp_ns = detection_stamp_ns;
  state.snapshot.roll_rate_sp_rad_s = roll_rate_sp_rad_s;
  state.snapshot.pitch_rate_sp_rad_s = pitch_rate_sp_rad_s;
  state.snapshot.yaw_rate_sp_rad_s = yaw_rate_sp_rad_s;
  state.snapshot.thrust_z = thrust_z;
  state.snapshot.bbox_area_ratio = bbox_area_ratio;
  state.snapshot.margin_x_px = bbox_margin_x_px;
  state.snapshot.margin_y_px = bbox_margin_y_px;
  state.snapshot.align_error_x_px = align_error_x_px;
  state.snapshot.align_error_y_px = align_error_y_px;
  state.snapshot.ex = ex;
  state.snapshot.ey = ey;
  state.snapshot.ex_dot = ex_dot;
  state.snapshot.ey_dot = ey_dot;

  if (blind_terminal) {
    state.active = true;
    state.terminal_ready = true;
    state.latch_start_time_ns = now_ns;
  }
}

bool CommitModule::checkTerminalReady(
    const CommitState& state,
    const FACommitParams& params,
    bool final_approach_active,
    float bbox_area_ratio,
    float align_error_x_px,
    float align_error_y_px,
    circle::types::TimestampNs) const {
  if (!params.enable || !final_approach_active || !state.snapshot.valid) {
    return false;
  }

  if (state.terminal_ready) {
    return true;
  }

  const bool terminal_area_ok =
      params.terminal_min_area_ratio <= 0.0F ||
      bbox_area_ratio >= params.terminal_min_area_ratio;

  const bool terminal_align_ok =
      std::abs(align_error_x_px) <= params.terminal_max_error_x_px &&
      std::abs(align_error_y_px) <= params.terminal_max_error_y_px;

  return terminal_area_ok && terminal_align_ok;
}

}  // namespace circle::strike
