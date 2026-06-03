#include "circle/strike/modules/thrust_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

void ThrustManager::reset() {
}

ThrustManagerOutput ThrustManager::compute(
    ThrustManagerState& state,
    const StrikeParams& params,
    bool final_approach_active,
    bool edge_protect_active,
    float edge_thrust_scale,
    float edge_taper_score,
    bool preclimb_xy_gate_active,
    bool preclimb_xy_gate_held,
    const std::optional<circle::types::TimestampNs>& preclimb_xy_released_since_ns,
    bool fresh_detection,
    float bbox_area_ratio,
    float, float,
    float ex_dot_filt, float ey_dot_filt,
    float fx, float fy,
    float vehicle_roll_rad, float vehicle_pitch_rad,
    float deadband_eff_half_w_px, float deadband_eff_half_h_px,
    float align_error_x_px, float align_error_y_px,
    bool commit_aligned,
    circle::types::TimestampNs now_ns,
    float dt_s) const {
  ThrustManagerOutput out;

  const auto& tp = params.thrust;
  const auto& fat = params.final_approach.thrust;

  // Tilt compensation
  out.tilt_denom = tp.enable_tilt_compensation
      ? std::max(std::cos(vehicle_pitch_rad) * std::cos(vehicle_roll_rad),
                 tp.tilt_cos_floor * tp.tilt_cos_floor)
      : 1.0F;

  // Base target: FA uses commit thrust scalar, otherwise constant
  out.base_target = final_approach_active
      ? params.final_approach.commit.thrust_scalar
      : tp.constant_scalar;

  // 8.2: FA thrust taper (area/edge/tilt scores → smoothstep)
  if (fat.thrust_taper_enable && final_approach_active) {
    float area_score = 0.0F;
    float edge_score = 0.0F;
    float tilt_score = 0.0F;

    if (fat.thrust_taper_area_enable) {
      const float start = fat.thrust_taper_start_ratio;
      const float end = fat.thrust_taper_end_ratio;
      area_score = (end > start)
          ? ((bbox_area_ratio - start) / (end - start)) : 1.0F;
    }
    if (fat.thrust_taper_edge_enable) {
      const float start = fat.thrust_taper_edge_start_score;
      const float end = fat.thrust_taper_edge_full_score;
      edge_score = (end > start)
          ? ((edge_taper_score - start) / (end - start)) : 1.0F;
    }
    if (fat.thrust_taper_tilt_enable) {
      const float tilt_mag_rad = std::hypot(vehicle_roll_rad, vehicle_pitch_rad);
      const float start = fat.thrust_taper_tilt_start_rad;
      const float end = fat.thrust_taper_tilt_full_rad;
      tilt_score = (end > start)
          ? ((tilt_mag_rad - start) / (end - start)) : 1.0F;
    }

    const float taper_score = std::clamp(
        std::max(area_score, std::max(edge_score, tilt_score)),
        0.0F, 1.0F);
    const float smooth_t = smoothstep01(taper_score);
    out.fa_thrust_taper_scale =
        1.0F - smooth_t * (1.0F - fat.thrust_taper_min_scale);
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    out.base_target =
        tp.hover_scalar + excess * out.fa_thrust_taper_scale;

    out.fa_ascent_budget_tilt_score = tilt_score;
  }

  // Edge protection thrust scale
  if (edge_protect_active) {
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    out.base_target = tp.hover_scalar + excess * edge_thrust_scale;
  }

  // 8.3: Unaligned slowdown
  if (final_approach_active && !commit_aligned &&
      fat.unaligned_thrust_excess_scale < 1.0F &&
      fat.unaligned_slowdown_end_ratio >
          fat.unaligned_slowdown_start_ratio) {
    const float start = fat.unaligned_slowdown_start_ratio;
    const float end = fat.unaligned_slowdown_end_ratio;
    const float t_raw = (bbox_area_ratio - start) / (end - start);
    const float t = std::clamp(t_raw, 0.0F, 1.0F);
    const float smooth_t = smoothstep01(t);
    out.fa_unaligned_slowdown_scale =
        1.0F - smooth_t * (1.0F - fat.unaligned_thrust_excess_scale);
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    out.base_target =
        tp.hover_scalar + excess * out.fa_unaligned_slowdown_scale;
  }

  // 8.4: Tilt slowdown
  if (fat.tilt_slowdown_enable && final_approach_active &&
      fat.tilt_slowdown_min_scale < 1.0F &&
      fat.tilt_slowdown_end_rad > fat.tilt_slowdown_start_rad) {
    const float tilt_mag_rad = std::hypot(vehicle_roll_rad, vehicle_pitch_rad);
    const float start = fat.tilt_slowdown_start_rad;
    const float end = fat.tilt_slowdown_end_rad;
    const float t_raw = (tilt_mag_rad - start) / (end - start);
    const float smooth_t = smoothstep01(t_raw);
    out.fa_tilt_slowdown_scale =
        1.0F - smooth_t * (1.0F - fat.tilt_slowdown_min_scale);
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    out.base_target =
        tp.hover_scalar + excess * out.fa_tilt_slowdown_scale;
  }

  // 8.5: Vertical drift slowdown
  if (fat.vertical_drift_slowdown_enable && final_approach_active &&
      fresh_detection &&
      fat.vertical_drift_min_scale < 1.0F &&
      fat.vertical_drift_start_px_s > 1.0F) {
    const float drift_px_s = ey_dot_filt * fy;
    if (drift_px_s > fat.vertical_drift_start_px_s) {
      const float excess_rate = drift_px_s - fat.vertical_drift_start_px_s;
      const float t = std::clamp(
          excess_rate / fat.vertical_drift_start_px_s, 0.0F, 1.0F);
      const float smooth_t = smoothstep01(t);
      out.fa_vertical_drift_slowdown_scale =
          1.0F - smooth_t * (1.0F - fat.vertical_drift_min_scale);
      const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
      out.base_target =
          tp.hover_scalar + excess * out.fa_vertical_drift_slowdown_scale;
      out.fa_vertical_drift_slowdown_active =
          out.fa_vertical_drift_slowdown_scale < 0.999F;
    }
  }

  // 8.6: Ascent budget
  if (fat.ascent_budget_enable && final_approach_active &&
      fat.ascent_budget_min_excess_scale < 1.0F) {
    const float tilt_mag_rad = std::hypot(vehicle_roll_rad, vehicle_pitch_rad);
    if (fat.ascent_budget_tilt_full_rad >
        fat.ascent_budget_tilt_start_rad + 1.0e-5F) {
      out.fa_ascent_budget_tilt_score = smoothstep01(
          (tilt_mag_rad - fat.ascent_budget_tilt_start_rad) /
          (fat.ascent_budget_tilt_full_rad -
           fat.ascent_budget_tilt_start_rad));
    } else {
      out.fa_ascent_budget_tilt_score =
          tilt_mag_rad > fat.ascent_budget_tilt_start_rad ? 1.0F : 0.0F;
    }

    if (fresh_detection) {
      const float x_rate_px_s = ex_dot_filt * fx;
      const float y_rate_px_s = ey_dot_filt * fy;
      const float image_speed_px_s = std::hypot(x_rate_px_s, y_rate_px_s);
      const float image_error_px = std::hypot(align_error_x_px, align_error_y_px);

      if (fat.ascent_budget_y_rate_full_px_s >
          fat.ascent_budget_y_rate_start_px_s + 1.0e-5F) {
        out.fa_ascent_budget_y_rate_score = smoothstep01(
            (image_speed_px_s - fat.ascent_budget_y_rate_start_px_s) /
            (fat.ascent_budget_y_rate_full_px_s -
             fat.ascent_budget_y_rate_start_px_s));
      } else {
        out.fa_ascent_budget_y_rate_score =
            image_speed_px_s > fat.ascent_budget_y_rate_start_px_s
                ? 1.0F : 0.0F;
      }

      if (fat.ascent_budget_y_error_full_px >
          fat.ascent_budget_y_error_start_px + 1.0e-5F) {
        out.fa_ascent_budget_y_error_score = smoothstep01(
            (image_error_px - fat.ascent_budget_y_error_start_px) /
            (fat.ascent_budget_y_error_full_px -
             fat.ascent_budget_y_error_start_px));
      } else {
        out.fa_ascent_budget_y_error_score =
            image_error_px > fat.ascent_budget_y_error_start_px
                ? 1.0F : 0.0F;
      }
    }

    const float budget_score =
        std::max(out.fa_ascent_budget_tilt_score,
                 std::max(out.fa_ascent_budget_y_rate_score,
                          out.fa_ascent_budget_y_error_score));
    out.fa_ascent_budget_scale =
        1.0F - budget_score *
                   (1.0F - fat.ascent_budget_min_excess_scale);
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    const float budget_target =
        tp.hover_scalar + excess * out.fa_ascent_budget_scale;
    if (budget_target < out.base_target - 1.0e-5F) {
      out.base_target = budget_target;
      out.fa_ascent_budget_active = true;
    }
  }

  // 8.7: Min thrust floor + budget relaxation
  if (final_approach_active && fat.min_thrust_scalar > 0.0F &&
      out.base_target < fat.min_thrust_scalar) {
    float min_thrust = fat.min_thrust_scalar;
    if (out.fa_ascent_budget_active &&
        fat.min_thrust_budget_relax_enable) {
      const float min_excess =
          std::max(0.0F, fat.min_thrust_scalar - tp.hover_scalar);
      min_thrust = tp.hover_scalar +
                   min_excess * fat.min_thrust_budget_relax_scale;
    }
    if (out.base_target < min_thrust) {
      out.base_target = min_thrust;
    }
  }

  // Preclimb release slowdown
  if (params.preclimb.release_slowdown_enable && preclimb_xy_gate_held &&
      !preclimb_xy_gate_active && !final_approach_active &&
      preclimb_xy_released_since_ns.has_value() &&
      params.preclimb.release_slowdown_s > 0.0F &&
      params.preclimb.release_thrust_excess_scale < 1.0F) {
    const double elapsed = circle::types::secondsBetween(
        *preclimb_xy_released_since_ns, now_ns);
    const float t_raw =
        static_cast<float>(elapsed) / params.preclimb.release_slowdown_s;
    const float smooth_t = smoothstep01(t_raw);
    out.preclimb_release_slowdown_scale =
        params.preclimb.release_thrust_excess_scale +
        smooth_t * (1.0F - params.preclimb.release_thrust_excess_scale);
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    const float min_release =
        std::max(tp.hover_scalar, params.preclimb.release_min_thrust_scalar);
    const float release_target =
        std::max(tp.hover_scalar + excess * out.preclimb_release_slowdown_scale,
                 min_release);
    out.base_target = std::min(out.base_target, release_target);
    out.preclimb_release_slowdown_active =
        out.preclimb_release_slowdown_scale < 0.999F ||
        out.base_target <= min_release + 1.0e-4F;
  }

  // 8.9: Ascent image velocity damping
  if (params.ascent_damping.image_velocity_damping_enable &&
      !final_approach_active && !preclimb_xy_gate_active &&
      fresh_detection &&
      params.ascent_damping.image_velocity_damping_start_px_s > 1.0F) {
    const float image_speed_px_s = std::hypot(
        ex_dot_filt * fx, ey_dot_filt * fy);
    if (image_speed_px_s >
        params.ascent_damping.image_velocity_damping_start_px_s) {
      const float excess_speed =
          image_speed_px_s -
          params.ascent_damping.image_velocity_damping_start_px_s;
      const float t = std::clamp(
          excess_speed /
              params.ascent_damping.image_velocity_damping_start_px_s,
          0.0F, 1.0F);
      const float smooth_t = smoothstep01(t);
      out.ascent_image_velocity_damping_scale =
          1.0F - smooth_t *
              (1.0F - params.ascent_damping.image_velocity_damping_min_scale);
      const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
      out.base_target =
          tp.hover_scalar + excess * out.ascent_image_velocity_damping_scale;
      out.ascent_image_velocity_damping_active =
          out.ascent_image_velocity_damping_scale < 0.999F;
    }
  }

  // 8.8: Tracking deadband priority
  if (params.tracking_deadband_priority.enable && !final_approach_active &&
      !preclimb_xy_gate_active && fresh_detection &&
      params.tracking_deadband_priority.min_excess_scale < 1.0F &&
      fx > 1.0e-6F && fy > 1.0e-6F) {
    const float x_deadband_px = std::max(1.0F, deadband_eff_half_w_px);
    const float y_deadband_px = std::max(1.0F, deadband_eff_half_h_px);
    const float x_over = align_error_x_px / x_deadband_px;
    const float y_over = align_error_y_px / y_deadband_px;
    const float error_scale = std::max(x_over, y_over);

    const float error_window =
        params.tracking_deadband_priority.error_full_scale -
        params.tracking_deadband_priority.error_start_scale;
    const float error_score = error_window > 1.0e-5F
        ? smoothstep01(
              (error_scale -
               params.tracking_deadband_priority.error_start_scale) /
              error_window)
        : 0.0F;

    const float image_speed_px_s = std::hypot(
        ex_dot_filt * fx, ey_dot_filt * fy);
    const float rate_window =
        params.tracking_deadband_priority.rate_full_px_s -
        params.tracking_deadband_priority.rate_start_px_s;
    const float rate_score = rate_window > 1.0e-5F
        ? smoothstep01(
              (image_speed_px_s -
               params.tracking_deadband_priority.rate_start_px_s) /
              rate_window)
        : 0.0F;

    const float priority_score = std::max(error_score, rate_score);
    out.tracking_deadband_priority_scale =
        1.0F - priority_score *
                   (1.0F - params.tracking_deadband_priority.min_excess_scale);
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    const float budget_target =
        tp.hover_scalar + excess * out.tracking_deadband_priority_scale;
    if (budget_target < out.base_target - 1.0e-5F) {
      out.base_target = budget_target;
      out.tracking_deadband_priority_active = true;
    }
  }

  // 8.7.5: Preclimb thrust gating (cap thrust during preclimb hold)
  if (preclimb_xy_gate_active && !final_approach_active &&
      params.preclimb.thrust_excess_scale < 1.0F) {
    const float excess = std::max(0.0F, out.base_target - tp.hover_scalar);
    const float min_thrust = std::max(tp.hover_scalar, params.preclimb.min_thrust_scalar);
    const float gated_target =
        std::max(tp.hover_scalar + excess * params.preclimb.thrust_excess_scale,
                 min_thrust);
    if (gated_target < out.base_target) {
      out.base_target = gated_target;
      out.preclimb_thrust_gated = true;
    }
  }

  // 8.1: Thrust ramp LPF
  float thrust_scalar_smooth = out.base_target;
  if (tp.tracking_ramp_tau_s > 0.0F && dt_s > 0.0F) {
    if (!std::isfinite(state.tracking_thrust_scalar_smooth)) {
      state.tracking_thrust_scalar_smooth = tp.hover_scalar;
    }
    const float a = lpfAlpha(tp.tracking_ramp_tau_s, dt_s);
    state.tracking_thrust_scalar_smooth +=
        a * (out.base_target - state.tracking_thrust_scalar_smooth);
    thrust_scalar_smooth = state.tracking_thrust_scalar_smooth;
  } else {
    state.tracking_thrust_scalar_smooth = out.base_target;
    thrust_scalar_smooth = out.base_target;
  }
  out.thrust_scalar_smooth = thrust_scalar_smooth;

  // Tilt compensation
  float thrust_scalar = thrust_scalar_smooth / out.tilt_denom;

  // 8.10: Thrust hard cap
  float hard_cap = std::numeric_limits<float>::infinity();
  const float vertical_safe =
      std::max(0.01F, out.base_target / std::max(out.tilt_denom, 1.0e-3F));

  if (fat.thrust_hard_cap_enable && final_approach_active) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }
  if (params.preclimb.thrust_hard_cap_enable && preclimb_xy_gate_active) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }
  if (params.preclimb.release_thrust_hard_cap_enable &&
      out.preclimb_release_slowdown_active) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }
  if (out.ascent_image_velocity_damping_active) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }
  if (out.tracking_deadband_priority_active &&
      params.tracking_deadband_priority.hard_cap_enable) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }
  if (out.fa_vertical_drift_slowdown_active) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }
  if (out.fa_ascent_budget_active &&
      fat.ascent_budget_hard_cap_enable) {
    hard_cap = std::min(hard_cap, vertical_safe);
  }

  if (std::isfinite(hard_cap) && thrust_scalar > hard_cap) {
    thrust_scalar = hard_cap;
  }

  // 8.11: Thrust slew rate
  if (tp.slew_rate_scalar_s > 0.0F && dt_s > 0.0F) {
    if (!std::isfinite(state.published_thrust_scalar_smooth)) {
      state.published_thrust_scalar_smooth = thrust_scalar;
    }
    const float max_delta = tp.slew_rate_scalar_s * dt_s;
    const float delta = thrust_scalar - state.published_thrust_scalar_smooth;
    thrust_scalar = state.published_thrust_scalar_smooth +
                    std::clamp(delta, -max_delta, max_delta);
  }
  state.published_thrust_scalar_smooth = thrust_scalar;

  // Output positive scalar; adapter negates for PX4 FRD body.z.
  out.thrust_z = clampThrustScalar(thrust_scalar, tp.scalar_min, tp.scalar_max);

  return out;
}

}  // namespace circle::strike
