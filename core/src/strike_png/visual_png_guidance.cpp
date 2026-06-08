#include "circle/strike_png/visual_png_guidance.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike_png {

namespace {

float finiteOrZero(float value) {
  return std::isfinite(value) ? value : 0.0F;
}

float ramp01(float value) {
  const float x = std::clamp(value, 0.0F, 1.0F);
  return x * x * (3.0F - 2.0F * x);
}

float edgeGuard(float error,
                float start,
                float full,
                float kp_rate,
                float min_rate,
                float max_rate) {
  const float e = finiteOrZero(error);
  const float abs_e = std::abs(e);
  const float range = std::max(1.0e-3F, full - start);
  if (abs_e <= start) {
    return 0.0F;
  }

  const float weight = ramp01((abs_e - start) / range);
  float command = kp_rate * e * weight;
  const float weighted_min = std::max(0.0F, min_rate) * weight;
  if (weighted_min > 0.0F && std::abs(command) < weighted_min) {
    command = std::copysign(weighted_min, e);
  }
  const float limit = std::max(0.0F, max_rate);
  return limit > 0.0F ? std::clamp(command, -limit, limit) : command;
}

float pursuitFallback(float error,
                      float png_command,
                      float start,
                      float full,
                      float kp_rate,
                      float min_rate,
                      float max_rate,
                      float png_weak_rate) {
  if (std::abs(finiteOrZero(png_command)) >
      std::max(0.0F, png_weak_rate)) {
    return 0.0F;
  }
  return edgeGuard(error, start, full, kp_rate, min_rate, max_rate);
}

float terminalStaleLateralTrim(float error,
                               float area_ratio,
                               float measurement_age_s,
                               float area_start,
                               float area_full,
                               float stale_start_s,
                               float stale_full_s,
                               float kp_rate,
                               float max_rate) {
  const float area_range = std::max(1.0e-6F, area_full - area_start);
  const float stale_range = std::max(1.0e-3F, stale_full_s - stale_start_s);
  const float area_weight =
      ramp01((finiteOrZero(area_ratio) - area_start) / area_range);
  const float stale_weight =
      ramp01((finiteOrZero(measurement_age_s) - stale_start_s) / stale_range);
  const float weight = area_weight * stale_weight;
  const float limit = std::max(0.0F, max_rate);
  const float command = finiteOrZero(kp_rate) * finiteOrZero(error) * weight;
  return limit > 0.0F ? std::clamp(command, -limit, limit) : command;
}

float terminalInterceptWeight(float area_ratio, float area_start, float area_full) {
  const float area_range = std::max(1.0e-6F, area_full - area_start);
  return ramp01((finiteOrZero(area_ratio) - area_start) / area_range);
}

float terminalTiltAimWeight(float area_ratio, float area_start, float area_full) {
  const float area_range = std::max(1.0e-6F, area_full - area_start);
  return ramp01((finiteOrZero(area_ratio) - area_start) / area_range);
}

float terminalCrossingWeight(float area_ratio,
                             float rate_norm_s,
                             float area_start,
                             float area_full,
                             float rate_start,
                             float rate_full) {
  const float area_range = std::max(1.0e-6F, area_full - area_start);
  const float rate_range = std::max(1.0e-3F, rate_full - rate_start);
  const float area_weight =
      ramp01((finiteOrZero(area_ratio) - area_start) / area_range);
  const float rate_weight =
      ramp01((std::abs(finiteOrZero(rate_norm_s)) - rate_start) / rate_range);
  return area_weight * rate_weight;
}

float terminalForwardSpeedGuardWeight(float area_ratio,
                                      float ownship_forward_speed_m_s,
                                      float area_start,
                                      float area_full,
                                      float speed_start,
                                      float speed_full) {
  const float area_range = std::max(1.0e-6F, area_full - area_start);
  const float speed_range = std::max(1.0e-3F, speed_full - speed_start);
  const float area_weight =
      ramp01((finiteOrZero(area_ratio) - area_start) / area_range);
  const float speed_weight =
      ramp01((finiteOrZero(ownship_forward_speed_m_s) - speed_start) /
             speed_range);
  return area_weight * speed_weight;
}

}  // namespace

VisualPngGuidanceOutput VisualPngGuidance::compute(
    const VisualPngGuidanceParams& params,
    const VisualPngGuidanceInput& input) const {
  VisualPngGuidanceOutput out;

  float ex_dot_inertial = finiteOrZero(input.ex_dot);
  float ey_dot_inertial = finiteOrZero(input.ey_dot);
  if (params.derotate_body_rates && input.derotate_rate_valid) {
    ex_dot_inertial += finiteOrZero(input.pitch_rate_rad_s) *
                       params.derotate_pitch_to_x_gain;
    ey_dot_inertial -= finiteOrZero(input.roll_rate_rad_s) *
                       params.derotate_roll_to_y_gain;
  }

  const float residual_limit =
      std::max(0.0F, params.residual_rate_limit_rad_s);
  if (residual_limit > 0.0F) {
    ex_dot_inertial =
        std::clamp(ex_dot_inertial, -residual_limit, residual_limit);
    ey_dot_inertial =
        std::clamp(ey_dot_inertial, -residual_limit, residual_limit);
  }

  const float area_term = std::sqrt(std::max(0.0F, input.bbox_area_ratio));
  out.closure_scale = std::max(
      0.0F, params.closure_base_scale + params.closure_area_gain * area_term);
  out.ex_dot_inertial = ex_dot_inertial;
  out.ey_dot_inertial = ey_dot_inertial;
  float terminal_aim_ex = 0.0F;
  float terminal_aim_ey = finiteOrZero(params.vertical_aim_ey);
  if (input.attitude_valid && params.terminal_tilt_aim_max_offset_norm > 0.0F) {
    const float tilt_weight = terminalTiltAimWeight(
        input.bbox_area_ratio,
        params.terminal_tilt_aim_area_ratio_start,
        params.terminal_tilt_aim_area_ratio_full);
    if (tilt_weight > 0.0F) {
      const float max_offset = std::max(0.0F,
                                        params.terminal_tilt_aim_max_offset_norm);
      terminal_aim_ex = std::clamp(params.terminal_tilt_aim_roll_gain *
                                       finiteOrZero(input.roll_rad) * tilt_weight,
                                   -max_offset,
                                   max_offset);
      terminal_aim_ey += std::clamp(params.terminal_tilt_aim_pitch_gain *
                                        finiteOrZero(input.pitch_rad) * tilt_weight,
                                    -max_offset,
                                    max_offset);
    }
  }
  out.terminal_aim_ex = terminal_aim_ex;
  out.terminal_aim_ey = terminal_aim_ey;
  const float ex_error = finiteOrZero(input.ex) - terminal_aim_ex;
  const float terminal_ey_error = finiteOrZero(input.ey) - terminal_aim_ey;

  float roll_ff = params.nav_ratio_x * out.closure_scale * ex_dot_inertial;
  float pitch_ff = -params.nav_ratio_y * out.closure_scale * ey_dot_inertial;

  const float ff_limit = std::max(0.0F, params.max_feedforward_rad_s);
  if (ff_limit > 0.0F) {
    roll_ff = std::clamp(roll_ff, -ff_limit, ff_limit);
    pitch_ff = std::clamp(pitch_ff, -ff_limit, ff_limit);
  }
  out.roll_png_ff_rad_s = roll_ff;
  out.pitch_png_ff_rad_s = pitch_ff;

  const float roll_trim = params.fov_trim_kp_rate * ex_error;
  const float pitch_trim = -params.fov_trim_kp_rate * terminal_ey_error;
  out.roll_fov_trim_rad_s = roll_trim;
  out.pitch_fov_trim_rad_s = pitch_trim;
  const float fov_trim_fade_range = std::max(
      1.0e-6F,
      params.fov_trim_fade_area_ratio_full -
          params.fov_trim_fade_area_ratio_start);
  const float fov_trim_scale =
      1.0F - ramp01((finiteOrZero(input.bbox_area_ratio) -
                     params.fov_trim_fade_area_ratio_start) /
                    fov_trim_fade_range);
  if (params.pursuit_fallback_enable) {
    out.roll_pursuit_fallback_rad_s =
        pursuitFallback(input.ex,
                        roll_ff,
                        params.pursuit_fallback_start_norm,
                        params.pursuit_fallback_full_norm,
                        params.pursuit_fallback_kp_rate,
                        params.pursuit_fallback_min_rate_rad_s,
                        params.pursuit_fallback_max_rate_rad_s,
                        params.pursuit_fallback_png_weak_rate_rad_s);
    out.pitch_pursuit_fallback_rad_s =
        -pursuitFallback(input.ey,
                         pitch_ff,
                         params.pursuit_fallback_start_norm,
                         params.pursuit_fallback_full_norm,
                         params.pursuit_fallback_kp_rate,
                         params.pursuit_fallback_min_rate_rad_s,
                         params.pursuit_fallback_max_rate_rad_s,
                         params.pursuit_fallback_png_weak_rate_rad_s);
  }
  if (params.edge_guard_enable) {
    out.roll_edge_guard_rad_s =
        edgeGuard(input.ex,
                  params.edge_guard_start_norm,
                  params.edge_guard_full_norm,
                  params.edge_guard_kp_rate,
                  params.edge_guard_min_rate_rad_s,
                  params.edge_guard_max_rate_rad_s);
    out.pitch_edge_guard_rad_s =
        -std::max(0.0F, params.edge_guard_pitch_scale) *
        edgeGuard(input.ey,
                  params.edge_guard_start_norm,
                  params.edge_guard_full_norm,
                  params.edge_guard_kp_rate,
                  params.edge_guard_min_rate_rad_s,
                  params.edge_guard_max_rate_rad_s);
  }
  if (params.terminal_stale_lateral_trim_enable) {
    out.roll_terminal_stale_trim_rad_s =
        terminalStaleLateralTrim(input.ex,
                                 input.bbox_area_ratio,
                                 input.measurement_age_s,
                                 params.terminal_stale_lateral_trim_area_ratio_start,
                                 params.terminal_stale_lateral_trim_area_ratio_full,
                                 params.terminal_stale_lateral_trim_stale_s_start,
                                 params.terminal_stale_lateral_trim_stale_s_full,
                                 params.terminal_stale_lateral_trim_kp_rate,
                                 params.terminal_stale_lateral_trim_max_rate_rad_s);
  }
  if (params.terminal_intercept_enable) {
    const float weight =
        terminalInterceptWeight(input.bbox_area_ratio,
                                params.terminal_intercept_area_ratio_start,
                                params.terminal_intercept_area_ratio_full);
    if (weight > 0.0F) {
      out.terminal_intercept_active = true;
      out.terminal_intercept_lead_s =
          std::clamp(params.terminal_intercept_lead_s, 0.0F, 1.0F);
      out.terminal_future_ex =
          ex_error + ex_dot_inertial * out.terminal_intercept_lead_s;
      out.terminal_future_ey =
          terminal_ey_error + ey_dot_inertial * out.terminal_intercept_lead_s;
      const float limit = std::max(0.0F, params.terminal_intercept_max_rate_rad_s);
      const float roll =
          params.terminal_intercept_kp_rate * out.terminal_future_ex * weight;
      const float pitch =
          -params.terminal_intercept_kp_rate * out.terminal_future_ey * weight;
      out.roll_terminal_intercept_rad_s =
          limit > 0.0F ? std::clamp(roll, -limit, limit) : roll;
      out.pitch_terminal_intercept_rad_s =
          limit > 0.0F ? std::clamp(pitch, -limit, limit) : pitch;
    }
  }
  if (params.terminal_crossing_enable) {
    const float crossing_weight =
        terminalCrossingWeight(input.bbox_area_ratio,
                               ey_dot_inertial,
                               params.terminal_crossing_area_ratio_start,
                               params.terminal_crossing_area_ratio_full,
                               params.terminal_crossing_rate_start_norm_s,
                               params.terminal_crossing_rate_full_norm_s);
    if (crossing_weight > 0.0F) {
      out.terminal_crossing_active = true;
      out.terminal_crossing_weight = crossing_weight;
      const float limit =
          std::max(0.0F, params.terminal_crossing_max_rate_rad_s);
      const float pitch =
          -params.terminal_crossing_kd_rate * ey_dot_inertial * crossing_weight;
      out.pitch_terminal_crossing_rad_s =
          limit > 0.0F ? std::clamp(pitch, -limit, limit) : pitch;
    }
  }
  float positive_pitch_scale = 1.0F;
  if (params.terminal_forward_speed_guard_enable &&
      input.ownship_forward_speed_valid) {
    const float speed_weight = terminalForwardSpeedGuardWeight(
        input.bbox_area_ratio,
        input.ownship_forward_speed_m_s,
        params.terminal_forward_speed_guard_area_ratio_start,
        params.terminal_forward_speed_guard_area_ratio_full,
        params.terminal_forward_speed_guard_start_m_s,
        params.terminal_forward_speed_guard_full_m_s);
    if (speed_weight > 0.0F) {
      out.terminal_forward_speed_guard_active = true;
      out.terminal_forward_speed_guard_weight = speed_weight;
      const float min_scale = std::clamp(
          params.terminal_forward_speed_guard_min_positive_pitch_scale,
          0.0F,
          1.0F);
      positive_pitch_scale = 1.0F - (1.0F - min_scale) * speed_weight;
      out.terminal_forward_speed_guard_scale = positive_pitch_scale;
    }
  }
  const float roll_limit = std::max(0.0F, input.max_roll_rate_rad_s);
  const float pitch_limit = std::max(0.0F, input.max_pitch_rate_rad_s);
  float pitch_command = pitch_ff + pitch_trim * fov_trim_scale +
                        out.pitch_pursuit_fallback_rad_s +
                        out.pitch_edge_guard_rad_s +
                        out.pitch_terminal_intercept_rad_s +
                        out.pitch_terminal_crossing_rad_s;
  if (pitch_command > 0.0F && positive_pitch_scale < 1.0F) {
    pitch_command *= positive_pitch_scale;
  }

  out.roll_rate_rad_s = std::clamp(roll_ff + roll_trim * fov_trim_scale +
                                       out.roll_pursuit_fallback_rad_s +
                                       out.roll_edge_guard_rad_s +
                                       out.roll_terminal_stale_trim_rad_s +
                                       out.roll_terminal_intercept_rad_s,
                                   -roll_limit,
                                   roll_limit);
  out.pitch_rate_rad_s =
      std::clamp(pitch_command, -pitch_limit, pitch_limit);

  // The math above is the PX4 baseline (lateral=+1, longitudinal=-1). Re-map to
  // the configured convention with a final ±1 multiply; defaults leave PX4
  // unchanged, BF top-cam sets longitudinal_output_sign=+1 to mirror pitch.
  out.roll_rate_rad_s *= params.lateral_output_sign;
  out.pitch_rate_rad_s *= -params.longitudinal_output_sign;
  out.active = true;
  return out;
}

}  // namespace circle::strike_png
