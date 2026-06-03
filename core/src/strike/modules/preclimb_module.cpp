#include "circle/strike/modules/preclimb_module.hpp"

#include <algorithm>
#include <cmath>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

void PreclimbModule::reset() {
}

PreclimbOutput PreclimbModule::compute(
    PreclimbState& state,
    const PreclimbParams& params,
    bool final_approach_active,
    bool fresh_detection,
    bool strike_confident,
    float image_ex, float image_ey,
    float ex_dot_filt, float ey_dot_filt,
    float fx, float fy,
    float bbox_margin_x_px, float bbox_margin_y_px,
    float vehicle_roll_rad, float vehicle_pitch_rad,
    float roll_rate_des, float pitch_rate_des,
    circle::types::TimestampNs now_ns) const {
  PreclimbOutput out;
  out.roll_rate_rad_s = roll_rate_des;
  out.pitch_rate_rad_s = pitch_rate_des;

  if (!params.xy_gate_enable) {
    state.xy_gate_released = true;
    return out;
  }

  if (final_approach_active) {
    state.xy_stable_since_ns.reset();
    if (!state.xy_released_since_ns.has_value()) {
      state.xy_released_since_ns = now_ns;
    }
    state.xy_gate_released = true;
    return out;
  }

  const bool was_released = state.xy_gate_released;

  out.error_x_px = std::abs(image_ex * fx);
  out.error_y_px = std::abs(image_ey * fy);
  out.rate_x_px_s = std::abs(ex_dot_filt * fx);
  out.rate_y_px_s = std::abs(ey_dot_filt * fy);

  out.error_ok = out.error_x_px <= params.xy_error_gate_x_px &&
                 out.error_y_px <= params.xy_error_gate_y_px;
  out.rate_ok = out.rate_x_px_s <= params.xy_rate_gate_x_px_s &&
                out.rate_y_px_s <= params.xy_rate_gate_y_px_s;
  out.margin_ok = bbox_margin_x_px >= params.clear_margin_x_px &&
                  bbox_margin_y_px >= params.clear_margin_y_px;
  out.level_ok = !params.level_gate_enable ||
                 (std::abs(vehicle_roll_rad) <= params.level_max_roll_rad &&
                  std::abs(vehicle_pitch_rad) <= params.level_max_pitch_rad);

  const bool input_ok = fresh_detection && out.error_ok && out.rate_ok &&
                        out.margin_ok && out.level_ok;

  if (state.xy_gate_released) {
    if (!state.xy_released_since_ns.has_value()) {
      state.xy_released_since_ns = now_ns;
    }
    if (input_ok) {
      if (!state.xy_stable_since_ns.has_value()) {
        state.xy_stable_since_ns = now_ns;
      }
      out.hold_elapsed_s = circle::types::secondsBetween(
          *state.xy_stable_since_ns, now_ns);
    } else {
      state.xy_stable_since_ns.reset();
      out.hold_elapsed_s = static_cast<double>(params.xy_hold_s);
    }
  } else if (input_ok) {
    if (!state.xy_stable_since_ns.has_value()) {
      state.xy_stable_since_ns = now_ns;
    }
    out.hold_elapsed_s = circle::types::secondsBetween(
        *state.xy_stable_since_ns, now_ns);
    state.xy_gate_released =
        out.hold_elapsed_s >= static_cast<double>(params.xy_hold_s);
    if (state.xy_gate_released && !was_released) {
      state.xy_released_since_ns = now_ns;
    }
  } else {
    state.xy_stable_since_ns.reset();
    state.xy_released_since_ns.reset();
    out.hold_elapsed_s = 0.0;
    state.xy_gate_released = false;
  }

  const bool gate_held = state.xy_gate_released;
  if (!gate_held) {
    out.xy_gate_active = true;
    out.thrust_scale = params.thrust_excess_scale;
  }

  // Safe hold: blend toward level when input is poor or tilt is high
  if (out.xy_gate_active && params.safe_hold_enable &&
      params.safe_hold_max_rate_rad_s > 0.0F) {
    const float image_scale =
        std::clamp(params.safe_hold_image_rate_scale, 0.0F, 1.0F);
    const bool pitch_recenters_y =
        (image_ey > 0.0F && pitch_rate_des < 0.0F) ||
        (image_ey < 0.0F && pitch_rate_des > 0.0F);
    const float y_error_over_gate = std::clamp(
        (out.error_y_px - params.xy_error_gate_y_px) /
            std::max(1.0F, params.xy_error_gate_y_px),
        0.0F, 1.0F);
    const float y_recenter_weight =
        pitch_recenters_y ? smoothstep01(y_error_over_gate) : 0.0F;
    const float pitch_recenter_scale_target = std::max(image_scale, 0.85F);
    const float pitch_image_scale = std::min(
        1.0F,
        image_scale + y_recenter_weight * (pitch_recenter_scale_target - image_scale));

    float roll_out = std::clamp(
        roll_rate_des * image_scale,
        -params.safe_hold_max_rate_rad_s,
        params.safe_hold_max_rate_rad_s);
    float pitch_out = std::clamp(
        pitch_rate_des * pitch_image_scale,
        -params.safe_hold_max_rate_rad_s,
        params.safe_hold_max_rate_rad_s);

    const bool unreliable = !fresh_detection || !strike_confident;
    const bool edge_bad = !out.margin_ok;
    const bool rate_bad = !out.rate_ok;
    const bool error_bad = !out.error_ok;

    float poor_blend = 0.0F;
    if (unreliable) poor_blend = std::max(poor_blend, 0.45F);
    if (edge_bad) poor_blend = std::max(poor_blend, 0.40F);
    if (rate_bad) poor_blend = std::max(poor_blend, 0.25F);
    if (error_bad) poor_blend = std::max(poor_blend, 0.10F);

    float pitch_poor_blend = poor_blend;
    if (pitch_recenters_y) {
      pitch_poor_blend *= (1.0F - 0.80F * y_recenter_weight);
    }

    const float tilt_mag_rad =
        std::max(std::abs(vehicle_roll_rad), std::abs(vehicle_pitch_rad));
    const float tilt_t =
        (params.safe_hold_tilt_full_rad >
         params.safe_hold_tilt_start_rad + 1.0e-5F)
            ? std::clamp(
                  (tilt_mag_rad - params.safe_hold_tilt_start_rad) /
                      (params.safe_hold_tilt_full_rad -
                       params.safe_hold_tilt_start_rad),
                  0.0F, 1.0F)
            : (tilt_mag_rad > params.safe_hold_tilt_start_rad ? 1.0F : 0.0F);
    const float tilt_blend = smoothstep01(tilt_t);
    float pitch_tilt_blend = tilt_blend;
    if (pitch_recenters_y) {
      pitch_tilt_blend *= (1.0F - 0.85F * y_recenter_weight);
    }

    const float roll_safe_blend = std::max(poor_blend, tilt_blend);
    const float pitch_safe_blend = std::max(pitch_poor_blend, pitch_tilt_blend);
    out.safe_hold_blend = std::max(roll_safe_blend, pitch_safe_blend);

    if (params.safe_hold_level_kp > 0.0F &&
        params.safe_hold_level_max_rad_s > 0.0F &&
        out.safe_hold_blend > 0.001F) {
      out.safe_hold_active = true;
      const float roll_level_rate = std::clamp(
          -params.safe_hold_level_kp * vehicle_roll_rad,
          -params.safe_hold_level_max_rad_s,
          params.safe_hold_level_max_rad_s);
      const float pitch_level_rate = std::clamp(
          -params.safe_hold_level_kp * vehicle_pitch_rad,
          -params.safe_hold_level_max_rad_s,
          params.safe_hold_level_max_rad_s);
      roll_out = roll_out * (1.0F - roll_safe_blend) +
                 roll_level_rate * roll_safe_blend;
      pitch_out = pitch_out * (1.0F - pitch_safe_blend) +
                  pitch_level_rate * pitch_safe_blend;
    }

    out.roll_rate_rad_s = roll_out;
    out.pitch_rate_rad_s = pitch_out;
  }

  // Level assist: smoothstep blend to level near tilt limits
  const bool level_assist_safe =
      out.error_ok && out.rate_ok && out.margin_ok;
  if (out.xy_gate_active && level_assist_safe &&
      params.level_assist_enable &&
      params.level_assist_kp > 0.0F &&
      params.level_assist_max_rad_s > 0.0F &&
      params.level_gate_enable &&
      (params.level_max_roll_rad > 0.0F ||
       params.level_max_pitch_rad > 0.0F)) {
    out.level_assist_active = true;
    const float roll_start = std::max(
        0.0F, params.level_max_roll_rad - params.level_assist_band_rad);
    const float pitch_start = std::max(
        0.0F, params.level_max_pitch_rad - params.level_assist_band_rad);
    const float roll_t =
        params.level_assist_band_rad > 1.0e-5F
            ? std::clamp(
                  (std::abs(vehicle_roll_rad) - roll_start) /
                      params.level_assist_band_rad,
                  0.0F, 1.0F)
            : (std::abs(vehicle_roll_rad) > params.level_max_roll_rad ? 1.0F : 0.0F);
    const float pitch_t =
        params.level_assist_band_rad > 1.0e-5F
            ? std::clamp(
                  (std::abs(vehicle_pitch_rad) - pitch_start) /
                      params.level_assist_band_rad,
                  0.0F, 1.0F)
            : (std::abs(vehicle_pitch_rad) > params.level_max_pitch_rad ? 1.0F : 0.0F);
    const float assist_t = std::max(roll_t, pitch_t);
    out.level_assist_blend = smoothstep01(assist_t);

    const float roll_level_rate = std::clamp(
        -params.level_assist_kp * vehicle_roll_rad,
        -params.level_assist_max_rad_s,
        params.level_assist_max_rad_s);
    const float pitch_level_rate = std::clamp(
        -params.level_assist_kp * vehicle_pitch_rad,
        -params.level_assist_max_rad_s,
        params.level_assist_max_rad_s);
    out.roll_rate_rad_s =
        out.roll_rate_rad_s * (1.0F - out.level_assist_blend) +
        roll_level_rate * out.level_assist_blend;
    out.pitch_rate_rad_s =
        out.pitch_rate_rad_s * (1.0F - out.level_assist_blend) +
        pitch_level_rate * out.level_assist_blend;
  }

  return out;
}

float PreclimbModule::computeReleaseSlowdown(
    const PreclimbParams& params,
    const PreclimbState& state,
    circle::types::TimestampNs now_ns) {
  if (!params.release_slowdown_enable ||
      params.release_slowdown_s <= 0.0F ||
      !state.xy_released_since_ns.has_value()) {
    return 1.0F;
  }
  const double elapsed = circle::types::secondsBetween(
      *state.xy_released_since_ns, now_ns);
  if (elapsed >= static_cast<double>(params.release_slowdown_s)) {
    return 1.0F;
  }
  const float t = static_cast<float>(
      elapsed / static_cast<double>(params.release_slowdown_s));
  return std::clamp(t, 0.0F, 1.0F);
}

}  // namespace circle::strike
