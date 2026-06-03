#include "circle/strike/modules/directional_dive.hpp"

#include "circle/strike/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike {

void DirectionalDive::reset() {
}

DirectionalDiveOutput DirectionalDive::compute(
    DirectionalDiveState& state,
    const DirectionalDiveParams& params,
    const TiltGuardParams& tilt_guard_params,
    bool final_approach_active,
    float max_edge_ratio,
    float tilt_angle,
    float image_ex,
    float image_ey,
    float ex_dot_filt,
    float ey_dot_filt,
    float vehicle_pitch_rad,
    float vehicle_roll_rad,
    float gov_scale,
    float current_thrust_z,
    float current_roll_rate,
    float current_pitch_rate,
    circle::types::TimestampNs detection_capture_ns,
    circle::types::TimestampNs now_ns,
    float dt_s) const {
  (void)dt_s;  // Reserved for future use
  DirectionalDiveOutput out;

  if (!params.enable || !final_approach_active || max_edge_ratio <= params.start_edge_ratio) {
    return out;
  }

  float dive_blend = smoothstep01(
      (max_edge_ratio - params.start_edge_ratio) / (params.end_edge_ratio - params.start_edge_ratio));

  const double detection_age_s = circle::types::secondsBetween(
      detection_capture_ns, now_ns);
  if (detection_age_s > params.detection_max_age_s) {
    dive_blend = 0.0F;
  }

  if (tilt_guard_params.enable && tilt_angle > tilt_guard_params.max_tilt_rad) {
    dive_blend = 0.0F;
  }

  state.dive_blend = dive_blend;
  out.dive_blend = dive_blend;

  if (dive_blend <= 0.01F) {
    return out;
  }

  out.active = true;

  const float cos_p = std::cos(vehicle_pitch_rad);
  const float sin_p = std::sin(vehicle_pitch_rad);
  const float cos_r = std::cos(vehicle_roll_rad);
  const float sin_r = std::sin(vehicle_roll_rad);

  const float ex_pred = std::clamp(image_ex + ex_dot_filt * params.lead_time_s, -0.8F, 0.8F);
  const float ey_pred = std::clamp(image_ey + ey_dot_filt * params.lead_time_s, -0.8F, 0.8F);
  const float pred_ray_x = ex_pred * cos_p + sin_p;
  const float pred_ray_y = ey_pred * cos_r + (ex_pred * sin_p - cos_p) * sin_r;
  const float pred_ray_z = ey_pred * sin_r + (cos_p - ex_pred * sin_p) * cos_r;
  const float pred_mag = std::sqrt(pred_ray_x * pred_ray_x + pred_ray_y * pred_ray_y + pred_ray_z * pred_ray_z);
  const float pred_dir_x = pred_ray_x / pred_mag;
  const float pred_dir_y = pred_ray_y / pred_mag;
  const float pred_dir_z = pred_ray_z / pred_mag;

  float dive_roll = pred_dir_y * params.dive_roll_gain;
  float dive_pitch = pred_dir_x * params.dive_pitch_gain;

  const float horiz_mag = std::sqrt(pred_dir_x * pred_dir_x + pred_dir_y * pred_dir_y);
  if (pred_dir_z < -0.5F && horiz_mag > 0.05F) {
    const float boost = std::min(params.min_horizontal_boost, 1.0F / std::max(0.1F, horiz_mag));
    dive_roll *= boost;
    dive_pitch *= boost;
  }

  dive_roll *= gov_scale;
  dive_pitch *= gov_scale;

  float dive_thrust = params.cruise_thrust;
  if (pred_dir_z < -0.3F) {
    dive_thrust = params.climb_thrust;
  } else if (pred_dir_z > 0.3F) {
    dive_thrust = params.descend_thrust;
  }

  out.roll_rate_rad_s = current_roll_rate * (1.0F - dive_blend) + dive_roll * dive_blend;
  out.pitch_rate_rad_s = current_pitch_rate * (1.0F - dive_blend) + dive_pitch * dive_blend;
  out.thrust_z = current_thrust_z * (1.0F - dive_blend) + dive_thrust * dive_blend;

  return out;
}

}  // namespace circle::strike
