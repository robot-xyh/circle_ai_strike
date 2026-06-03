#include "circle/strike/modules/tilt_guard.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike {

TiltGuardOutput TiltGuard::compute(
    const TiltGuardParams& params,
    const circle::types::FcState& vehicle,
    float current_roll_rate,
    float current_pitch_rate,
    float max_edge_ratio) const {
  TiltGuardOutput out;
  out.max_edge_ratio = max_edge_ratio;

  if (!params.enable) {
    return out;
  }

  const float cos_p = std::cos(vehicle.pitch_rad);
  const float sin_p = std::sin(vehicle.pitch_rad);
  const float cos_r = std::cos(vehicle.roll_rad);
  const float sin_r = std::sin(vehicle.roll_rad);
  const float cam_axis_z = cos_p * cos_r;
  const float tilt_angle = std::acos(std::clamp(std::abs(cam_axis_z), -1.0F, 1.0F));
  out.tilt_angle = tilt_angle;

  if (tilt_angle <= params.max_tilt_rad) {
    return out;
  }

  out.active = true;
  const float pitch_contrib = std::abs(sin_p);
  const float roll_contrib = std::abs(sin_r);
  const float half_max = params.max_tilt_rad * 0.5F;

  if (pitch_contrib > roll_contrib) {
    if (vehicle.pitch_rad < 0) {
      out.pitch_rate_correction = std::max(0.0F, current_pitch_rate);
    }
    const float pitch_target = (vehicle.pitch_rad > 0) ? half_max : -half_max;
    out.pitch_rate_correction += params.recovery_kp * (pitch_target - vehicle.pitch_rad);
  }

  if (roll_contrib >= pitch_contrib) {
    if (vehicle.roll_rad > 0) {
      out.roll_rate_correction = std::min(0.0F, current_roll_rate);
    } else {
      out.roll_rate_correction = std::max(0.0F, current_roll_rate);
    }
    const float roll_target = (vehicle.roll_rad > 0) ? half_max : -half_max;
    out.roll_rate_correction += params.recovery_kp * (roll_target - vehicle.roll_rad);
  }

  return out;
}

}  // namespace circle::strike
