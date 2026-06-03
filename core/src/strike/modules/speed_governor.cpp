#include "circle/strike/modules/speed_governor.hpp"
#include "circle/strike/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike {

SpeedGovernorOutput SpeedGovernor::compute(
    const SpeedGovernorParams& params,
    bool final_approach_active,
    bool vehicle_velocity_valid,
    float vehicle_vxy_m_s,
    float vehicle_roll_rad,
    float vehicle_pitch_rad,
    float roll_rate_des,
    float pitch_rate_des) const {
  SpeedGovernorOutput out;
  out.roll_rate_rad_s = roll_rate_des;
  out.pitch_rate_rad_s = pitch_rate_des;

  if (!params.enable ||
      !vehicle_velocity_valid ||
      params.full_m_s <= params.start_m_s + 1.0e-5F ||
      params.min_image_scale >= 1.0F) {
    return out;
  }

  const float t_raw =
      (vehicle_vxy_m_s - params.start_m_s) /
      (params.full_m_s - params.start_m_s);
  out.blend = smoothstep01(t_raw);

  if (out.blend <= 0.001F) {
    return out;
  }

  out.active = true;
  out.scale = 1.0F - out.blend * (1.0F - params.min_image_scale);

  roll_rate_des *= out.scale;
  pitch_rate_des *= out.scale;

  const float roll_blend = final_approach_active
      ? std::min(out.blend, params.fa_roll_level_blend_max)
      : out.blend;
  const float pitch_blend = final_approach_active
      ? std::min(out.blend, params.fa_pitch_level_blend_max)
      : out.blend;

  if (params.level_kp > 0.0F && params.level_max_rad_s > 0.0F) {
    const float roll_level_rate = std::clamp(
        -params.level_kp * vehicle_roll_rad,
        -params.level_max_rad_s,
        params.level_max_rad_s);
    const float pitch_level_rate = std::clamp(
        -params.level_kp * vehicle_pitch_rad,
        -params.level_max_rad_s,
        params.level_max_rad_s);

    roll_rate_des = roll_rate_des * (1.0F - roll_blend) +
                    roll_level_rate * roll_blend;
    pitch_rate_des = pitch_rate_des * (1.0F - pitch_blend) +
                     pitch_level_rate * pitch_blend;
  }

  out.roll_rate_rad_s = roll_rate_des;
  out.pitch_rate_rad_s = pitch_rate_des;

  return out;
}

}  // namespace circle::strike
