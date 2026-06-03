#include "circle/strike/modules/yaw_controller.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike {

YawControllerOutput YawController::compute(
    float image_ex, float rho_scale,
    float yaw_bearing_kp,
    float yaw_track_deadband_rad,
    float yaw_rate_min_rad_s,
    float yaw_rate_max_rad_s) const {
  YawControllerOutput out;

  float bearing_err = std::atan2(image_ex, 1.0F);
  if (std::fabs(bearing_err) < yaw_track_deadband_rad) {
    bearing_err = 0.0F;
  }

  out.yaw_rate_min_eff = yaw_rate_min_rad_s * rho_scale;
  out.yaw_rate_max_eff = yaw_rate_max_rad_s * rho_scale;
  const float yaw_rate_raw = yaw_bearing_kp * bearing_err;
  out.yaw_rate_target = std::clamp(
      yaw_rate_raw, out.yaw_rate_min_eff, out.yaw_rate_max_eff);

  return out;
}

}  // namespace circle::strike
