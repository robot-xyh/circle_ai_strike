#pragma once

namespace circle::strike {

struct YawControllerOutput {
  float yaw_rate_target{0.0F};
  float yaw_rate_min_eff{0.0F};
  float yaw_rate_max_eff{0.0F};
};

class YawController {
 public:
  YawControllerOutput compute(float image_ex, float rho_scale,
                              float yaw_bearing_kp,
                              float yaw_track_deadband_rad,
                              float yaw_rate_min_rad_s,
                              float yaw_rate_max_rad_s) const;
};

}  // namespace circle::strike
