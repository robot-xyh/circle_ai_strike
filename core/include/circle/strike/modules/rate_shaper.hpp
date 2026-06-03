#pragma once

#include <cstdint>

namespace circle::strike {

struct RateShaperState {
  float roll_rate_filt{0.0F};
  float pitch_rate_filt{0.0F};
  float roll_rate_slew{0.0F};
  float pitch_rate_slew{0.0F};
  float yaw_rate_filt{0.0F};
};

struct RateShaperOutput {
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float yaw_rate_rad_s{0.0F};
  float roll_softcap_factor{1.0F};
  float pitch_softcap_factor{1.0F};
};

class RateShaper {
 public:
  void reset();
  
  RateShaperOutput compute(float roll_cmd, float pitch_cmd, float yaw_target,
                           float roll_attitude, float pitch_attitude,
                           float max_roll_angle_rad, float max_pitch_angle_rad,
                           float softcap_band_rad,
                           float rate_lpf_tau_s, float max_jerk_rad_s2,
                           float max_roll_rate_rad_s, float max_pitch_rate_rad_s,
                           float yaw_rate_lpf_tau_s, bool yaw_lock_enabled,
                           float rho_scale, float fa_jerk_scale, float fa_rate_scale,
                           float dt_ctrl, float safe_dt, bool rate_shaper_diag_log);

  float prevPitchRateSlew() const { return state_.pitch_rate_slew; }

 private:
  RateShaperState state_{};
};

}  // namespace circle::strike
