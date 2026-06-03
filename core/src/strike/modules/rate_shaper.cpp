#include "circle/strike/modules/rate_shaper.hpp"

#include <algorithm>
#include <cmath>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

void RateShaper::reset() {
  state_ = RateShaperState{};
}

RateShaperOutput RateShaper::compute(
    float roll_cmd, float pitch_cmd, float yaw_target,
    float roll_attitude, float pitch_attitude,
    float max_roll_angle_rad, float max_pitch_angle_rad,
    float softcap_band_rad,
    float rate_lpf_tau_s, float max_jerk_rad_s2,
    float max_roll_rate_rad_s, float max_pitch_rate_rad_s,
    float yaw_rate_lpf_tau_s, bool yaw_lock_enabled,
    float rho_scale, float fa_jerk_scale, float fa_rate_scale,
    float dt_ctrl, float safe_dt, bool rate_shaper_diag_log) {
  RateShaperOutput out;
  out.roll_softcap_factor = tiltSoftcapFactor(
      roll_attitude, roll_cmd, max_roll_angle_rad, softcap_band_rad);
  out.pitch_softcap_factor = tiltSoftcapFactor(
      pitch_attitude, pitch_cmd, max_pitch_angle_rad, softcap_band_rad);
  roll_cmd *= out.roll_softcap_factor;
  pitch_cmd *= out.pitch_softcap_factor;

  // FA-scaled shaping
  const float max_jerk_eff = max_jerk_rad_s2 * fa_jerk_scale;
  const float max_roll_rate_eff = max_roll_rate_rad_s * rho_scale * fa_rate_scale;
  const float max_pitch_rate_eff = max_pitch_rate_rad_s * rho_scale * fa_rate_scale;

  // Unified rate axis shaping
  RateAxisShaperDiag roll_diag{};
  RateAxisShaperDiag pitch_diag{};
  out.roll_rate_rad_s = shapeRateAxis(
      roll_cmd, state_.roll_rate_filt, state_.roll_rate_slew,
      rate_lpf_tau_s, max_jerk_eff, max_roll_rate_eff, dt_ctrl,
      rate_shaper_diag_log ? &roll_diag : nullptr);
  out.pitch_rate_rad_s = shapeRateAxis(
      pitch_cmd, state_.pitch_rate_filt, state_.pitch_rate_slew,
      rate_lpf_tau_s, max_jerk_eff, max_pitch_rate_eff, dt_ctrl,
      rate_shaper_diag_log ? &pitch_diag : nullptr);

  // Yaw
  if (yaw_lock_enabled) {
    state_.yaw_rate_filt = 0.0F;
    out.yaw_rate_rad_s = 0.0F;
  } else {
    const float yaw_alpha = lpfAlpha(yaw_rate_lpf_tau_s, safe_dt);
    state_.yaw_rate_filt += yaw_alpha * (yaw_target - state_.yaw_rate_filt);
    out.yaw_rate_rad_s = state_.yaw_rate_filt;
  }

  return out;
}

}  // namespace circle::strike
