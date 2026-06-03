#include "circle/strike/modules/tilt_envelope.hpp"

#include <algorithm>
#include <cmath>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

namespace {

float applyAxis(float cmd, float att, float max_angle, float band, float margin,
                float kp, float max_level_rate, float lpf_tau_s, float max_jerk,
                float dt_s, float& filt, float& slew, float& softcap_out,
                float& weight_out) {
  // Stage 1: soft cap (continuous over [max_angle - band, max_angle]).
  const float factor = tiltSoftcapFactor(att, cmd, max_angle, band);
  const float soft = cmd * factor;
  softcap_out = factor;

  // Stage 2: hard cap via smoothstep blend toward the leveling rate.
  const float level = std::clamp(-kp * att, -max_level_rate, max_level_rate);
  const float w = (margin > 1.0e-4F)
                      ? smoothstep01((std::fabs(att) - max_angle) / margin)
                      : (std::fabs(att) >= max_angle ? 1.0F : 0.0F);
  weight_out = w;
  const float blended = soft * (1.0F - w) + level * w;

  // Stage 3: optional output smoothing (LPF + jerk). Pass a large rate cap so
  // shapeRateAxis only smooths; magnitude limiting is handled upstream.
  return shapeRateAxis(blended, filt, slew, lpf_tau_s, max_jerk, 1.0e3F, dt_s);
}

}  // namespace

void TiltEnvelope::reset() {
  state_ = TiltEnvelopeState{};
}

TiltEnvelopeOutput TiltEnvelope::compute(const TiltEnvelopeParams& params,
                                         float roll_cmd, float pitch_cmd,
                                         float roll_att, float pitch_att,
                                         float dt_s) {
  TiltEnvelopeOutput out;
  out.roll_rate_rad_s = roll_cmd;
  out.pitch_rate_rad_s = pitch_cmd;
  if (!params.enable) {
    state_.roll_filt = roll_cmd;
    state_.roll_slew = roll_cmd;
    state_.pitch_filt = pitch_cmd;
    state_.pitch_slew = pitch_cmd;
    return out;
  }

  const float safe_dt = std::clamp(dt_s, 0.0F, 0.1F);
  out.roll_rate_rad_s = applyAxis(
      roll_cmd, roll_att, params.max_roll_angle_rad, params.softcap_band_rad,
      params.hardcap_margin_rad, params.hardcap_level_kp,
      params.max_level_rate_rad_s, params.out_lpf_tau_s,
      params.out_max_jerk_rad_s2, safe_dt, state_.roll_filt, state_.roll_slew,
      out.roll_softcap_factor, out.roll_level_weight);
  out.pitch_rate_rad_s = applyAxis(
      pitch_cmd, pitch_att, params.max_pitch_angle_rad, params.softcap_band_rad,
      params.hardcap_margin_rad, params.hardcap_level_kp,
      params.max_level_rate_rad_s, params.out_lpf_tau_s,
      params.out_max_jerk_rad_s2, safe_dt, state_.pitch_filt, state_.pitch_slew,
      out.pitch_softcap_factor, out.pitch_level_weight);
  out.hardcap_active =
      std::max(out.roll_level_weight, out.pitch_level_weight) > 0.5F;
  return out;
}

}  // namespace circle::strike
