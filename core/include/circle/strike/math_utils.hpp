#pragma once

#include <algorithm>
#include <cmath>

namespace circle::strike {

inline float applyDeadband(float value, float deadband) {
  if (std::abs(value) < deadband) {
    return 0.0F;
  }
  return (value > 0.0F) ? (value - deadband) : (value + deadband);
}

inline float wrapAngle(float angle) {
  while (angle > static_cast<float>(M_PI)) {
    angle -= 2.0F * static_cast<float>(M_PI);
  }
  while (angle < -static_cast<float>(M_PI)) {
    angle += 2.0F * static_cast<float>(M_PI);
  }
  return angle;
}

inline float lpfAlpha(float tau_s, float dt_s) {
  if (tau_s <= 0.0F || dt_s <= 0.0F) {
    return 1.0F;
  }
  return std::clamp(dt_s / (tau_s + dt_s), 0.0F, 1.0F);
}

inline float smoothstep01(float t) {
  const float c = std::clamp(t, 0.0F, 1.0F);
  return c * c * (3.0F - 2.0F * c);
}

inline float lerp(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) return y0;
  const float t = std::clamp((x - x0) / (x1 - x0), 0.0F, 1.0F);
  return y0 + t * (y1 - y0);
}

inline float clampThrustScalar(float v, float scalar_min, float scalar_max) {
  return std::clamp(v, scalar_min, scalar_max);
}

inline float scalarToFrdThrustZ(float scalar) {
  return -scalar;
}

inline float tiltSoftcapFactor(float attitude_actual, float rate_desired,
                               float max_angle, float band) {
  const float abs_att = std::fabs(attitude_actual);
  if (abs_att < max_angle - band) {
    return 1.0F;
  }
  if (abs_att >= max_angle) {
    if ((attitude_actual > 0.0F && rate_desired > 0.0F) ||
        (attitude_actual < 0.0F && rate_desired < 0.0F)) {
      return 0.0F;
    }
    return 1.0F;
  }
  const float t = (abs_att - (max_angle - band)) / band;
  const float factor = 1.0F - std::clamp(t, 0.0F, 1.0F);
  if ((attitude_actual > 0.0F && rate_desired > 0.0F) ||
      (attitude_actual < 0.0F && rate_desired < 0.0F)) {
    return factor;
  }
  return 1.0F;
}

struct RateAxisShaperDiag {
  float desired_in{0.0F};
  float lpf_goal{0.0F};
  float rate_pre_jerk{0.0F};
  float rate_post_jerk{0.0F};
  float output_rate_sp{0.0F};
  bool jerk_limited{false};
  bool rate_mag_clamped{false};
};

inline float shapeRateAxis(float desired, float& state_filt,
                           float& state_slew, float lpf_tau_s,
                           float max_jerk, float max_rate, float dt_s,
                           RateAxisShaperDiag* diag_out = nullptr) {
  const float alpha = lpfAlpha(lpf_tau_s, dt_s);
  const float lpf_goal = state_filt + alpha * (desired - state_filt);
  state_filt = lpf_goal;

  float rate = state_filt;
  if (max_jerk > 0.0F) {
    const float max_delta = max_jerk * dt_s;
    const float delta = rate - state_slew;
    const float clamped_delta = std::clamp(delta, -max_delta, max_delta);
    state_slew += clamped_delta;
    rate = state_slew;
  } else {
    state_slew = rate;
  }

  const float before_clamp = rate;
  rate = std::clamp(rate, -max_rate, max_rate);

  if (diag_out) {
    diag_out->desired_in = desired;
    diag_out->lpf_goal = state_filt;
    diag_out->rate_pre_jerk = before_clamp;
    diag_out->rate_post_jerk = rate;
    diag_out->output_rate_sp = rate;
    diag_out->jerk_limited =
        (max_jerk > 0.0F) &&
        (std::fabs(desired - state_slew) > max_jerk * dt_s + 1.0e-6F);
    diag_out->rate_mag_clamped = std::fabs(before_clamp) > max_rate + 1.0e-6F;
  }

  return rate;
}

}  // namespace circle::strike
