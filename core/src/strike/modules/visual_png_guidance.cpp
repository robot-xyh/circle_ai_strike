#include "circle/strike/modules/visual_png_guidance.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike {

namespace {

constexpr float kNsToS = 1.0e-9F;

float lpfAlpha(float tau_s, float dt_s) {
  if (tau_s <= 0.0F) {
    return 1.0F;
  }
  return std::clamp(dt_s / (tau_s + dt_s), 0.0F, 1.0F);
}

float clampFinite(float value, float min_value, float max_value) {
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  return std::clamp(value, min_value, max_value);
}

}  // namespace

float VisualPngGuidance::derotateExDot(float ex_dot_raw,
                                       float pitch_rate_rad_s,
                                       float gain) {
  return ex_dot_raw + pitch_rate_rad_s * gain;
}

float VisualPngGuidance::derotateEyDot(float ey_dot_raw,
                                       float roll_rate_rad_s,
                                       float gain) {
  return ey_dot_raw - roll_rate_rad_s * gain;
}

VisualPngGuidanceOutput VisualPngGuidance::compute(
    const VisualPngGuidanceParams& params,
    const VisualPngGuidanceInput& input) const {
  VisualPngGuidanceOutput out;
  if (!params.enable || !input.active) {
    return out;
  }

  float ex_dot_inertial = input.ex_dot;
  float ey_dot_inertial = input.ey_dot;
  if (params.derotate_body_rates) {
    ex_dot_inertial = derotateExDot(
        input.ex_dot, input.pitch_rate_rad_s, params.derotate_pitch_to_x_gain);
    ey_dot_inertial = derotateEyDot(
        input.ey_dot, input.roll_rate_rad_s, params.derotate_roll_to_y_gain);
  }

  const float residual_limit = std::max(0.0F, params.residual_rate_limit_rad_s);
  if (residual_limit > 0.0F) {
    ex_dot_inertial = clampFinite(ex_dot_inertial, -residual_limit, residual_limit);
    ey_dot_inertial = clampFinite(ey_dot_inertial, -residual_limit, residual_limit);
  }

  const float closing = std::max(0.0F, -input.e_rho_dot);
  const float area_term = std::sqrt(std::max(0.0F, input.bbox_area_ratio));
  out.closure_scale = std::clamp(
      params.closure_base_scale +
          params.closure_rho_dot_gain * closing +
          params.closure_area_gain * area_term,
      params.closure_min_scale,
      params.closure_max_scale);

  out.ex_dot_inertial = ex_dot_inertial;
  out.ey_dot_inertial = ey_dot_inertial;
  out.roll_png_ff_rad_s =
      input.lateral_output_sign * params.nav_ratio_x *
      out.closure_scale * ex_dot_inertial;
  out.pitch_png_ff_rad_s =
      input.longitudinal_output_sign * params.nav_ratio_y *
      out.closure_scale * ey_dot_inertial;

  const float ff_limit = std::max(0.0F, params.max_feedforward_rad_s);
  if (ff_limit > 0.0F) {
    out.roll_png_ff_rad_s =
        std::clamp(out.roll_png_ff_rad_s, -ff_limit, ff_limit);
    out.pitch_png_ff_rad_s =
        std::clamp(out.pitch_png_ff_rad_s, -ff_limit, ff_limit);
  }

  out.roll_trim_rad_s =
      input.lateral_output_sign *
      (params.fov_trim_kp_rate * input.ex +
       params.fov_trim_kd_rate * ex_dot_inertial);
  out.pitch_trim_rad_s =
      input.longitudinal_output_sign *
      (params.fov_trim_kp_rate * input.ey +
       params.fov_trim_kd_rate * ey_dot_inertial);

  const float blend = std::clamp(params.blend, 0.0F, 1.0F);
  out.roll_rate_rad_s = std::clamp(
      (out.roll_png_ff_rad_s + out.roll_trim_rad_s) * blend,
      -std::max(0.0F, input.max_roll_rate_rad_s),
      std::max(0.0F, input.max_roll_rate_rad_s));
  out.pitch_rate_rad_s = std::clamp(
      (out.pitch_png_ff_rad_s + out.pitch_trim_rad_s) * blend,
      -std::max(0.0F, input.max_pitch_rate_rad_s),
      std::max(0.0F, input.max_pitch_rate_rad_s));
  out.active = true;
  return out;
}

void RhoRateWindowEstimator::reset() {
  start_ = 0;
  count_ = 0;
  filtered_valid_ = false;
  filtered_rate_ = 0.0F;
}

float RhoRateWindowEstimator::addSample(const RhoRateWindowParams& params,
                                        float e_rho,
                                        uint64_t stamp_ns) {
  if (!params.enable || !std::isfinite(e_rho) || stamp_ns == 0) {
    reset();
    return 0.0F;
  }

  const std::size_t capacity = static_cast<std::size_t>(
      std::clamp(params.window_samples, 2, static_cast<int>(samples_.size())));
  if (count_ > 0) {
    const std::size_t last_index = (start_ + count_ - 1U) % samples_.size();
    if (stamp_ns <= samples_[last_index].stamp_ns) {
      reset();
    }
  }

  if (count_ < capacity) {
    samples_[(start_ + count_) % samples_.size()] = Sample{e_rho, stamp_ns};
    ++count_;
  } else {
    samples_[start_] = Sample{e_rho, stamp_ns};
    start_ = (start_ + 1U) % samples_.size();
  }

  if (count_ < 2U) {
    return filtered_rate_;
  }

  const uint64_t t_ref_ns = samples_[start_].stamp_ns;
  float mean_t = 0.0F;
  float mean_y = 0.0F;
  for (std::size_t i = 0; i < count_; ++i) {
    const auto& s = samples_[(start_ + i) % samples_.size()];
    mean_t += static_cast<float>(s.stamp_ns - t_ref_ns) * kNsToS;
    mean_y += s.e_rho;
  }
  mean_t /= static_cast<float>(count_);
  mean_y /= static_cast<float>(count_);

  float denom = 0.0F;
  float numer = 0.0F;
  for (std::size_t i = 0; i < count_; ++i) {
    const auto& s = samples_[(start_ + i) % samples_.size()];
    const float t = static_cast<float>(s.stamp_ns - t_ref_ns) * kNsToS;
    const float dt = t - mean_t;
    numer += dt * (s.e_rho - mean_y);
    denom += dt * dt;
  }

  if (denom <= 1.0e-9F) {
    return filtered_rate_;
  }

  const float raw_rate = numer / denom;
  if (!filtered_valid_) {
    filtered_rate_ = raw_rate;
    filtered_valid_ = true;
    return filtered_rate_;
  }

  const std::size_t last_index = (start_ + count_ - 1U) % samples_.size();
  const std::size_t prev_index = (start_ + count_ - 2U) % samples_.size();
  const float dt_s = static_cast<float>(
      samples_[last_index].stamp_ns - samples_[prev_index].stamp_ns) * kNsToS;
  const float a = lpfAlpha(params.lpf_tau_s, std::max(0.0F, dt_s));
  filtered_rate_ += a * (raw_rate - filtered_rate_);
  return filtered_rate_;
}

}  // namespace circle::strike
