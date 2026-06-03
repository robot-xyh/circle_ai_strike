#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace circle::strike {

struct VisualPngGuidanceParams {
  bool enable{false};
  float nav_ratio_x{3.0F};
  float nav_ratio_y{3.0F};
  float fov_trim_kp_rate{0.35F};
  float fov_trim_kd_rate{0.05F};
  bool derotate_body_rates{true};
  float derotate_pitch_to_x_gain{1.0F};
  float derotate_roll_to_y_gain{1.0F};
  float residual_rate_limit_rad_s{2.5F};
  float closure_base_scale{1.0F};
  float closure_rho_dot_gain{0.35F};
  float closure_area_gain{0.80F};
  float closure_min_scale{0.45F};
  float closure_max_scale{2.6F};
  float max_feedforward_rad_s{1.8F};
  float blend{1.0F};
};

struct VisualPngGuidanceInput {
  bool active{false};
  float ex{0.0F};
  float ey{0.0F};
  float ex_dot{0.0F};
  float ey_dot{0.0F};
  float e_rho_dot{0.0F};
  float bbox_area_ratio{0.0F};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float lateral_output_sign{1.0F};
  float longitudinal_output_sign{-1.0F};
  float max_roll_rate_rad_s{2.0F};
  float max_pitch_rate_rad_s{2.0F};
};

struct VisualPngGuidanceOutput {
  bool active{false};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float roll_png_ff_rad_s{0.0F};
  float pitch_png_ff_rad_s{0.0F};
  float roll_trim_rad_s{0.0F};
  float pitch_trim_rad_s{0.0F};
  float closure_scale{1.0F};
  float ex_dot_inertial{0.0F};
  float ey_dot_inertial{0.0F};
};

class VisualPngGuidance {
 public:
  static float derotateExDot(float ex_dot_raw,
                             float pitch_rate_rad_s,
                             float gain);
  static float derotateEyDot(float ey_dot_raw,
                             float roll_rate_rad_s,
                             float gain);

  [[nodiscard]] VisualPngGuidanceOutput compute(
      const VisualPngGuidanceParams& params,
      const VisualPngGuidanceInput& input) const;
};

struct RhoRateWindowParams {
  bool enable{false};
  int window_samples{11};
  float lpf_tau_s{0.05F};
};

class RhoRateWindowEstimator {
 public:
  void reset();
  float addSample(const RhoRateWindowParams& params,
                  float e_rho,
                  uint64_t stamp_ns);

 private:
  struct Sample {
    float e_rho{0.0F};
    uint64_t stamp_ns{0};
  };

  std::array<Sample, 32> samples_{};
  std::size_t start_{0};
  std::size_t count_{0};
  bool filtered_valid_{false};
  float filtered_rate_{0.0F};
};

}  // namespace circle::strike
