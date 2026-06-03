#pragma once

namespace circle::strike_png {

struct VisualPngGuidanceParams {
  float nav_ratio_x{3.0F};
  float nav_ratio_y{3.0F};
  bool derotate_body_rates{true};
  float derotate_pitch_to_x_gain{1.0F};
  float derotate_roll_to_y_gain{1.0F};
  float residual_rate_limit_rad_s{1.2F};
  float closure_base_scale{0.7F};
  float closure_area_gain{0.35F};
  float max_feedforward_rad_s{0.9F};
  float fov_trim_kp_rate{0.15F};
  float fov_trim_fade_area_ratio_start{1.0F};
  float fov_trim_fade_area_ratio_full{1.0F};
  float vertical_aim_ey{0.0F};
  float terminal_tilt_aim_area_ratio_start{1.0F};
  float terminal_tilt_aim_area_ratio_full{1.0F};
  float terminal_tilt_aim_roll_gain{0.0F};
  float terminal_tilt_aim_pitch_gain{0.0F};
  float terminal_tilt_aim_max_offset_norm{0.0F};
  bool edge_guard_enable{true};
  float edge_guard_start_norm{0.20F};
  float edge_guard_full_norm{0.38F};
  float edge_guard_kp_rate{0.85F};
  float edge_guard_min_rate_rad_s{0.08F};
  float edge_guard_max_rate_rad_s{0.55F};
  float edge_guard_pitch_scale{0.0F};
  bool pursuit_fallback_enable{true};
  float pursuit_fallback_kp_rate{0.85F};
  float pursuit_fallback_start_norm{0.04F};
  float pursuit_fallback_full_norm{0.30F};
  float pursuit_fallback_min_rate_rad_s{0.18F};
  float pursuit_fallback_max_rate_rad_s{0.75F};
  float pursuit_fallback_png_weak_rate_rad_s{0.18F};
  bool terminal_stale_lateral_trim_enable{false};
  float terminal_stale_lateral_trim_area_ratio_start{0.003F};
  float terminal_stale_lateral_trim_area_ratio_full{0.006F};
  float terminal_stale_lateral_trim_stale_s_start{0.08F};
  float terminal_stale_lateral_trim_stale_s_full{0.20F};
  float terminal_stale_lateral_trim_kp_rate{3.0F};
  float terminal_stale_lateral_trim_max_rate_rad_s{0.30F};
  bool terminal_intercept_enable{false};
  float terminal_intercept_area_ratio_start{0.003F};
  float terminal_intercept_area_ratio_full{0.008F};
  float terminal_intercept_lead_s{0.16F};
  float terminal_intercept_kp_rate{1.8F};
  float terminal_intercept_max_rate_rad_s{0.25F};
  bool terminal_crossing_enable{false};
  float terminal_crossing_area_ratio_start{0.003F};
  float terminal_crossing_area_ratio_full{0.008F};
  float terminal_crossing_rate_start_norm_s{0.05F};
  float terminal_crossing_rate_full_norm_s{0.30F};
  float terminal_crossing_kd_rate{0.80F};
  float terminal_crossing_max_rate_rad_s{0.25F};
  bool terminal_forward_speed_guard_enable{false};
  float terminal_forward_speed_guard_area_ratio_start{0.003F};
  float terminal_forward_speed_guard_area_ratio_full{0.008F};
  float terminal_forward_speed_guard_start_m_s{34.0F};
  float terminal_forward_speed_guard_full_m_s{40.0F};
  float terminal_forward_speed_guard_min_positive_pitch_scale{0.35F};
  // Output-axis sign. Internal math is the PX4 baseline (lateral=+1 roll=+ex,
  // longitudinal=-1 pitch=-ey); these mirror the final axis for other frames
  // (e.g. BF top-cam needs longitudinal=+1). Only ±1 are meaningful.
  float lateral_output_sign{1.0F};
  float longitudinal_output_sign{-1.0F};
};

struct VisualPngGuidanceInput {
  float ex{0.0F};
  float ey{0.0F};
  float ex_dot{0.0F};
  float ey_dot{0.0F};
  float bbox_area_ratio{0.0F};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  bool attitude_valid{false};
  float roll_rad{0.0F};
  float pitch_rad{0.0F};
  float max_roll_rate_rad_s{1.2F};
  float max_pitch_rate_rad_s{1.2F};
  float measurement_age_s{0.0F};
  bool ownship_forward_speed_valid{false};
  float ownship_forward_speed_m_s{0.0F};
};

struct VisualPngGuidanceOutput {
  bool active{false};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float closure_scale{1.0F};
  float ex_dot_inertial{0.0F};
  float ey_dot_inertial{0.0F};
  float roll_png_ff_rad_s{0.0F};
  float pitch_png_ff_rad_s{0.0F};
  float roll_fov_trim_rad_s{0.0F};
  float pitch_fov_trim_rad_s{0.0F};
  float roll_edge_guard_rad_s{0.0F};
  float pitch_edge_guard_rad_s{0.0F};
  float roll_pursuit_fallback_rad_s{0.0F};
  float pitch_pursuit_fallback_rad_s{0.0F};
  float roll_terminal_stale_trim_rad_s{0.0F};
  bool terminal_intercept_active{false};
  float terminal_intercept_lead_s{0.0F};
  float terminal_future_ex{0.0F};
  float terminal_future_ey{0.0F};
  float roll_terminal_intercept_rad_s{0.0F};
  float pitch_terminal_intercept_rad_s{0.0F};
  float terminal_aim_ex{0.0F};
  float terminal_aim_ey{0.0F};
  bool terminal_crossing_active{false};
  float terminal_crossing_weight{0.0F};
  float roll_terminal_crossing_rad_s{0.0F};
  float pitch_terminal_crossing_rad_s{0.0F};
  bool terminal_forward_speed_guard_active{false};
  float terminal_forward_speed_guard_weight{0.0F};
  float terminal_forward_speed_guard_scale{1.0F};
};

class VisualPngGuidance {
 public:
  [[nodiscard]] VisualPngGuidanceOutput compute(
      const VisualPngGuidanceParams& params,
      const VisualPngGuidanceInput& input) const;
};

}  // namespace circle::strike_png
