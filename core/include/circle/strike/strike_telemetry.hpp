#pragma once

#include <cstdint>
#include <limits>

namespace circle::strike {

struct LastVisualContactMetric {
  bool valid{false};
  int64_t stamp_ns{0};
  double run_elapsed_s{std::numeric_limits<double>::quiet_NaN()};
  bool vehicle_position_valid{false};
  float vehicle_altitude_m{std::numeric_limits<float>::quiet_NaN()};
  bool vehicle_velocity_valid{false};
  float vehicle_vxy_m_s{std::numeric_limits<float>::quiet_NaN()};
  float bbox_area_ratio{std::numeric_limits<float>::quiet_NaN()};
  float detection_score{std::numeric_limits<float>::quiet_NaN()};
  float align_error_x_px{std::numeric_limits<float>::quiet_NaN()};
  float align_error_y_px{std::numeric_limits<float>::quiet_NaN()};
  bool margin_valid{false};
  float margin_x_px{std::numeric_limits<float>::quiet_NaN()};
  float margin_y_px{std::numeric_limits<float>::quiet_NaN()};
  bool final_approach_active{false};
  bool target_altitude_valid{false};
  float target_altitude_m{std::numeric_limits<float>::quiet_NaN()};
  float altitude_gap_to_target_m{std::numeric_limits<float>::quiet_NaN()};
  bool altitude_goal_met{false};
};

struct StrikeTelemetry {
  bool valid{false};
  bool coasting{false};
  bool final_approach_active{false};
  bool final_approach_commit_active{false};
  bool tracker_fallback_active{false};
  bool strike_confident{false};
  bool preclimb_xy_gate_active{false};
  bool preclimb_xy_gate_released{false};
  bool preclimb_release_slowdown_active{false};
  bool tracking_speed_governor_active{false};
  bool edge_protect_active{false};
  bool bottom_pitch_guard_active{false};
  bool image_lead_active{false};
  bool terminal_predictor_active{false};
  bool ascent_image_velocity_damping_active{false};
  bool tracking_deadband_priority_active{false};
  bool fa_vertical_drift_slowdown_active{false};
  bool fa_ascent_budget_active{false};
  bool png_active{false};

  float roll_rate_sp_rad_s{0.0F};
  float pitch_rate_sp_rad_s{0.0F};
  float yaw_rate_sp_rad_s{0.0F};
  float thrust_z{0.0F};

  float ex{0.0F};
  float ey{0.0F};
  float ex_raw{0.0F};
  float ey_raw{0.0F};
  float ex_dot_filt{0.0F};
  float ey_dot_filt{0.0F};
  float e_rho{0.0F};
  float e_rho_dot_filt{0.0F};
  float rho_scale{1.0F};

  float aim_comp_x_px{0.0F};
  float aim_comp_y_px{0.0F};
  float image_lead_x_px{0.0F};
  float image_lead_y_px{0.0F};
  float terminal_predictor_lead_x_px{0.0F};
  float terminal_predictor_lead_y_px{0.0F};
  float terminal_predictor_predicted_ex{0.0F};
  float terminal_predictor_predicted_ey{0.0F};
  float terminal_predictor_closing_rate{0.0F};
  float png_roll_ff_rad_s{0.0F};
  float png_pitch_ff_rad_s{0.0F};
  float png_roll_trim_rad_s{0.0F};
  float png_pitch_trim_rad_s{0.0F};
  float png_closure_scale{1.0F};
  float png_ex_dot_inertial{0.0F};
  float png_ey_dot_inertial{0.0F};

  float constant_thrust{0.0F};
  float tracking_thrust_scalar_smooth{std::numeric_limits<float>::quiet_NaN()};
  float tracking_thrust_scalar_target{std::numeric_limits<float>::quiet_NaN()};
  float actual_thrust_z{std::numeric_limits<float>::quiet_NaN()};

  float roll_softcap_factor{1.0F};
  float pitch_softcap_factor{1.0F};
  float roll_hard_headroom_rad{0.0F};
  float pitch_hard_headroom_rad{0.0F};

  float deadband_eff_half_w_px{0.0F};
  float deadband_eff_half_h_px{0.0F};

  float waiting_altitude_ref_ned{std::numeric_limits<float>::quiet_NaN()};
  float waiting_altitude_error_ned{std::numeric_limits<float>::quiet_NaN()};
  float waiting_altitude_correction{std::numeric_limits<float>::quiet_NaN()};

  float detection_score{0.0F};
  float detection_age_s{0.0F};
  float bbox_area_ratio{0.0F};
  float bbox_margin_x_px{0.0F};
  float bbox_margin_y_px{0.0F};

  float fa_kp_scale{1.0F};
  float fa_kd_scale{1.0F};
  float fa_prox_norm{1.0F};
  float fa_thrust_taper_scale{1.0F};
  float fa_unaligned_slowdown_scale{1.0F};
  float fa_tilt_slowdown_scale{1.0F};
  float fa_vertical_drift_slowdown_scale{1.0F};
  float fa_ascent_budget_tilt_score{0.0F};
  float fa_ascent_budget_y_rate_score{0.0F};
  float fa_ascent_budget_y_error_score{0.0F};
  float fa_roll_level_blend{0.0F};
  float fa_pitch_level_blend{0.0F};
  float edge_taper_score{0.0F};
  float bottom_pitch_guard_blend{0.0F};

  float speed_governor_blend{0.0F};
  float speed_governor_scale{1.0F};

  float preclimb_safe_hold_blend{0.0F};
  float preclimb_xy_thrust_scale{1.0F};
  float preclimb_release_slowdown_scale{1.0F};

  float tracking_deadband_priority_scale{1.0F};

  int state{0};
  int guidance_mode{0};
  
  LastVisualContactMetric last_visual_contact{};
};

}  // namespace circle::strike
