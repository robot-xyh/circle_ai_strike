#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace circle::debug {

struct VisionPlotSample {
  double t_sec{0.0};
  float ex{0.F};
  float ey{0.F};
  float ex_raw{0.F};
  float ey_raw{0.F};
  float e_rho{0.F};
  float yaw_rate_sp{0.F};
  float fwd{0.F};
  float right{0.F};
  float down{0.F};
  float roll_sp{0.F};
  float pitch_sp{0.F};
  float thrust_z{0.F};
  float vehicle_roll_rad{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_pitch_rad{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_yaw_rad{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_roll_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_pitch_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_yaw_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  uint8_t track_phase{0};
  bool coasting{false};
  bool area_thrust_valid{false};
  float tracking_thrust{0.F};
  float thrust_target{0.F};
  float thrust_floor{0.F};
  float rho_dot{0.F};
  float filtered_rho_dot{0.F};
  float filtered_ey_thrust{0.F};
  float high_thrust_duration_s{0.F};
  bool alt_valid{false};
  float alt_setpoint{0.F};
  float alt_pos_err{0.F};
  float alt_vz_cmd{0.F};
  float alt_vz_err{0.F};
  float alt_integ{0.F};
  float alt_d{0.F};
  float tilt_hard_headroom_roll_rad{0.F};
  float tilt_hard_headroom_pitch_rad{0.F};
  // --- target_strike_png guidance component decomposition (png_debug_valid) ---
  bool png_valid{false};
  float png_ff_roll_rad_s{0.F};
  float png_ff_pitch_rad_s{0.F};
  float png_fov_trim_roll_rad_s{0.F};
  float png_fov_trim_pitch_rad_s{0.F};
  float png_edge_guard_roll_rad_s{0.F};
  float png_edge_guard_pitch_rad_s{0.F};
  float png_pursuit_roll_rad_s{0.F};
  float png_pursuit_pitch_rad_s{0.F};
  float png_stale_trim_roll_rad_s{0.F};
  float png_intercept_roll_rad_s{0.F};
  float png_intercept_pitch_rad_s{0.F};
  float png_crossing_pitch_rad_s{0.F};
  float png_closure_scale{1.F};
  float png_ex_dot_inertial{0.F};
  float png_ey_dot_inertial{0.F};
  float png_future_ex{0.F};
  float png_future_ey{0.F};
  float png_intercept_lead_s{0.F};
  float png_crossing_weight{0.F};
  float png_fwd_guard_scale{1.F};
  float png_entry_handoff_progress{1.F};
  float png_measurement_age_s{0.F};
  float png_tilt_softcap_roll{1.F};
  float png_tilt_softcap_pitch{1.F};
  bool png_intercept_active{false};
  bool png_crossing_active{false};
  bool png_fwd_guard_active{false};
  bool png_loss_hold_latched{false};
  bool png_tilt_hardcap_active{false};
  float detection_score{0.F};
  bool detection_valid{false};
};

std::string makeVisionSeriesJson(const std::string& query,
                                 const std::vector<VisionPlotSample>& samples,
                                 const std::string& tune_target_node,
                                 int default_max_points,
                                 double default_window_s,
                                 std::optional<float> live_px4_thrust_z);

struct TunableParamDesc {
  const char* name;
  double min;
  double max;
  double step;
  bool is_bool;
};

// Whitelist for HTTP tuning (vision_tracking.* on vision executor).
// Boolean / mode switches are listed first so the tune page shows them at the top.
constexpr TunableParamDesc kVisionTunableParams[] = {
    {"vision_tracking.dry_run", 0.0, 1.0, 1.0, true},
    {"vision_tracking.hold_altitude_enable", 0.0, 1.0, 1.0, true},
    {"vision_tracking.debug_hold_only", 0.0, 1.0, 1.0, true},
    {"vision_tracking.debug_detect_only", 0.0, 1.0, 1.0, true},
    {"vision_tracking.lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"vision_tracking.longitudinal_output_sign", -1.0, 1.0, 1.0, false},
    {"vision_tracking.max_roll_angle_rad", 0.05, 1.2, 0.01, false},
    {"vision_tracking.max_pitch_angle_rad", 0.05, 1.2, 0.01, false},
    {"vision_tracking.x_deadband", 0.0, 0.5, 0.002, false},
    {"vision_tracking.y_deadband", 0.0, 0.5, 0.002, false},
    {"vision_tracking.x_deadband_px", 0.0, 200.0, 1.0, false},
    {"vision_tracking.y_deadband_px", 0.0, 200.0, 1.0, false},
    {"vision_tracking.rho_deadband", 0.0, 0.5, 0.002, false},
    {"vision_tracking.attitude_sp_lpf_tau_s", 0.0, 1.0, 0.01, false},
    {"vision_tracking.attitude_sp_max_rate_rad_s", 0.0, 10.0, 0.1, false},
    {"vision_tracking.attitude_sp_pitch_max_rate_up_rad_s", 0.0, 10.0, 0.05, false},
    {"vision_tracking.attitude_sp_pitch_max_rate_down_rad_s", 0.0, 10.0, 0.05, false},
    {"vision_tracking.pitch_setpoint_max_rad", -1.3, 1.3, 0.02, false},
    {"vision_tracking.pitch_setpoint_min_rad", -1.3, 1.3, 0.02, false},
    {"vision_tracking.pitch_feedforward_bias_rad", -0.5, 0.5, 0.01, false},
    {"vision_tracking.pitch_fixed_rad", -1.2, 1.2, 0.02, false},
    {"vision_tracking.pitch_fixed_blend", 0.0, 1.0, 0.02, false},
    {"vision_tracking.pitch_soft_limits_apply_in_hold", 0.0, 1.0, 1.0, true},
    {"vision_tracking.hold_thrust_blend_rate", 0.5, 40.0, 0.5, false},
    {"vision_tracking.hold_thrust_blend_alpha_cap", 0.05, 1.0, 0.02, false},
    {"vision_tracking.hold_altitude_kp", 0.0, 1.0, 0.01, false},
    {"vision_tracking.hold_altitude_ki", 0.0, 0.5, 0.005, false},
    {"vision_tracking.hold_altitude_kd", 0.0, 1.0, 0.01, false},
    {"vision_tracking.hold_altitude_integral_max", 0.0, 20.0, 0.1, false},
    {"vision_tracking.hold_altitude_delta_max", 0.0, 0.5, 0.01, false},
    {"vision_tracking.hold_altitude_max_vz", 0.0, 10.0, 0.1, false},
    {"vision_tracking.lateral_outer_kp", 0.0, 20.0, 0.1, false},
    {"vision_tracking.lateral_inner_kp", 0.0, 5.0, 0.01, false},
    {"vision_tracking.lateral_vel_max", 0.1, 10.0, 0.1, false},
    {"vision_tracking.longitudinal_outer_kp", 0.0, 20.0, 0.1, false},
    {"vision_tracking.longitudinal_inner_kp", 0.0, 5.0, 0.01, false},
    {"vision_tracking.longitudinal_vel_max", 0.1, 10.0, 0.1, false},
    {"vision_tracking.yaw_bearing_kp", 0.0, 10.0, 0.02, false},
    {"vision_tracking.yaw_hold_gain", 0.0, 5.0, 0.02, false},
    {"vision_tracking.yaw_assist_ex_gain", -5.0, 5.0, 0.02, false},
    {"vision_tracking.yaw_assist_ey_gain", -5.0, 5.0, 0.02, false},
    {"vision_tracking.yaw_image_flow_gain", -5.0, 5.0, 0.02, false},
    {"vision_tracking.yaw_image_flow_deadband", 0.0, 1.0, 0.005, false},
    {"vision_tracking.range_kp", 0.0, 10.0, 0.02, false},
    {"vision_tracking.range_kd", 0.0, 10.0, 0.02, false},
    {"vision_tracking.max_forward_speed_mps", 0.0, 30.0, 0.1, false},
    {"vision_tracking.max_yaw_rate_rad_s", 0.0, 6.0, 0.02, false},
    {"vision_tracking.forward_to_pitch_gain", 0.0, 2.0, 0.02, false},
    {"vision_tracking.area_thrust_gain", 0.0, 2.0, 0.01, false},
    {"vision_tracking.area_thrust_max", 0.3, 1.0, 0.01, false},
    {"vision_tracking.rho_dot_thrust_deadband", 0.0, 1.0, 0.01, false},
    {"vision_tracking.rho_dot_filter_alpha", 0.01, 1.0, 0.01, false},
    {"vision_tracking.tracking_thrust_decay_rate", 0.0, 0.5, 0.005, false},
    {"vision_tracking.max_rho_dot_for_thrust", 0.1, 10.0, 0.1, false},
    {"vision_tracking.vertical_thrust_boost_gain", 0.0, 1.0, 0.01, false},
    {"vision_tracking.area_thrust_pitch_gain", 0.0, 1.0, 0.01, false},
    {"vision_tracking.coast_gain_scale", 0.0, 1.0, 0.01, false},
    {"vision_tracking.lock_ramp_s", 0.0, 5.0, 0.05, false},
    {"vision_tracking.desired_bbox_area_px", 10.0, 100000.0, 10.0, false},
    {"vision_tracking.min_score", 0.0, 1.0, 0.01, false},
    {"vision_tracking.detection_stale_s", 0.1, 5.0, 0.05, false},
    {"vision_tracking.lost_timeout_s", 0.5, 30.0, 0.1, false},
};

constexpr TunableParamDesc kVerticalTunableParams[] = {
    {"vertical_tracking.dry_run", 0.0, 1.0, 1.0, true},
    {"vertical_tracking.enable_tilt_compensation", 0.0, 1.0, 1.0, true},
    {"vertical_tracking.min_score", 0.0, 1.0, 0.01, false},
    {"vertical_tracking.detection_stale_s", 0.05, 2.0, 0.05, false},
    {"vertical_tracking.lost_timeout_s", 0.2, 5.0, 0.1, false},
    {"vertical_tracking.lateral_kp", 0.0, 12.0, 0.1, false},
    {"vertical_tracking.lateral_kd", 0.0, 3.0, 0.02, false},
    {"vertical_tracking.longitudinal_kp", 0.0, 12.0, 0.1, false},
    {"vertical_tracking.longitudinal_kd", 0.0, 3.0, 0.02, false},
    {"vertical_tracking.pixel_dot_lpf_tau_s", 0.0, 0.5, 0.005, false},
    {"vertical_tracking.lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"vertical_tracking.longitudinal_output_sign", -1.0, 1.0, 1.0, false},
    {"vertical_tracking.max_roll_angle_rad", 0.05, 1.4, 0.01, false},
    {"vertical_tracking.max_pitch_angle_rad", 0.05, 1.4, 0.01, false},
    {"vertical_tracking.x_deadband", 0.0, 0.20, 0.002, false},
    {"vertical_tracking.y_deadband", 0.0, 0.20, 0.002, false},
    {"vertical_tracking.x_deadband_px", 0.0, 200.0, 1.0, false},
    {"vertical_tracking.y_deadband_px", 0.0, 200.0, 1.0, false},
    {"vertical_tracking.attitude_sp_lpf_tau_s", 0.0, 0.30, 0.005, false},
    {"vertical_tracking.attitude_sp_max_rate_rad_s", 0.0, 12.0, 0.1, false},
    {"vertical_tracking.attitude_sp_max_accel_rad_s2", 0.0, 30.0, 0.2, false},
    {"vertical_tracking.desired_bbox_area_px", 0.0, 50000.0, 50.0, false},
    {"vertical_tracking.rho_scale_near", 0.05, 2.0, 0.05, false},
    {"vertical_tracking.rho_scale_far", 0.05, 1.5, 0.05, false},
    {"vertical_tracking.rho_scale_near_thresh", -2.0, 0.0, 0.05, false},
    {"vertical_tracking.rho_scale_far_thresh", 0.0, 2.0, 0.05, false},
    {"vertical_tracking.yaw_bearing_kp", 0.0, 5.0, 0.05, false},
    {"vertical_tracking.yaw_hold_gain", 0.0, 3.0, 0.02, false},
    {"vertical_tracking.yaw_rate_min_rad_s", -6.28, 6.28, 0.05, false},
    {"vertical_tracking.yaw_rate_max_rad_s", -6.28, 6.28, 0.05, false},
    {"vertical_tracking.yaw_hold_deadband_rad", 0.0, 0.1, 0.001, false},
    {"vertical_tracking.yaw_track_deadband_rad", 0.0, 0.3, 0.002, false},
    {"vertical_tracking.yaw_rate_lpf_tau_s", 0.0, 1.0, 0.01, false},
    {"vertical_tracking.lost_target_attitude_decay_tau_s", 0.0, 1.5, 0.01, false},
    {"vertical_tracking.lost_target_attitude_safety_s", 0.0, 3.0, 0.05, false},
    {"vertical_tracking.attitude_ff_max_age_s", 0.0, 1.0, 0.01, false},
    {"vertical_tracking.constant_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"vertical_tracking.waiting_altitude_kp", 0.0, 1.0, 0.01, false},
    {"vertical_tracking.waiting_altitude_kd", 0.0, 1.0, 0.01, false},
    {"vertical_tracking.waiting_altitude_max_correction", 0.0, 0.5, 0.005, false},
    {"vertical_tracking.tilt_cos_floor", 0.20, 0.99, 0.01, false},
    {"vertical_tracking.attitude_thrust_z_min", -1.0, -0.05, 0.005, false},
    {"vertical_tracking.attitude_thrust_z_max", -0.01, 0.0, 0.005, false},
};

// Whitelist for HTTP tuning of target_strike_executor (image-PD → body rates).
// Mirrors gz_target_strike_top_cam.yaml / target_strike_top_cam_real.yaml.
constexpr TunableParamDesc kTargetStrikeTunableParams[] = {
    {"target_strike.dry_run", 0.0, 1.0, 1.0, true},
    {"target_strike.require_armed_to_command", 0.0, 1.0, 1.0, true},
    {"target_strike.rate_shaper_diag_log", 0.0, 1.0, 1.0, true},
    {"target_strike.enable_tilt_compensation", 0.0, 1.0, 1.0, true},
    {"target_strike.hover_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"target_strike.tracking_thrust_ramp_tau_s", 0.0, 2.0, 0.01, false},
    {"target_strike.min_score", 0.0, 1.0, 0.01, false},
    {"target_strike.detection_stale_s", 0.05, 2.0, 0.05, false},
    {"target_strike.lost_timeout_s", 0.2, 10.0, 0.1, false},
    {"target_strike.complete_on_target_loss_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.target_loss_complete_s", 0.1, 5.0, 0.05, false},
    {"target_strike.lateral_kp_rate", 0.0, 12.0, 0.1, false},
    {"target_strike.lateral_kd_rate", 0.0, 4.0, 0.02, false},
    {"target_strike.longitudinal_kp_rate", 0.0, 12.0, 0.1, false},
    {"target_strike.longitudinal_kd_rate", 0.0, 4.0, 0.02, false},
    {"target_strike.pixel_dot_lpf_tau_s", 0.0, 0.6, 0.005, false},
    {"target_strike.lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"target_strike.longitudinal_output_sign", -1.0, 1.0, 1.0, false},
    {"target_strike.x_deadband", 0.0, 0.20, 0.002, false},
    {"target_strike.y_deadband", 0.0, 0.20, 0.002, false},
    {"target_strike.x_deadband_px", 0.0, 200.0, 1.0, false},
    {"target_strike.y_deadband_px", 0.0, 200.0, 1.0, false},
    {"target_strike.rate_lpf_tau_s", 0.0, 0.30, 0.005, false},
    {"target_strike.max_jerk_rad_s2", 0.0, 200.0, 1.0, false},
    {"target_strike.max_roll_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"target_strike.max_pitch_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"target_strike.max_roll_angle_rad", 0.05, 1.5, 0.01, false},
    {"target_strike.max_pitch_angle_rad", 0.05, 1.5, 0.01, false},
    {"target_strike.tilt_softcap_band_rad", 0.01, 0.6, 0.005, false},
    {"target_strike.tilt_hardcap_margin_rad", 0.0, 0.6, 0.005, false},
    {"target_strike.hard_level_kp", 0.0, 15.0, 0.1, false},
    {"target_strike.level_kp", 0.0, 10.0, 0.05, false},
    {"target_strike.level_deadband_rad", 0.0, 0.1, 0.001, false},
    {"target_strike.lost_target_rate_decay_tau_s", 0.0, 1.5, 0.01, false},
    {"target_strike.desired_bbox_area_px", 0.0, 50000.0, 50.0, false},
    {"target_strike.rho_scale_near", 0.05, 2.0, 0.05, false},
    {"target_strike.rho_scale_far", 0.05, 1.5, 0.05, false},
    {"target_strike.rho_scale_near_ratio", 1.01, 10.0, 0.01, false},
    {"target_strike.rho_scale_far_ratio", 0.01, 0.99, 0.01, false},
    {"target_strike.approach_drive_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.approach_e_rho_deadband", 0.0, 2.0, 0.01, false},
    {"target_strike.approach_pitch_rate_gain", 0.0, 2.0, 0.01, false},
    {"target_strike.approach_pitch_rate_max_rad_s", 0.0, 1.5, 0.01, false},
    {"target_strike.approach_pitch_output_sign", -1.0, 1.0, 1.0, false},
    {"target_strike.preclimb_level_gate_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.preclimb_level_max_roll_rad", 0.0, 0.8, 0.01, false},
    {"target_strike.preclimb_level_max_pitch_rad", 0.0, 0.8, 0.01, false},
    {"target_strike.final_approach_boost_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_area_ratio_enter", 0.001, 1.0, 0.001, false},
    {"target_strike.final_approach_area_ratio_exit", 0.0, 0.99, 0.001, false},
    {"target_strike.final_approach_jerk_scale", 1.0, 5.0, 0.05, false},
    {"target_strike.final_approach_roll_rate_scale", 0.1, 4.0, 0.01, false},
    {"target_strike.final_approach_kp_scale", 0.1, 5.0, 0.05, false},
    {"target_strike.final_approach_kd_scale", 0.0, 5.0, 0.05, false},
    {"target_strike.final_approach_pixel_dot_lpf_scale", 0.05, 1.0, 0.05, false},
    {"target_strike.final_approach_rate_lpf_scale", 0.05, 1.0, 0.05, false},
    {"target_strike.final_approach_deadband_scale", 0.0, 1.0, 0.05, false},
    {"target_strike.final_approach_roll_level_start_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_roll_level_end_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_roll_level_kp", 0.0, 5.0, 0.05, false},
    {"target_strike.final_approach_roll_level_max_rad_s", 0.0, 1.5, 0.02, false},
    {"target_strike.final_approach_pitch_level_start_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_pitch_level_end_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_pitch_level_kp", 0.0, 5.0, 0.05, false},
    {"target_strike.final_approach_pitch_level_max_rad_s", 0.0, 1.5, 0.02, false},
    {"target_strike.final_approach_tilt_aim_comp_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_tilt_aim_comp_start_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_tilt_aim_comp_end_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_tilt_aim_comp_gain", 0.0, 3.0, 0.05, false},
    {"target_strike.final_approach_tilt_aim_comp_max_px", 0.0, 320.0, 5.0, false},
    {"target_strike.final_approach_tilt_aim_comp_roll_sign", -1.0, 1.0, 1.0, false},
    {"target_strike.final_approach_tilt_aim_comp_pitch_sign", -1.0, 1.0, 1.0, false},
    {"target_strike.final_approach_tilt_slowdown_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_tilt_slowdown_start_rad", 0.0, 0.8, 0.01, false},
    {"target_strike.final_approach_tilt_slowdown_end_rad", 0.0, 0.8, 0.01, false},
    {"target_strike.final_approach_tilt_slowdown_min_scale", 0.0, 1.0, 0.05, false},
    {"target_strike.final_approach_commit_align_gate_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_commit_align_max_error_x_px", 0.0, 120.0, 2.0, false},
    {"target_strike.final_approach_commit_align_max_error_y_px", 0.0, 120.0, 2.0, false},
    {"target_strike.final_approach_commit_align_hold_s", 0.0, 0.5, 0.01, false},
    {"target_strike.final_approach_unaligned_slowdown_start_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_unaligned_slowdown_end_ratio", 0.0, 0.20, 0.001, false},
    {"target_strike.final_approach_unaligned_thrust_excess_scale", 0.0, 1.0, 0.05, false},
    {"target_strike.final_approach_commit_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_commit_command_hold_s", 0.0, 3.0, 0.05, false},
    {"target_strike.final_approach_commit_detection_stale_s", 0.05, 3.0, 0.05, false},
    {"target_strike.final_approach_commit_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"target_strike.final_approach_commit_snapshot_max_area_ratio", 0.0, 0.20, 0.005, false},
    {"target_strike.final_approach_commit_freeze_on_edge_protect", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_commit_forward_pitch_rate_rad_s", 0.0, 1.0, 0.02, false},
    {"target_strike.final_approach_commit_roll_level_kp", 0.0, 5.0, 0.05, false},
    {"target_strike.final_approach_commit_roll_level_max_rad_s", 0.0, 1.5, 0.02, false},
    {"target_strike.final_approach_commit_pitch_level_kp", 0.0, 5.0, 0.05, false},
    {"target_strike.final_approach_commit_pitch_level_max_rad_s", 0.0, 1.5, 0.02, false},
    {"target_strike.final_approach_commit_yaw_lock_enabled", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_commit_min_margin_x_px", 0.0, 240.0, 5.0, false},
    {"target_strike.final_approach_commit_min_margin_y_px", 0.0, 200.0, 5.0, false},
    {"target_strike.final_approach_edge_protect_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_edge_margin_x_px", 0.0, 240.0, 5.0, false},
    {"target_strike.final_approach_edge_margin_y_px", 0.0, 240.0, 5.0, false},
    {"target_strike.final_approach_edge_predict_s", 0.0, 1.0, 0.05, false},
    {"target_strike.final_approach_edge_roll_kp_rate", 0.0, 20.0, 0.1, false},
    {"target_strike.final_approach_edge_pitch_kp_rate", 0.0, 20.0, 0.1, false},
    {"target_strike.final_approach_edge_pitch_boost_max_rad_s", 0.0, 1.0, 0.02, false},
    {"target_strike.final_approach_edge_thrust_scale", 0.1, 1.0, 0.05, false},
    {"target_strike.final_approach_thrust_taper_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_thrust_taper_start_ratio", 0.0, 0.05, 0.0005, false},
    {"target_strike.final_approach_thrust_taper_end_ratio", 0.0, 0.05, 0.0005, false},
    {"target_strike.final_approach_thrust_taper_min_scale", 0.1, 1.0, 0.05, false},
    {"target_strike.final_approach_thrust_hard_cap_enable", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_pitch_ff_erho_gain", 0.0, 2.0, 0.05, false},
    {"target_strike.final_approach_kp_proximity_gain", 0.0, 3.0, 0.05, false},
    {"target_strike.final_approach_kd_proximity_gain", 0.0, 3.0, 0.05, false},
    {"target_strike.final_approach_proximity_error_norm", 0.0, 1.0, 1.0, true},
    {"target_strike.final_approach_fallback_decay_tau_s", 0.1, 5.0, 0.05, false},
    {"target_strike.final_approach_fallback_max_s", 0.0, 10.0, 0.1, false},
    {"target_strike.final_approach_fallback_pitch_bias_rad", -0.6, 0.6, 0.01, false},
    {"target_strike.yaw_lock_enabled", 0.0, 1.0, 1.0, true},
    {"target_strike.yaw_bearing_kp", 0.0, 5.0, 0.05, false},
    {"target_strike.yaw_hold_gain", 0.0, 3.0, 0.02, false},
    {"target_strike.yaw_rate_min_rad_s", -6.28, 6.28, 0.05, false},
    {"target_strike.yaw_rate_max_rad_s", -6.28, 6.28, 0.05, false},
    {"target_strike.yaw_hold_deadband_rad", 0.0, 0.1, 0.001, false},
    {"target_strike.yaw_track_deadband_rad", 0.0, 0.3, 0.002, false},
    {"target_strike.yaw_rate_lpf_tau_s", 0.0, 1.0, 0.01, false},
    {"target_strike.constant_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"target_strike.waiting_altitude_kp", 0.0, 1.0, 0.01, false},
    {"target_strike.waiting_altitude_kd", 0.0, 1.0, 0.01, false},
    {"target_strike.waiting_altitude_max_correction", 0.0, 0.5, 0.005, false},
    {"target_strike.tilt_cos_floor", 0.20, 0.99, 0.01, false},
    {"target_strike.thrust_scalar_min", 0.0, 0.95, 0.005, false},
    {"target_strike.thrust_scalar_max", 0.05, 1.0, 0.005, false},
};

// Whitelist for HTTP tuning of the rate-based vision_tracking_rates_ctrl_executor.
// Mirrors gz_vision_tracking_rates_ctrl_top_cam.yaml. Values that the executor
// clampParams() refuses (e.g. attitude_thrust_z_max > -0.05) are bounded on the
// safe side here so the slider cannot push the executor into a runtime reject.
constexpr TunableParamDesc kRatesCtrlTunableParams[] = {
    {"vision_tracking_rates_ctrl.dry_run", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.require_armed_to_command", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.rate_shaper_diag_log", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.enable_tilt_compensation", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.hover_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"vision_tracking_rates_ctrl.tracking_thrust_ramp_tau_s", 0.0, 2.0, 0.01,
     false},
    {"vision_tracking_rates_ctrl.min_score", 0.0, 1.0, 0.01, false},
    {"vision_tracking_rates_ctrl.detection_stale_s", 0.05, 2.0, 0.05, false},
    {"vision_tracking_rates_ctrl.lost_timeout_s", 0.2, 10.0, 0.1, false},
    {"vision_tracking_rates_ctrl.lateral_kp_rate", 0.0, 12.0, 0.1, false},
    {"vision_tracking_rates_ctrl.lateral_kd_rate", 0.0, 4.0, 0.02, false},
    {"vision_tracking_rates_ctrl.longitudinal_kp_rate", 0.0, 12.0, 0.1, false},
    {"vision_tracking_rates_ctrl.longitudinal_kd_rate", 0.0, 4.0, 0.02, false},
    {"vision_tracking_rates_ctrl.pixel_dot_lpf_tau_s", 0.0, 0.6, 0.005, false},
    {"vision_tracking_rates_ctrl.lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"vision_tracking_rates_ctrl.longitudinal_output_sign", -1.0, 1.0, 1.0, false},
    {"vision_tracking_rates_ctrl.x_deadband", 0.0, 0.20, 0.002, false},
    {"vision_tracking_rates_ctrl.y_deadband", 0.0, 0.20, 0.002, false},
    {"vision_tracking_rates_ctrl.x_deadband_px", 0.0, 200.0, 1.0, false},
    {"vision_tracking_rates_ctrl.y_deadband_px", 0.0, 200.0, 1.0, false},
    {"vision_tracking_rates_ctrl.rate_lpf_tau_s", 0.0, 0.30, 0.005, false},
    {"vision_tracking_rates_ctrl.max_jerk_rad_s2", 0.0, 200.0, 1.0, false},
    // 滑条上限 6.9813 rad/s ≈ 400 deg/s；同步要求 hpp 的 kAbsMaxRateRadS ≥ 6.9813，
    // 否则 clampParams 会先把值截回 kAbsMaxRateRadS。
    {"vision_tracking_rates_ctrl.max_roll_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"vision_tracking_rates_ctrl.max_pitch_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"vision_tracking_rates_ctrl.max_roll_angle_rad", 0.05, 1.5, 0.01, false},
    {"vision_tracking_rates_ctrl.max_pitch_angle_rad", 0.05, 1.5, 0.01, false},
    {"vision_tracking_rates_ctrl.tilt_softcap_band_rad", 0.01, 0.6, 0.005, false},
    {"vision_tracking_rates_ctrl.tilt_hardcap_margin_rad", 0.0, 0.6, 0.005, false},
    {"vision_tracking_rates_ctrl.hard_level_kp", 0.0, 15.0, 0.1, false},
    {"vision_tracking_rates_ctrl.level_kp", 0.0, 10.0, 0.05, false},
    {"vision_tracking_rates_ctrl.level_deadband_rad", 0.0, 0.1, 0.001, false},
    {"vision_tracking_rates_ctrl.lost_target_rate_decay_tau_s", 0.0, 1.5, 0.01, false},
    {"vision_tracking_rates_ctrl.desired_bbox_area_px", 0.0, 50000.0, 50.0, false},
    {"vision_tracking_rates_ctrl.rho_scale_near", 0.05, 2.0, 0.05, false},
    {"vision_tracking_rates_ctrl.rho_scale_far", 0.05, 1.5, 0.05, false},
    {"vision_tracking_rates_ctrl.rho_scale_near_ratio", 1.01, 10.0, 0.01, false},
    {"vision_tracking_rates_ctrl.rho_scale_far_ratio", 0.01, 0.99, 0.01, false},
    {"vision_tracking_rates_ctrl.final_approach_boost_enable", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.final_approach_area_ratio_enter", 0.001, 1.0, 0.001, false},
    {"vision_tracking_rates_ctrl.final_approach_area_ratio_exit", 0.0, 0.99, 0.001, false},
    {"vision_tracking_rates_ctrl.final_approach_jerk_scale", 1.0, 5.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_roll_rate_scale", 0.1, 1.0, 0.01, false},
    {"vision_tracking_rates_ctrl.final_approach_kp_scale", 0.1, 5.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_kd_scale", 0.0, 5.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_pixel_dot_lpf_scale", 0.05, 1.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_rate_lpf_scale", 0.05, 1.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_deadband_scale", 0.0, 1.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_commit_enable", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.final_approach_commit_command_hold_s", 0.0, 1.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_commit_detection_stale_s", 0.05, 1.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_commit_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"vision_tracking_rates_ctrl.final_approach_commit_yaw_lock_enabled", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.final_approach_commit_min_margin_x_px", 0.0, 240.0, 5.0, false},
    {"vision_tracking_rates_ctrl.final_approach_commit_min_margin_y_px", 0.0, 200.0, 5.0, false},
    // Part A: pitch feedforward gain on d(e_rho)/dt; closing rate (e_rho_dot<0)
    // injects ff_gain * e_rho_dot into pitch_rate_des. 0 = disable.
    {"vision_tracking_rates_ctrl.final_approach_pitch_ff_erho_gain", 0.0, 2.0, 0.05, false},
    // Part B: proximity-adaptive Kp/Kd boost — effective scale =
    // base_scale * (1 + proximity_gain * max(0, -e_rho)). Larger = stronger
    // gain ramp as the drone gets closer. Keep Kd_proximity ≥ Kp_proximity * 0.5
    // to avoid losing damping ratio at proximity > 0.
    {"vision_tracking_rates_ctrl.final_approach_kp_proximity_gain", 0.0, 3.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_kd_proximity_gain", 0.0, 3.0, 0.05, false},
    // Multiply PD error inputs by exp(min(0, e_rho)) during FA to cancel the
    // 1/distance angular amplification at close range.
    {"vision_tracking_rates_ctrl.final_approach_proximity_error_norm", 0.0, 1.0, 1.0, true},
    // Part C: FA fallback (after commit hold expires) — slow rate decay
    // (larger τ = more forward momentum), max duration cap before Waiting,
    // and a target pitch bias to maintain forward drive.
    {"vision_tracking_rates_ctrl.final_approach_fallback_decay_tau_s", 0.1, 5.0, 0.05, false},
    {"vision_tracking_rates_ctrl.final_approach_fallback_max_s", 0.0, 10.0, 0.1, false},
    {"vision_tracking_rates_ctrl.final_approach_fallback_pitch_bias_rad", -0.6, 0.6, 0.01, false},
    // 布尔开关：true = publishRates() 入口直接把 yaw_rate_sp 钳为 0、并把
    // yaw LPF 状态归零，PX4 内环跟踪 yaw_rate=0 即等效于"锁住当前航向"。
    // 仅 roll / pitch 继续视觉跟踪。其余 yaw_* 滑条对 yaw_lock 打开时无效。
    {"vision_tracking_rates_ctrl.yaw_lock_enabled", 0.0, 1.0, 1.0, true},
    {"vision_tracking_rates_ctrl.yaw_bearing_kp", 0.0, 5.0, 0.05, false},
    {"vision_tracking_rates_ctrl.yaw_hold_gain", 0.0, 3.0, 0.02, false},
    {"vision_tracking_rates_ctrl.yaw_rate_min_rad_s", -6.28, 6.28, 0.05, false},
    {"vision_tracking_rates_ctrl.yaw_rate_max_rad_s", -6.28, 6.28, 0.05, false},
    {"vision_tracking_rates_ctrl.yaw_hold_deadband_rad", 0.0, 0.1, 0.001, false},
    {"vision_tracking_rates_ctrl.yaw_track_deadband_rad", 0.0, 0.3, 0.002, false},
    {"vision_tracking_rates_ctrl.yaw_rate_lpf_tau_s", 0.0, 1.0, 0.01, false},
    {"vision_tracking_rates_ctrl.constant_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"vision_tracking_rates_ctrl.waiting_altitude_kp", 0.0, 1.0, 0.01, false},
    {"vision_tracking_rates_ctrl.waiting_altitude_kd", 0.0, 1.0, 0.01, false},
    {"vision_tracking_rates_ctrl.waiting_altitude_max_correction", 0.0, 0.5, 0.005, false},
    {"vision_tracking_rates_ctrl.tilt_cos_floor", 0.20, 0.99, 0.01, false},
    {"vision_tracking_rates_ctrl.attitude_thrust_z_min", -1.0, -0.05, 0.005, false},
    // Executor refuses values > -0.05 (would be near-zero or upward). Cap the
    // upper bound at -0.10 so the slider stays inside the safe envelope.
    {"vision_tracking_rates_ctrl.attitude_thrust_z_max", -0.95, -0.05, 0.005, false},
};

// Whitelist for HTTP tuning of the target_strike_png executor (visual PNG → body
// rates). Mirrors the target_strike_png.* ROS parameters declared in
// target_strike_png_main.cpp (and the BF strike_png_param_tune kPngDescs set).
// Keys not declared by the executor (derotate gains, residual_rate_limit) are
// intentionally omitted so a POST never targets an undeclared parameter.
constexpr TunableParamDesc kTargetStrikePngTunableParams[] = {
    {"target_strike_png.dry_run", 0.0, 1.0, 1.0, true},
    {"target_strike_png.require_armed_to_command", 0.0, 1.0, 1.0, true},
    {"target_strike_png.min_score", 0.0, 1.0, 0.01, false},
    {"target_strike_png.detection_stale_s", 0.05, 2.0, 0.05, false},
    {"target_strike_png.hover_thrust_z", 0.0, 0.99, 0.005, false},
    {"target_strike_png.strike_thrust_z", 0.0, 0.99, 0.005, false},
    {"target_strike_png.target_lost_hold_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.target_lost_hold_delay_s", 0.0, 5.0, 0.05, false},
    {"target_strike_png.entry_smooth_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.entry_smooth_duration_s", 0.0, 5.0, 0.05, false},
    {"target_strike_png.entry_smooth_initial_thrust_z", 0.0, 0.99, 0.005, false},
    {"target_strike_png.enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.max_roll_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"target_strike_png.max_pitch_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"target_strike_png.pixel_dot_lpf_tau_s", 0.0, 0.6, 0.005, false},
    {"target_strike_png.nav_ratio_x", 0.0, 8.0, 0.1, false},
    {"target_strike_png.nav_ratio_y", 0.0, 8.0, 0.1, false},
    {"target_strike_png.derotate_body_rates", 0.0, 1.0, 1.0, true},
    {"target_strike_png.closure_base_scale", 0.0, 2.0, 0.05, false},
    {"target_strike_png.closure_area_gain", 0.0, 2.0, 0.05, false},
    {"target_strike_png.max_feedforward_rad_s", 0.0, 6.9813, 0.05, false},
    {"target_strike_png.fov_trim_kp_rate", 0.0, 4.0, 0.05, false},
    {"target_strike_png.los_rate_hold_tau_s", 0.0, 2.0, 0.01, false},
    {"target_strike_png.edge_guard_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.edge_guard_start_norm", 0.0, 1.0, 0.01, false},
    {"target_strike_png.edge_guard_full_norm", 0.0, 1.0, 0.01, false},
    {"target_strike_png.edge_guard_kp_rate", 0.0, 4.0, 0.05, false},
    {"target_strike_png.edge_guard_min_rate_rad_s", 0.0, 4.0, 0.01, false},
    {"target_strike_png.edge_guard_max_rate_rad_s", 0.0, 6.9813, 0.05, false},
    {"target_strike_png.edge_guard_pitch_scale", 0.0, 2.0, 0.05, false},
    {"target_strike_png.pursuit_fallback_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.pursuit_fallback_kp_rate", 0.0, 4.0, 0.05, false},
    {"target_strike_png.pursuit_fallback_start_norm", 0.0, 1.0, 0.01, false},
    {"target_strike_png.pursuit_fallback_full_norm", 0.0, 1.0, 0.01, false},
    {"target_strike_png.pursuit_fallback_min_rate_rad_s", 0.0, 4.0, 0.01, false},
    {"target_strike_png.pursuit_fallback_max_rate_rad_s", 0.0, 6.9813, 0.05, false},
    {"target_strike_png.pursuit_fallback_png_weak_rate_rad_s", 0.0, 4.0, 0.01, false},
    {"target_strike_png.terminal_stale_lateral_trim_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.terminal_stale_lateral_trim_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_stale_lateral_trim_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_stale_lateral_trim_stale_s_start", 0.0, 2.0, 0.01, false},
    {"target_strike_png.terminal_stale_lateral_trim_stale_s_full", 0.0, 2.0, 0.01, false},
    {"target_strike_png.terminal_stale_lateral_trim_kp_rate", 0.0, 10.0, 0.1, false},
    {"target_strike_png.terminal_stale_lateral_trim_max_rate_rad_s", 0.0, 4.0, 0.05, false},
    {"target_strike_png.terminal_intercept_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.terminal_intercept_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_intercept_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_intercept_lead_s", 0.0, 1.0, 0.01, false},
    {"target_strike_png.terminal_intercept_kp_rate", 0.0, 10.0, 0.1, false},
    {"target_strike_png.terminal_intercept_max_rate_rad_s", 0.0, 4.0, 0.05, false},
    {"target_strike_png.terminal_crossing_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.terminal_crossing_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_crossing_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_crossing_rate_start_norm_s", 0.0, 2.0, 0.01, false},
    {"target_strike_png.terminal_crossing_rate_full_norm_s", 0.0, 2.0, 0.01, false},
    {"target_strike_png.terminal_crossing_kd_rate", 0.0, 4.0, 0.05, false},
    {"target_strike_png.terminal_crossing_max_rate_rad_s", 0.0, 4.0, 0.05, false},
    {"target_strike_png.terminal_forward_speed_guard_enable", 0.0, 1.0, 1.0, true},
    {"target_strike_png.terminal_forward_speed_guard_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_forward_speed_guard_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"target_strike_png.terminal_forward_speed_guard_start_m_s", 0.0, 80.0, 0.5, false},
    {"target_strike_png.terminal_forward_speed_guard_full_m_s", 0.0, 80.0, 0.5, false},
    {"target_strike_png.terminal_forward_speed_guard_min_positive_pitch_scale", 0.0, 1.0, 0.05, false},
    {"target_strike_png.lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"target_strike_png.longitudinal_output_sign", -1.0, 1.0, 1.0, false},
};

// Discriminator for HTTP tuning whitelist. Match target_strike_png before
// target_strike (substring) and target_strike before rates_ctrl so names like
// target_strike_png_executor / target_strike_executor route correctly.
enum class TuneMode { Vision, Vertical, RatesCtrl, TargetStrike, TargetStrikePng };

inline TuneMode detectTuneMode(const std::string& tune_target_node) {
  if (tune_target_node.find("target_strike_png") != std::string::npos) {
    return TuneMode::TargetStrikePng;
  }
  if (tune_target_node.find("target_strike") != std::string::npos) {
    return TuneMode::TargetStrike;
  }
  if (tune_target_node.find("rates_ctrl") != std::string::npos) {
    return TuneMode::RatesCtrl;
  }
  if (tune_target_node.find("vertical") != std::string::npos) {
    return TuneMode::Vertical;
  }
  return TuneMode::Vision;
}

inline const char* tuneModeKey(TuneMode m) {
  switch (m) {
    case TuneMode::TargetStrikePng: return "target_strike_png";
    case TuneMode::TargetStrike: return "target_strike";
    case TuneMode::RatesCtrl: return "rates_ctrl";
    case TuneMode::Vertical:  return "vertical";
    case TuneMode::Vision:    return "vision";
  }
  return "vision";
}

inline const char* tuneModeParamPrefix(TuneMode m) {
  switch (m) {
    case TuneMode::TargetStrikePng: return "target_strike_png.";
    case TuneMode::TargetStrike: return "target_strike.";
    case TuneMode::RatesCtrl: return "vision_tracking_rates_ctrl.";
    case TuneMode::Vertical:  return "vertical_tracking.";
    case TuneMode::Vision:    return "vision_tracking.";
  }
  return "vision_tracking.";
}

inline const TunableParamDesc* findTunable(const std::string& name,
                                    const std::string& tune_target_node) {
  switch (detectTuneMode(tune_target_node)) {
    case TuneMode::TargetStrikePng:
      for (const auto& d : kTargetStrikePngTunableParams) {
        if (name == d.name) return &d;
      }
      return nullptr;
    case TuneMode::TargetStrike:
      for (const auto& d : kTargetStrikeTunableParams) {
        if (name == d.name) return &d;
      }
      return nullptr;
    case TuneMode::RatesCtrl:
      for (const auto& d : kRatesCtrlTunableParams) {
        if (name == d.name) return &d;
      }
      return nullptr;
    case TuneMode::Vertical:
      for (const auto& d : kVerticalTunableParams) {
        if (name == d.name) return &d;
      }
      return nullptr;
    case TuneMode::Vision:
      for (const auto& d : kVisionTunableParams) {
        if (name == d.name) return &d;
      }
      return nullptr;
  }
  return nullptr;
}

constexpr TunableParamDesc kTrackingDebugSelfParams[] = {
    {"tracking_debug_max_fps", 1.0, 60.0, 0.5, false},
};

inline const TunableParamDesc* findSelfTunable(const std::string& name) {
  for (const auto& d : kTrackingDebugSelfParams) {
    if (name == d.name) return &d;
  }
  return nullptr;
}

inline std::optional<std::string> jsonExtractString(const std::string& body,
                                             const char* key) {
  const std::string needle = std::string("\"") + key + "\"";
  size_t p = body.find(needle);
  if (p == std::string::npos) return std::nullopt;
  p = body.find(':', p + needle.size());
  if (p == std::string::npos) return std::nullopt;
  ++p;
  while (p < body.size() &&
         (body[p] == ' ' || body[p] == '\t' || body[p] == '\n' || body[p] == '\r')) {
    ++p;
  }
  if (p >= body.size() || body[p] != '"') return std::nullopt;
  ++p;
  const size_t start = p;
  while (p < body.size() && body[p] != '"') ++p;
  if (p >= body.size()) return std::nullopt;
  return body.substr(start, p - start);
}

inline bool jsonExtractBoolOrDouble(const std::string& body, bool* as_bool, bool* is_bool,
                             double* as_double) {
  const char* key = "\"value\"";
  size_t p = body.find(key);
  if (p == std::string::npos) return false;
  p = body.find(':', p + std::strlen(key));
  if (p == std::string::npos) return false;
  ++p;
  while (p < body.size() &&
         (body[p] == ' ' || body[p] == '\t' || body[p] == '\n' || body[p] == '\r')) {
    ++p;
  }
  if (p >= body.size()) return false;
  if (body.compare(p, 4, "true") == 0) {
    *is_bool = true;
    *as_bool = true;
    return true;
  }
  if (body.compare(p, 5, "false") == 0) {
    *is_bool = true;
    *as_bool = false;
    return true;
  }
  *is_bool = false;
  char* end = nullptr;
  *as_double = std::strtod(body.c_str() + p, &end);
  if (end == body.c_str() + p) return false;
  return std::isfinite(*as_double);
}

}  // namespace circle::debug
