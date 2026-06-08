#pragma once

#include "circle/strike/delayed_pixel_kalman.hpp"
#include "circle/strike/modules/tilt_envelope.hpp"
#include "circle/strike_png/visual_png_guidance.hpp"

#include <cstdint>

namespace circle::strike_png {

// Stateless attitude safety envelope applied to the final rate command.
// Angles/rates are expressed in degrees here for config clarity; the controller
// converts to rad/rad-s internally when applying the cap.
struct TiltCapParams {
  bool enable{false};
  float max_roll_angle_deg{30.0F};
  float max_pitch_angle_deg{40.0F};
  float softcap_band_deg{10.0F};
  float hardcap_margin_deg{6.0F};
  float hardcap_level_kp{3.0F};
  float hardcap_max_level_rate_deg_s{86.0F};
  float out_lpf_tau_s{0.0F};        // output smoothing LPF (s); 0 = off
  float out_max_jerk_deg_s2{0.0F};  // output jerk limit (deg/s^2); 0 = off
};

struct StrikePngParams {
  bool enable{true};
  float max_roll_rate_rad_s{1.2F};
  float max_pitch_rate_rad_s{1.2F};
  float pixel_dot_lpf_tau_s{0.08F};
  bool dkf_enable{false};
  circle::strike::DelayedPixelKalman::Params dkf{};
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
  float los_rate_hold_tau_s{0.20F};
  bool visual_prediction_enable{true};
  float visual_prediction_max_age_s{0.25F};
  float visual_prediction_max_offset_norm{0.18F};
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
  // Output-axis sign (±1). PX4 top-cam: lateral=+1, longitudinal=-1; BF top-cam
  // needs longitudinal=+1 (pitch mirrored vs PX4).
  float lateral_output_sign{1.0F};
  float longitudinal_output_sign{-1.0F};
  TiltCapParams tilt_cap{};
};

struct StrikePngInput {
  uint64_t now_ns{0};
  uint64_t measurement_ns{0};
  bool detection_valid{false};
  float ex{0.0F};
  float ey{0.0F};
  float bbox_area_ratio{0.0F};
  float bbox_area_px{0.0F};
  float detection_score{0.0F};
  float fx{0.0F};
  float fy{0.0F};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  bool derotate_rate_valid{true};
  bool attitude_valid{false};
  bool ownship_forward_speed_valid{false};
  float ownship_forward_speed_m_s{0.0F};
  // Current vehicle attitude (rad), gated by attitude_valid. Consumed by both
  // the terminal_tilt_aim guidance term and the tilt_cap envelope.
  float vehicle_roll_rad{0.0F};
  float vehicle_pitch_rad{0.0F};
};

struct StrikePngOutput {
  bool has_target{false};
  bool png_active{false};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float thrust_z{0.0F};
  float ex_dot_filt{0.0F};
  float ey_dot_filt{0.0F};
  float png_closure_scale{1.0F};
  float png_ex_dot_inertial{0.0F};
  float png_ey_dot_inertial{0.0F};
  float measurement_age_s{0.0F};
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
  float roll_tilt_softcap_factor{1.0F};
  float pitch_tilt_softcap_factor{1.0F};
  bool tilt_hardcap_active{false};
  bool visual_prediction_active{false};
  float control_ex{0.0F};
  float control_ey{0.0F};
  bool derotate_rate_valid{false};
};

class StrikePngController {
 public:
  void reset();
  [[nodiscard]] StrikePngOutput tick(const StrikePngParams& params,
                                     const StrikePngInput& input);

 private:
  bool last_valid_{false};
  uint64_t last_stamp_ns_{0};
  float last_ex_{0.0F};
  float last_ey_{0.0F};
  uint64_t last_measurement_ns_{0};
  float ex_dot_filt_{0.0F};
  float ey_dot_filt_{0.0F};
  circle::strike::DelayedPixelKalman dkf_;
  VisualPngGuidance guidance_;
  circle::strike::TiltEnvelope tilt_envelope_;
};

}  // namespace circle::strike_png
