#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "circle/strike/delayed_pixel_kalman.hpp"
#include "circle/strike/modules/terminal_predictor.hpp"
#include "circle/strike/modules/visual_png_guidance.hpp"
#include "circle/strike/strike_telemetry.hpp"
#include "circle/types/detection.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/rate_command.hpp"
#include "circle/types/time.hpp"
#include "circle/vision/detection_filter.hpp"

namespace circle::strike {

enum class StrikeState : uint8_t {
  WaitingTarget = 0,
  Tracking = 1,
  ForceLevel = 2,
  CommitHold = 3,
  FaFallback = 4,
  Complete = 5,
};

enum class GuidanceMode : uint8_t {
  LegacyPd = 0,
  PaperPng = 1,
};

struct TrackerFallbackParams {
  bool enable{true};
  float after_s{0.12F};
  float max_s{0.45F};
  float max_cov_trace{1.0F};
  float min_score{0.50F};
};

struct StrikeConfidenceParams {
  bool gate_enable{true};
  float min_score{0.50F};
  float max_cov_trace{0.30F};
};

struct ImageLeadParams {
  bool enable{false};
  float time_s{0.0F};
  float max_px{120.0F};
};

struct TrackingStartSmoothingParams {
  bool enable{true};
  float smoothing_s{0.80F};
  float kp_scale_initial{1.0F};
  float lead_scale_initial{0.20F};
  float kd_scale_initial{0.35F};
};

struct RhoScaleParams {
  double desired_bbox_area_px{1600.0};
  float scale_near{1.0F};
  float scale_far{0.35F};
  float near_ratio{1.82F};
  float far_ratio{0.33F};
};

struct ApproachDriveParams {
  bool enable{false};
  float e_rho_deadband{0.10F};
  float pitch_rate_gain{0.20F};
  float pitch_rate_max_rad_s{0.25F};
  float pitch_output_sign{-1.0F};
  bool fov_gate_enable{false};
  float fov_gate_high_error_px{160.0F};
  float fov_gate_release_error_px{90.0F};
  float fov_gate_min_scale{0.0F};
};

struct SpeedGovernorParams {
  bool enable{false};
  float start_m_s{8.5F};
  float full_m_s{12.0F};
  float min_image_scale{0.45F};
  float level_kp{1.2F};
  float level_max_rad_s{0.35F};
  float fa_roll_level_blend_max{1.0F};
  float fa_pitch_level_blend_max{1.0F};
  bool fa_gate_enable{false};
  float fa_max_vxy_m_s{9.0F};
};

struct PreclimbParams {
  bool xy_gate_enable{false};
  float xy_error_gate_x_px{200.0F};
  float xy_error_gate_y_px{200.0F};
  float xy_rate_gate_x_px_s{180.0F};
  float xy_rate_gate_y_px_s{180.0F};
  float clear_margin_x_px{40.0F};
  float clear_margin_y_px{40.0F};
  bool level_gate_enable{false};
  float level_max_roll_rad{0.25F};
  float level_max_pitch_rad{0.25F};
  float xy_hold_s{0.35F};
  bool level_assist_enable{false};
  float level_assist_band_rad{0.08F};
  float level_assist_kp{2.5F};
  float level_assist_max_rad_s{0.50F};
  bool safe_hold_enable{false};
  float safe_hold_level_kp{1.2F};
  float safe_hold_level_max_rad_s{0.35F};
  float safe_hold_image_rate_scale{0.30F};
  float safe_hold_max_rate_rad_s{0.35F};
  float safe_hold_tilt_start_rad{0.18F};
  float safe_hold_tilt_full_rad{0.35F};
  float thrust_excess_scale{0.0F};
  float min_thrust_scalar{0.0F};
  bool thrust_hard_cap_enable{false};
  bool release_slowdown_enable{false};
  float release_slowdown_s{0.0F};
  float release_thrust_excess_scale{1.0F};
  float release_min_thrust_scalar{0.0F};
  bool release_thrust_hard_cap_enable{false};
};

struct AscentDampingParams {
  bool image_velocity_damping_enable{false};
  float image_velocity_damping_start_px_s{120.0F};
  float image_velocity_damping_min_scale{0.45F};
};

struct TrackingDeadbandPriorityParams {
  bool enable{false};
  float error_start_scale{1.0F};
  float error_full_scale{4.0F};
  float rate_start_px_s{120.0F};
  float rate_full_px_s{360.0F};
  float min_excess_scale{0.35F};
  bool hard_cap_enable{true};
};

struct FAGateParams {
  bool boost_enable{false};
  float area_ratio_enter{0.18F};
  float area_ratio_exit{0.14F};
  bool altitude_gate_enable{false};
  float altitude_gate_max_gap_m{15.0F};
  bool area_quality_gate_enable{false};
  float area_quality_error_x_px{60.0F};
  float area_quality_error_y_px{80.0F};
  float area_quality_rate_x_px_s{120.0F};
  float area_quality_rate_y_px_s{160.0F};
  float area_quality_max_tilt_rad{0.45F};
  bool tilt_gate_enable{false};
  float tilt_gate_min_area_ratio{0.0F};
  float tilt_gate_start_rad{0.28F};
  float tilt_gate_error_x_px{60.0F};
  float tilt_gate_error_y_px{80.0F};
  float tilt_gate_rate_x_px_s{120.0F};
  float tilt_gate_rate_y_px_s{160.0F};
  bool stable_gate_enable{false};
  float stable_gate_min_area_ratio{0.0F};
  float stable_gate_max_area_ratio{0.0F};
  float stable_gate_error_x_px{40.0F};
  float stable_gate_error_y_px{45.0F};
  float stable_gate_rate_x_px_s{35.0F};
  float stable_gate_rate_y_px_s{45.0F};
  float stable_gate_max_tilt_rad{0.45F};
  float stable_gate_hold_s{0.50F};
  float hold_s{0.0F};
};

struct FAScalingParams {
  float jerk_scale{1.5F};
  float roll_rate_scale{1.0F};
  float kp_scale{1.0F};
  float kd_scale{1.0F};
  float pixel_dot_lpf_scale{1.0F};
  float rate_lpf_scale{1.0F};
  float deadband_scale{0.0F};
  float kp_proximity_gain{1.0F};
  float kd_proximity_gain{0.5F};
  bool proximity_error_norm{true};
  float pitch_ff_erho_gain{0.5F};
};

struct FALevelingParams {
  float roll_level_start_ratio{0.0F};
  float roll_level_end_ratio{0.0F};
  float roll_level_kp{0.0F};
  float roll_level_max_rad_s{0.0F};
  float pitch_level_start_ratio{0.0F};
  float pitch_level_end_ratio{0.0F};
  float pitch_level_kp{0.0F};
  float pitch_level_max_rad_s{0.0F};
};

struct FAPitchChatterGuardParams {
  bool enable{false};
  float max_area_ratio{0.0F};
  float max_error_y_px{0.0F};
  float max_rate_y_px_s{0.0F};
  float prev_min_rate_rad_s{0.20F};
  float max_reversal_rate_rad_s{0.45F};
};

struct FATiltAimCompParams {
  bool enable{false};
  float start_ratio{0.0F};
  float end_ratio{0.0F};
  float gain{1.0F};
  float max_px{120.0F};
  float roll_sign{1.0F};
  float pitch_sign{1.0F};
};

struct FAThrustParams {
  bool tilt_slowdown_enable{false};
  float tilt_slowdown_start_rad{0.10F};
  float tilt_slowdown_end_rad{0.22F};
  float tilt_slowdown_min_scale{0.08F};
  bool vertical_drift_slowdown_enable{false};
  float vertical_drift_start_px_s{120.0F};
  float vertical_drift_min_scale{0.45F};
  bool ascent_budget_enable{false};
  float ascent_budget_tilt_start_rad{0.20F};
  float ascent_budget_tilt_full_rad{0.45F};
  float ascent_budget_y_rate_start_px_s{80.0F};
  float ascent_budget_y_rate_full_px_s{240.0F};
  float ascent_budget_y_error_start_px{45.0F};
  float ascent_budget_y_error_full_px{180.0F};
  float ascent_budget_min_excess_scale{0.15F};
  bool ascent_budget_hard_cap_enable{true};
  float min_thrust_scalar{0.0F};
  bool min_thrust_budget_relax_enable{false};
  float min_thrust_budget_relax_scale{1.0F};
  bool thrust_taper_enable{true};
  float thrust_taper_start_ratio{0.004F};
  float thrust_taper_end_ratio{0.007F};
  float thrust_taper_min_scale{0.82F};
  bool thrust_taper_area_enable{true};
  bool thrust_taper_edge_enable{false};
  float thrust_taper_edge_start_score{0.20F};
  float thrust_taper_edge_full_score{1.0F};
  bool thrust_taper_tilt_enable{false};
  float thrust_taper_tilt_start_rad{0.20F};
  float thrust_taper_tilt_full_rad{0.45F};
  bool thrust_hard_cap_enable{false};
  float unaligned_slowdown_start_ratio{0.024F};
  float unaligned_slowdown_end_ratio{0.080F};
  float unaligned_thrust_excess_scale{0.25F};
};

struct FAEdgeProtectParams {
  bool enable{true};
  float margin_x_px{48.0F};
  float margin_y_px{48.0F};
  float predict_s{0.0F};
  float roll_kp_rate{4.0F};
  float pitch_kp_rate{4.0F};
  float pitch_boost_max_rad_s{0.35F};
  float thrust_scale{0.85F};
};

struct FABottomPitchGuardParams {
  bool enable{false};
  float margin_px{80.0F};
  float error_start_px{70.0F};
  float error_full_px{150.0F};
  float level_kp{2.0F};
  float max_rad_s{0.65F};
};

struct FACommitParams {
  bool enable{true};
  float command_hold_s{0.70F};
  float detection_stale_s{0.60F};
  float min_latch_s{0.0F};
  float thrust_scalar{0.50F};
  float min_area_ratio{0.0F};
  float terminal_min_area_ratio{0.0F};
  float terminal_max_error_x_px{0.0F};
  float terminal_max_error_y_px{0.0F};
  bool stable_bypass_area_enable{false};
  bool start_on_terminal_ready_enable{false};
  bool recent_centered_handoff_enable{false};
  float recent_centered_handoff_min_area_ratio{0.0F};
  float recent_centered_handoff_max_age_s{0.0F};
  float recent_centered_handoff_trigger_error_x_px{0.0F};
  float recent_centered_handoff_trigger_error_y_px{0.0F};
  float recent_centered_handoff_live_blend{1.0F};
  float recent_centered_hold_through_s{0.0F};
  float thrust_ramp_s{0.0F};
  float snapshot_max_area_ratio{0.0F};
  bool align_gate_enable{true};
  float align_max_error_x_px{30.0F};
  float align_max_error_y_px{30.0F};
  float align_max_rate_x_px_s{0.0F};
  float align_max_rate_y_px_s{0.0F};
  float align_hold_s{0.08F};
  bool future_gate_enable{false};
  float future_lead_s{0.0F};
  float future_max_error_x_px{0.0F};
  float future_max_error_y_px{0.0F};
  bool tilt_gate_enable{false};
  float max_tilt_rad{0.0F};
  bool blind_commit_enable{false};
  float blind_commit_min_area_ratio{0.0F};
  float blind_commit_edge_margin_x_px{0.0F};
  float blind_commit_edge_margin_y_px{0.0F};
  bool blind_commit_edge_vertical_gate_enable{false};
  bool blind_commit_edge_horizontal_gate_enable{false};
  float blind_commit_edge_max_error_x_px{0.0F};
  float blind_commit_edge_max_rate_x_px_s{0.0F};
  float blind_commit_edge_max_error_y_px{0.0F};
  float blind_commit_edge_max_rate_y_px_s{0.0F};
  float blind_commit_min_score{0.0F};
  float blind_commit_trend_lead_s{0.0F};
  float blind_commit_trend_max_rate_rad_s{0.0F};
  bool predictive_enable{false};
  float predict_lead_s{0.0F};
  float predict_kp_rate{0.0F};
  float predict_kd_rate{0.0F};
  float predict_max_rate_rad_s{0.0F};
  float predict_blend_s{0.0F};
  bool freeze_on_edge_protect{false};
  float forward_pitch_rate_rad_s{0.0F};
  float roll_level_kp{0.0F};
  float roll_level_max_rad_s{0.0F};
  float roll_level_blend{1.0F};
  float pitch_level_kp{0.0F};
  float pitch_level_max_rad_s{0.0F};
  float pitch_level_blend{1.0F};
  bool yaw_lock_enabled{true};
  float min_margin_x_px{60.0F};
  float min_margin_y_px{50.0F};
};

struct FAFallbackParams {
  float decay_tau_s{1.5F};
  float max_s{2.0F};
  float pitch_bias_rad{-0.15F};
};

struct YawParams {
  float bearing_kp{0.6F};
  float hold_gain{0.5F};
  float rate_min_rad_s{-0.7F};
  float rate_max_rad_s{0.7F};
  float hold_deadband_rad{0.0087F};
  float track_deadband_rad{0.0524F};
  float rate_lpf_tau_s{0.15F};
  bool lock_enabled{false};
};

struct WaitingParams {
  bool level_hold_enabled{true};
  float level_kp{3.0F};
  float level_deadband_rad{0.0087F};
  float altitude_kp{0.20F};
  float altitude_kd{0.10F};
  float altitude_max_correction{0.18F};
};

struct ForceLevelParams {
  float hard_level_kp{5.0F};
  int min_hold_ms{200};
};

struct TiltCapParams {
  float max_roll_angle_rad{0.5235987755982988F};
  float max_pitch_angle_rad{1.0471975511965976F};
  float softcap_band_rad{0.15F};
  float hardcap_margin_rad{0.10F};
};

struct ThrustParams {
  bool enable_tilt_compensation{false};
  float hover_scalar{0.72F};
  float constant_scalar{0.77F};
  float tracking_ramp_tau_s{0.25F};
  float slew_rate_scalar_s{0.0F};
  float tilt_cos_floor{0.5F};
  /// Published thrust_z scalar lower bound (positive, 0–1). BF: throttle fraction;
  /// PX4: negated to FRD body.z after core clamp.
  float scalar_min{0.01F};
  /// Published thrust_z scalar upper bound (positive, 0–1).
  float scalar_max{0.99F};
};

struct TargetLossParams {
  bool complete_on_loss_enable{true};
  float loss_complete_s{0.8F};
  float lost_target_rate_decay_tau_s{0.30F};
  bool complete_altitude_gate_enable{false};
  float complete_max_altitude_gap_m{3.0F};
};

struct EvaluationParams {
  bool target_altitude_enable{false};
  float target_altitude_m{0.0F};
  float success_altitude_gap_m{3.0F};
  bool relative_distance_enable{true};
  float relative_distance_max_age_s{0.25F};
  float target_origin_offset_x_m{0.0F};
  float target_origin_offset_y_m{0.0F};
};

struct TiltGuardParams {
  bool enable{false};
  float max_tilt_rad{0.785F};
  float recovery_kp{0.8F};
};

struct DirectionalDiveParams {
  bool enable{false};
  float start_edge_ratio{0.20F};
  float end_edge_ratio{0.30F};
  float dive_roll_gain{1.2F};
  float dive_pitch_gain{0.8F};
  float climb_thrust{0.90F};
  float descend_thrust{0.55F};
  float cruise_thrust{0.75F};
  float lead_time_s{0.10F};
  float detection_max_age_s{0.30F};
  float min_horizontal_boost{3.0F};
  float tilt_comp_threshold{0.85F};
  float tilt_comp_gain{0.5F};
};

struct FinalApproachParams {
  FAGateParams gate{};
  FAScalingParams scaling{};
  FALevelingParams leveling{};
  FAPitchChatterGuardParams pitch_chatter_guard{};
  FATiltAimCompParams tilt_aim_comp{};
  TerminalPredictorParams terminal_predictor{};
  FAThrustParams thrust{};
  FAEdgeProtectParams edge_protect{};
  FABottomPitchGuardParams bottom_pitch_guard{};
  FACommitParams commit{};
  FAFallbackParams fallback{};
};

struct StrikeParams {
  vision::DetectionFilterParams filter{};
  double detection_stale_s{0.4};
  double lost_timeout_s{2.0};

  DelayedPixelKalman::Params dkf{};
  bool dkf_enable{true};
  GuidanceMode guidance_mode{GuidanceMode::LegacyPd};
  VisualPngGuidanceParams visual_png{};
  RhoRateWindowParams rho_rate_window{};

  TrackerFallbackParams tracker_fallback{};
  StrikeConfidenceParams confidence{};

  float lateral_output_sign{1.0F};
  float longitudinal_output_sign{-1.0F};
  float lateral_kp_rate{3.0F};
  float lateral_kd_rate{0.5F};
  float longitudinal_kp_rate{3.0F};
  float longitudinal_kd_rate{0.5F};
  float x_deadband{0.020F};
  float y_deadband{0.020F};
  float x_deadband_px{30.0F};
  float y_deadband_px{30.0F};
  float aim_offset_x_px{0.0F};
  float aim_offset_y_px{0.0F};
  float pixel_dot_lpf_tau_s{0.25F};

  ImageLeadParams image_lead{};
  TrackingStartSmoothingParams tracking_start_smoothing{};

  float rate_lpf_tau_s{0.05F};
  float max_jerk_rad_s2{20.0F};
  float max_roll_rate_rad_s{2.0F};
  float max_pitch_rate_rad_s{2.0F};
  bool rate_shaper_diag_log{false};

  TiltCapParams tilt_cap{};
  ForceLevelParams force_level{};
  WaitingParams waiting{};

  RhoScaleParams rho_scale{};
  ApproachDriveParams approach_drive{};
  SpeedGovernorParams speed_governor{};
  PreclimbParams preclimb{};
  AscentDampingParams ascent_damping{};
  TrackingDeadbandPriorityParams tracking_deadband_priority{};

  FinalApproachParams final_approach{};
  YawParams yaw{};
  TargetLossParams target_loss{};
  ThrustParams thrust{};
  EvaluationParams evaluation{};
  TiltGuardParams tilt_guard{};
  DirectionalDiveParams directional_dive{};

  bool require_armed_to_command{true};
  bool dry_run{false};
  uint32_t armed_latch_ttl_ms{750};
  uint8_t armed_disarm_debounce_count{2};

  void clamp();
};

struct StrikeInputs {
  circle::types::FrameDetection detection{};
  circle::types::FcState vehicle{};
  circle::types::TimestampNs now_ns{0};
  bool mode_active{false};
};

struct FinalApproachCommitSnapshot {
  bool valid{false};
  bool blind_terminal{false};
  circle::types::TimestampNs command_stamp_ns{0};
  circle::types::TimestampNs detection_stamp_ns{0};
  float roll_rate_sp_rad_s{0.0F};
  float pitch_rate_sp_rad_s{0.0F};
  float yaw_rate_sp_rad_s{0.0F};
  float thrust_z{0.0F};
  float bbox_area_ratio{0.0F};
  float margin_x_px{0.0F};
  float margin_y_px{0.0F};
  float align_error_x_px{0.0F};
  float align_error_y_px{0.0F};
  float ex{0.0F};
  float ey{0.0F};
  float ex_dot{0.0F};
  float ey_dot{0.0F};
  bool recent_centered_terminal{false};
};

struct StrikeOutputs {
  circle::types::RateCommand rates{};
  circle::types::SafetyContext safety{};
  StrikeState state{StrikeState::WaitingTarget};
  bool has_valid_target{false};
  float image_ex{0.0F};
  float image_ey{0.0F};
  StrikeTelemetry telemetry{};
};

}  // namespace circle::strike
