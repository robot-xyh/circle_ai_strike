#include "circle/strike/strike_params.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace circle::strike {

static constexpr float kAbsMaxRateRadS = 6.9813F;
static constexpr float kAbsMinThrustScalar = 0.0F;
static constexpr float kAbsMaxThrustScalar = 1.0F;

void StrikeParams::clamp() {
  detection_stale_s = std::max(0.02, detection_stale_s);
  lost_timeout_s = std::max(detection_stale_s, lost_timeout_s);

  filter.min_score = std::clamp(filter.min_score, 0.0, 1.0);
  filter.min_bbox_area = std::max(0.0, filter.min_bbox_area);
  filter.max_bbox_aspect_ratio = std::max(1.0, filter.max_bbox_aspect_ratio);

  dkf.process_accel_noise = std::max(0.0F, dkf.process_accel_noise);
  dkf.meas_noise_px = std::clamp(dkf.meas_noise_px, 0.1F, 100.0F);
  dkf.predict_extra_delay_s = std::clamp(dkf.predict_extra_delay_s, 0.0F, 0.5F);
  dkf.max_cov_trace = std::max(1.0e-6F, dkf.max_cov_trace);

  visual_png.nav_ratio_x = std::clamp(visual_png.nav_ratio_x, 0.0F, 8.0F);
  visual_png.nav_ratio_y = std::clamp(visual_png.nav_ratio_y, 0.0F, 8.0F);
  visual_png.fov_trim_kp_rate =
      std::clamp(visual_png.fov_trim_kp_rate, 0.0F, 5.0F);
  visual_png.fov_trim_kd_rate =
      std::clamp(visual_png.fov_trim_kd_rate, 0.0F, 5.0F);
  visual_png.derotate_pitch_to_x_gain =
      std::clamp(visual_png.derotate_pitch_to_x_gain, -3.0F, 3.0F);
  visual_png.derotate_roll_to_y_gain =
      std::clamp(visual_png.derotate_roll_to_y_gain, -3.0F, 3.0F);
  visual_png.residual_rate_limit_rad_s =
      std::clamp(visual_png.residual_rate_limit_rad_s, 0.0F, 20.0F);
  visual_png.closure_base_scale =
      std::clamp(visual_png.closure_base_scale, 0.0F, 5.0F);
  visual_png.closure_rho_dot_gain =
      std::clamp(visual_png.closure_rho_dot_gain, 0.0F, 5.0F);
  visual_png.closure_area_gain =
      std::clamp(visual_png.closure_area_gain, 0.0F, 5.0F);
  visual_png.closure_min_scale =
      std::clamp(visual_png.closure_min_scale, 0.0F, 5.0F);
  visual_png.closure_max_scale =
      std::clamp(visual_png.closure_max_scale,
                 visual_png.closure_min_scale + 1.0e-3F, 10.0F);
  visual_png.max_feedforward_rad_s =
      std::clamp(visual_png.max_feedforward_rad_s, 0.0F,
                 std::max(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  visual_png.blend = std::clamp(visual_png.blend, 0.0F, 1.0F);
  rho_rate_window.window_samples =
      std::clamp(rho_rate_window.window_samples, 2, 32);
  rho_rate_window.lpf_tau_s =
      std::clamp(rho_rate_window.lpf_tau_s, 0.0F, 2.0F);

  tracker_fallback.after_s =
      std::clamp(tracker_fallback.after_s, 0.0F,
                 static_cast<float>(lost_timeout_s));
  tracker_fallback.max_s =
      std::clamp(tracker_fallback.max_s, tracker_fallback.after_s,
                 static_cast<float>(lost_timeout_s));
  tracker_fallback.max_cov_trace =
      std::max(1.0e-6F, tracker_fallback.max_cov_trace);
  tracker_fallback.min_score =
      std::clamp(tracker_fallback.min_score, 0.0F, 1.0F);

  confidence.min_score = std::clamp(confidence.min_score, 0.0F, 1.0F);
  confidence.max_cov_trace = std::max(1.0e-6F, confidence.max_cov_trace);

  x_deadband = std::max(0.0F, x_deadband);
  y_deadband = std::max(0.0F, y_deadband);
  x_deadband_px = std::max(0.0F, x_deadband_px);
  y_deadband_px = std::max(0.0F, y_deadband_px);
  aim_offset_x_px = std::clamp(aim_offset_x_px, -1000.0F, 1000.0F);
  aim_offset_y_px = std::clamp(aim_offset_y_px, -1000.0F, 1000.0F);
  pixel_dot_lpf_tau_s = std::max(0.0F, pixel_dot_lpf_tau_s);
  image_lead.time_s = std::clamp(image_lead.time_s, 0.0F, 0.50F);
  image_lead.max_px = std::clamp(image_lead.max_px, 0.0F, 500.0F);
  tracking_start_smoothing.smoothing_s =
      std::clamp(tracking_start_smoothing.smoothing_s, 0.0F, 5.0F);
  tracking_start_smoothing.kp_scale_initial =
      std::clamp(tracking_start_smoothing.kp_scale_initial, 0.0F, 1.0F);
  tracking_start_smoothing.lead_scale_initial =
      std::clamp(tracking_start_smoothing.lead_scale_initial, 0.0F, 1.0F);
  tracking_start_smoothing.kd_scale_initial =
      std::clamp(tracking_start_smoothing.kd_scale_initial, 0.0F, 1.0F);

  rate_lpf_tau_s = std::max(0.0F, rate_lpf_tau_s);
  max_jerk_rad_s2 = std::max(0.0F, max_jerk_rad_s2);
  max_roll_rate_rad_s = std::clamp(max_roll_rate_rad_s, 0.05F, kAbsMaxRateRadS);
  max_pitch_rate_rad_s =
      std::clamp(max_pitch_rate_rad_s, 0.05F, kAbsMaxRateRadS);

  tilt_cap.max_roll_angle_rad =
      std::clamp(tilt_cap.max_roll_angle_rad, 0.05F, 1.5F);
  tilt_cap.max_pitch_angle_rad =
      std::clamp(tilt_cap.max_pitch_angle_rad, 0.05F, 1.5F);
  tilt_cap.softcap_band_rad = std::max(0.01F, tilt_cap.softcap_band_rad);
  tilt_cap.hardcap_margin_rad = std::max(0.0F, tilt_cap.hardcap_margin_rad);
  force_level.hard_level_kp = std::max(0.0F, force_level.hard_level_kp);
  force_level.min_hold_ms = std::max(0, force_level.min_hold_ms);

  waiting.level_kp = std::max(0.0F, waiting.level_kp);
  waiting.level_deadband_rad = std::max(0.0F, waiting.level_deadband_rad);

  rho_scale.scale_near = std::clamp(rho_scale.scale_near, 0.01F, 5.0F);
  rho_scale.scale_far = std::clamp(rho_scale.scale_far, 0.01F, 5.0F);
  rho_scale.near_ratio = std::max(1.0001F, rho_scale.near_ratio);
  rho_scale.far_ratio = std::clamp(rho_scale.far_ratio, 1.0e-4F, 0.9999F);

  approach_drive.e_rho_deadband =
      std::clamp(approach_drive.e_rho_deadband, 0.0F, 5.0F);
  approach_drive.pitch_rate_gain =
      std::clamp(approach_drive.pitch_rate_gain, 0.0F, 5.0F);
  approach_drive.pitch_rate_max_rad_s =
      std::clamp(approach_drive.pitch_rate_max_rad_s, 0.0F,
                 max_pitch_rate_rad_s);
  approach_drive.pitch_output_sign =
      std::clamp(approach_drive.pitch_output_sign, -1.0F, 1.0F);
  approach_drive.fov_gate_high_error_px =
      std::clamp(approach_drive.fov_gate_high_error_px, 1.0F, 2000.0F);
  approach_drive.fov_gate_release_error_px =
      std::clamp(approach_drive.fov_gate_release_error_px, 0.0F,
                 approach_drive.fov_gate_high_error_px - 1.0e-3F);
  approach_drive.fov_gate_min_scale =
      std::clamp(approach_drive.fov_gate_min_scale, 0.0F, 1.0F);

  speed_governor.start_m_s =
      std::clamp(speed_governor.start_m_s, 0.0F, 50.0F);
  speed_governor.full_m_s =
      std::clamp(speed_governor.full_m_s,
                 speed_governor.start_m_s + 0.1F, 60.0F);
  speed_governor.min_image_scale =
      std::clamp(speed_governor.min_image_scale, 0.0F, 1.0F);
  speed_governor.level_kp =
      std::clamp(speed_governor.level_kp, 0.0F, 20.0F);
  speed_governor.level_max_rad_s =
      std::clamp(speed_governor.level_max_rad_s, 0.0F,
                 std::max(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  speed_governor.fa_roll_level_blend_max =
      std::clamp(speed_governor.fa_roll_level_blend_max, 0.0F, 1.0F);
  speed_governor.fa_pitch_level_blend_max =
      std::clamp(speed_governor.fa_pitch_level_blend_max, 0.0F, 1.0F);
  speed_governor.fa_max_vxy_m_s =
      std::clamp(speed_governor.fa_max_vxy_m_s, 0.0F, 60.0F);

  preclimb.xy_error_gate_x_px =
      std::clamp(preclimb.xy_error_gate_x_px, 1.0F, 10000.0F);
  preclimb.xy_error_gate_y_px =
      std::clamp(preclimb.xy_error_gate_y_px, 1.0F, 10000.0F);
  preclimb.xy_rate_gate_x_px_s =
      std::clamp(preclimb.xy_rate_gate_x_px_s, 0.0F, 10000.0F);
  preclimb.xy_rate_gate_y_px_s =
      std::clamp(preclimb.xy_rate_gate_y_px_s, 0.0F, 10000.0F);
  preclimb.clear_margin_x_px =
      std::clamp(preclimb.clear_margin_x_px, 0.0F, 10000.0F);
  preclimb.clear_margin_y_px =
      std::clamp(preclimb.clear_margin_y_px, 0.0F, 10000.0F);
  preclimb.level_max_roll_rad =
      std::clamp(preclimb.level_max_roll_rad, 0.0F,
                 tilt_cap.max_roll_angle_rad);
  preclimb.level_max_pitch_rad =
      std::clamp(preclimb.level_max_pitch_rad, 0.0F,
                 tilt_cap.max_pitch_angle_rad);
  preclimb.xy_hold_s = std::clamp(preclimb.xy_hold_s, 0.0F, 5.0F);
  preclimb.level_assist_band_rad =
      std::clamp(preclimb.level_assist_band_rad, 0.0F, 1.0F);
  preclimb.level_assist_kp =
      std::clamp(preclimb.level_assist_kp, 0.0F, 20.0F);
  preclimb.level_assist_max_rad_s =
      std::clamp(preclimb.level_assist_max_rad_s, 0.0F,
                 std::max(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  preclimb.safe_hold_level_kp =
      std::clamp(preclimb.safe_hold_level_kp, 0.0F, 20.0F);
  preclimb.safe_hold_level_max_rad_s =
      std::clamp(preclimb.safe_hold_level_max_rad_s, 0.0F,
                 std::max(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  preclimb.safe_hold_image_rate_scale =
      std::clamp(preclimb.safe_hold_image_rate_scale, 0.0F, 1.0F);
  preclimb.safe_hold_max_rate_rad_s =
      std::clamp(preclimb.safe_hold_max_rate_rad_s, 0.0F,
                 std::max(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  preclimb.safe_hold_tilt_start_rad =
      std::clamp(preclimb.safe_hold_tilt_start_rad, 0.0F, 1.5F);
  preclimb.safe_hold_tilt_full_rad =
      std::clamp(preclimb.safe_hold_tilt_full_rad,
                 preclimb.safe_hold_tilt_start_rad, 1.5F);
  preclimb.thrust_excess_scale =
      std::clamp(preclimb.thrust_excess_scale, 0.0F, 1.0F);
  preclimb.min_thrust_scalar =
      std::clamp(preclimb.min_thrust_scalar, 0.0F, 0.99F);
  preclimb.release_slowdown_s =
      std::clamp(preclimb.release_slowdown_s, 0.0F, 10.0F);
  preclimb.release_thrust_excess_scale =
      std::clamp(preclimb.release_thrust_excess_scale, 0.0F, 1.0F);
  preclimb.release_min_thrust_scalar =
      std::clamp(preclimb.release_min_thrust_scalar, 0.0F, 0.99F);

  ascent_damping.image_velocity_damping_start_px_s =
      std::clamp(ascent_damping.image_velocity_damping_start_px_s, 1.0F,
                 10000.0F);
  ascent_damping.image_velocity_damping_min_scale =
      std::clamp(ascent_damping.image_velocity_damping_min_scale, 0.0F, 1.0F);

  tracking_deadband_priority.error_start_scale =
      std::clamp(tracking_deadband_priority.error_start_scale, 0.0F, 100.0F);
  tracking_deadband_priority.error_full_scale =
      std::clamp(tracking_deadband_priority.error_full_scale,
                 tracking_deadband_priority.error_start_scale + 1.0e-3F,
                 100.0F);
  tracking_deadband_priority.rate_start_px_s =
      std::clamp(tracking_deadband_priority.rate_start_px_s, 1.0F, 10000.0F);
  tracking_deadband_priority.rate_full_px_s =
      std::clamp(tracking_deadband_priority.rate_full_px_s,
                 tracking_deadband_priority.rate_start_px_s, 10000.0F);
  tracking_deadband_priority.min_excess_scale =
      std::clamp(tracking_deadband_priority.min_excess_scale, 0.0F, 1.0F);

  auto& fa = final_approach;
  fa.gate.area_ratio_enter =
      std::clamp(fa.gate.area_ratio_enter, 0.0F, 1.0F);
  fa.gate.area_ratio_exit =
      std::clamp(fa.gate.area_ratio_exit, 0.0F, 1.0F);
  if (fa.gate.area_ratio_exit > fa.gate.area_ratio_enter) {
    fa.gate.area_ratio_exit = fa.gate.area_ratio_enter;
  }
  fa.gate.altitude_gate_max_gap_m =
      std::clamp(fa.gate.altitude_gate_max_gap_m, 0.0F, 1000.0F);
  fa.gate.area_quality_error_x_px =
      std::clamp(fa.gate.area_quality_error_x_px, 0.0F, 10000.0F);
  fa.gate.area_quality_error_y_px =
      std::clamp(fa.gate.area_quality_error_y_px, 0.0F, 10000.0F);
  fa.gate.area_quality_rate_x_px_s =
      std::clamp(fa.gate.area_quality_rate_x_px_s, 0.0F, 10000.0F);
  fa.gate.area_quality_rate_y_px_s =
      std::clamp(fa.gate.area_quality_rate_y_px_s, 0.0F, 10000.0F);
  fa.gate.area_quality_max_tilt_rad =
      std::clamp(fa.gate.area_quality_max_tilt_rad, 0.0F, 1.5F);
  fa.gate.tilt_gate_min_area_ratio =
      std::clamp(fa.gate.tilt_gate_min_area_ratio, 0.0F, 1.0F);
  fa.gate.tilt_gate_start_rad =
      std::clamp(fa.gate.tilt_gate_start_rad, 0.0F, 1.5F);
  fa.gate.tilt_gate_error_x_px =
      std::clamp(fa.gate.tilt_gate_error_x_px, 0.0F, 10000.0F);
  fa.gate.tilt_gate_error_y_px =
      std::clamp(fa.gate.tilt_gate_error_y_px, 0.0F, 10000.0F);
  fa.gate.tilt_gate_rate_x_px_s =
      std::clamp(fa.gate.tilt_gate_rate_x_px_s, 0.0F, 10000.0F);
  fa.gate.tilt_gate_rate_y_px_s =
      std::clamp(fa.gate.tilt_gate_rate_y_px_s, 0.0F, 10000.0F);
  fa.gate.stable_gate_min_area_ratio =
      std::clamp(fa.gate.stable_gate_min_area_ratio, 0.0F, 1.0F);
  fa.gate.stable_gate_max_area_ratio =
      std::clamp(fa.gate.stable_gate_max_area_ratio, 0.0F, 1.0F);
  if (fa.gate.stable_gate_max_area_ratio > 0.0F &&
      fa.gate.stable_gate_max_area_ratio <
          fa.gate.stable_gate_min_area_ratio) {
    fa.gate.stable_gate_max_area_ratio = fa.gate.stable_gate_min_area_ratio;
  }
  fa.gate.stable_gate_error_x_px =
      std::clamp(fa.gate.stable_gate_error_x_px, 0.0F, 10000.0F);
  fa.gate.stable_gate_error_y_px =
      std::clamp(fa.gate.stable_gate_error_y_px, 0.0F, 10000.0F);
  fa.gate.stable_gate_rate_x_px_s =
      std::clamp(fa.gate.stable_gate_rate_x_px_s, 0.0F, 10000.0F);
  fa.gate.stable_gate_rate_y_px_s =
      std::clamp(fa.gate.stable_gate_rate_y_px_s, 0.0F, 10000.0F);
  fa.gate.stable_gate_max_tilt_rad =
      std::clamp(fa.gate.stable_gate_max_tilt_rad, 0.0F, 1.5F);
  fa.gate.stable_gate_hold_s =
      std::clamp(fa.gate.stable_gate_hold_s, 0.0F, 10.0F);
  fa.gate.hold_s = std::clamp(fa.gate.hold_s, 0.0F, 6.0F);

  fa.scaling.jerk_scale = std::clamp(fa.scaling.jerk_scale, 1.0F, 5.0F);
  fa.scaling.roll_rate_scale =
      std::clamp(fa.scaling.roll_rate_scale, 0.1F, 4.0F);
  fa.scaling.kp_scale = std::clamp(fa.scaling.kp_scale, 0.1F, 5.0F);
  fa.scaling.kd_scale = std::clamp(fa.scaling.kd_scale, 0.0F, 5.0F);
  fa.scaling.pixel_dot_lpf_scale =
      std::clamp(fa.scaling.pixel_dot_lpf_scale, 0.05F, 1.0F);
  fa.scaling.rate_lpf_scale =
      std::clamp(fa.scaling.rate_lpf_scale, 0.05F, 1.0F);
  fa.scaling.deadband_scale =
      std::clamp(fa.scaling.deadband_scale, 0.0F, 1.0F);
  fa.scaling.kp_proximity_gain =
      std::max(0.0F, fa.scaling.kp_proximity_gain);
  fa.scaling.kd_proximity_gain =
      std::max(0.0F, fa.scaling.kd_proximity_gain);
  fa.scaling.pitch_ff_erho_gain =
      std::max(0.0F, fa.scaling.pitch_ff_erho_gain);

  fa.terminal_predictor.min_area_ratio =
      std::clamp(fa.terminal_predictor.min_area_ratio, 0.0F, 1.0F);
  fa.terminal_predictor.min_closing_rate =
      std::clamp(fa.terminal_predictor.min_closing_rate, 0.0F, 10.0F);
  fa.terminal_predictor.lead_s =
      std::clamp(fa.terminal_predictor.lead_s, 0.0F, 0.50F);
  fa.terminal_predictor.max_lead_px =
      std::clamp(fa.terminal_predictor.max_lead_px, 0.0F, 500.0F);
  fa.terminal_predictor.blend =
      std::clamp(fa.terminal_predictor.blend, 0.0F, 1.0F);

  fa.leveling.roll_level_start_ratio =
      std::clamp(fa.leveling.roll_level_start_ratio, 0.0F, 1.0F);
  fa.leveling.roll_level_end_ratio =
      std::clamp(fa.leveling.roll_level_end_ratio,
                 fa.leveling.roll_level_start_ratio, 1.0F);
  fa.leveling.roll_level_kp =
      std::clamp(fa.leveling.roll_level_kp, 0.0F, 20.0F);
  fa.leveling.roll_level_max_rad_s =
      std::clamp(fa.leveling.roll_level_max_rad_s, 0.0F,
                 max_roll_rate_rad_s);
  fa.leveling.pitch_level_start_ratio =
      std::clamp(fa.leveling.pitch_level_start_ratio, 0.0F, 1.0F);
  fa.leveling.pitch_level_end_ratio =
      std::clamp(fa.leveling.pitch_level_end_ratio,
                 fa.leveling.pitch_level_start_ratio, 1.0F);
  fa.leveling.pitch_level_kp =
      std::clamp(fa.leveling.pitch_level_kp, 0.0F, 20.0F);
  fa.leveling.pitch_level_max_rad_s =
      std::clamp(fa.leveling.pitch_level_max_rad_s, 0.0F,
                 max_pitch_rate_rad_s);

  fa.pitch_chatter_guard.max_area_ratio =
      std::clamp(fa.pitch_chatter_guard.max_area_ratio, 0.0F, 1.0F);
  fa.pitch_chatter_guard.max_error_y_px =
      std::clamp(fa.pitch_chatter_guard.max_error_y_px, 0.0F, 10000.0F);
  fa.pitch_chatter_guard.max_rate_y_px_s =
      std::clamp(fa.pitch_chatter_guard.max_rate_y_px_s, 0.0F, 10000.0F);
  fa.pitch_chatter_guard.prev_min_rate_rad_s =
      std::clamp(fa.pitch_chatter_guard.prev_min_rate_rad_s, 0.0F,
                 max_pitch_rate_rad_s);
  fa.pitch_chatter_guard.max_reversal_rate_rad_s =
      std::clamp(fa.pitch_chatter_guard.max_reversal_rate_rad_s, 0.0F,
                 max_pitch_rate_rad_s);

  fa.tilt_aim_comp.start_ratio =
      std::clamp(fa.tilt_aim_comp.start_ratio, 0.0F, 1.0F);
  fa.tilt_aim_comp.end_ratio =
      std::clamp(fa.tilt_aim_comp.end_ratio,
                 fa.tilt_aim_comp.start_ratio, 1.0F);
  fa.tilt_aim_comp.gain =
      std::clamp(fa.tilt_aim_comp.gain, 0.0F, 3.0F);
  fa.tilt_aim_comp.max_px =
      std::clamp(fa.tilt_aim_comp.max_px, 0.0F, 500.0F);
  fa.tilt_aim_comp.roll_sign =
      std::clamp(fa.tilt_aim_comp.roll_sign, -1.0F, 1.0F);
  fa.tilt_aim_comp.pitch_sign =
      std::clamp(fa.tilt_aim_comp.pitch_sign, -1.0F, 1.0F);

  fa.thrust.tilt_slowdown_start_rad =
      std::clamp(fa.thrust.tilt_slowdown_start_rad, 0.0F, 1.5F);
  fa.thrust.tilt_slowdown_end_rad =
      std::clamp(fa.thrust.tilt_slowdown_end_rad,
                 fa.thrust.tilt_slowdown_start_rad, 1.5F);
  fa.thrust.tilt_slowdown_min_scale =
      std::clamp(fa.thrust.tilt_slowdown_min_scale, 0.0F, 1.0F);
  fa.thrust.vertical_drift_start_px_s =
      std::clamp(fa.thrust.vertical_drift_start_px_s, 1.0F, 10000.0F);
  fa.thrust.vertical_drift_min_scale =
      std::clamp(fa.thrust.vertical_drift_min_scale, 0.0F, 1.0F);
  fa.thrust.ascent_budget_tilt_start_rad =
      std::clamp(fa.thrust.ascent_budget_tilt_start_rad, 0.0F, 1.5F);
  fa.thrust.ascent_budget_tilt_full_rad =
      std::clamp(fa.thrust.ascent_budget_tilt_full_rad,
                 fa.thrust.ascent_budget_tilt_start_rad, 1.5F);
  fa.thrust.ascent_budget_y_rate_start_px_s =
      std::clamp(fa.thrust.ascent_budget_y_rate_start_px_s, 1.0F, 10000.0F);
  fa.thrust.ascent_budget_y_rate_full_px_s =
      std::clamp(fa.thrust.ascent_budget_y_rate_full_px_s,
                 fa.thrust.ascent_budget_y_rate_start_px_s, 10000.0F);
  fa.thrust.ascent_budget_y_error_start_px =
      std::clamp(fa.thrust.ascent_budget_y_error_start_px, 0.0F, 10000.0F);
  fa.thrust.ascent_budget_y_error_full_px =
      std::clamp(fa.thrust.ascent_budget_y_error_full_px,
                 fa.thrust.ascent_budget_y_error_start_px, 10000.0F);
  fa.thrust.ascent_budget_min_excess_scale =
      std::clamp(fa.thrust.ascent_budget_min_excess_scale, 0.0F, 1.0F);
  fa.thrust.min_thrust_scalar =
      std::clamp(fa.thrust.min_thrust_scalar, 0.0F, 0.95F);
  fa.thrust.min_thrust_budget_relax_scale =
      std::clamp(fa.thrust.min_thrust_budget_relax_scale, 0.0F, 1.0F);
  fa.thrust.thrust_taper_start_ratio =
      std::clamp(fa.thrust.thrust_taper_start_ratio, 0.0F, 1.0F);
  fa.thrust.thrust_taper_end_ratio =
      std::clamp(fa.thrust.thrust_taper_end_ratio,
                 fa.thrust.thrust_taper_start_ratio + 1.0e-5F, 1.0F);
  fa.thrust.thrust_taper_min_scale =
      std::clamp(fa.thrust.thrust_taper_min_scale, 0.1F, 1.0F);
  fa.thrust.thrust_taper_edge_start_score =
      std::clamp(fa.thrust.thrust_taper_edge_start_score, 0.0F, 10.0F);
  fa.thrust.thrust_taper_edge_full_score =
      std::clamp(fa.thrust.thrust_taper_edge_full_score,
                 fa.thrust.thrust_taper_edge_start_score + 1.0e-5F, 10.0F);
  fa.thrust.thrust_taper_tilt_start_rad =
      std::clamp(fa.thrust.thrust_taper_tilt_start_rad, 0.0F, 1.5F);
  fa.thrust.thrust_taper_tilt_full_rad =
      std::clamp(fa.thrust.thrust_taper_tilt_full_rad,
                 fa.thrust.thrust_taper_tilt_start_rad + 1.0e-5F, 1.5F);
  fa.thrust.unaligned_slowdown_start_ratio =
      std::clamp(fa.thrust.unaligned_slowdown_start_ratio, 0.0F, 1.0F);
  fa.thrust.unaligned_slowdown_end_ratio =
      std::clamp(fa.thrust.unaligned_slowdown_end_ratio,
                 fa.thrust.unaligned_slowdown_start_ratio, 1.0F);
  fa.thrust.unaligned_thrust_excess_scale =
      std::clamp(fa.thrust.unaligned_thrust_excess_scale, 0.0F, 1.0F);

  fa.edge_protect.margin_x_px =
      std::clamp(fa.edge_protect.margin_x_px, 0.0F, 10000.0F);
  fa.edge_protect.margin_y_px =
      std::clamp(fa.edge_protect.margin_y_px, 0.0F, 10000.0F);
  fa.edge_protect.predict_s =
      std::clamp(fa.edge_protect.predict_s, 0.0F, 1.0F);
  fa.edge_protect.roll_kp_rate =
      std::clamp(fa.edge_protect.roll_kp_rate, 0.0F, 20.0F);
  fa.edge_protect.pitch_kp_rate =
      std::clamp(fa.edge_protect.pitch_kp_rate, 0.0F, 20.0F);
  fa.edge_protect.pitch_boost_max_rad_s =
      std::clamp(fa.edge_protect.pitch_boost_max_rad_s, 0.0F,
                 max_pitch_rate_rad_s);
  fa.edge_protect.thrust_scale =
      std::clamp(fa.edge_protect.thrust_scale, 0.1F, 1.0F);

  fa.bottom_pitch_guard.margin_px =
      std::clamp(fa.bottom_pitch_guard.margin_px, 0.0F, 10000.0F);
  fa.bottom_pitch_guard.error_start_px =
      std::clamp(fa.bottom_pitch_guard.error_start_px, 0.0F, 10000.0F);
  fa.bottom_pitch_guard.error_full_px =
      std::clamp(fa.bottom_pitch_guard.error_full_px,
                 fa.bottom_pitch_guard.error_start_px + 1.0F, 10000.0F);
  fa.bottom_pitch_guard.level_kp =
      std::clamp(fa.bottom_pitch_guard.level_kp, 0.0F, 20.0F);
  fa.bottom_pitch_guard.max_rad_s =
      std::clamp(fa.bottom_pitch_guard.max_rad_s, 0.0F,
                 max_pitch_rate_rad_s);

  fa.commit.command_hold_s =
      std::clamp(fa.commit.command_hold_s, 0.0F, 5.0F);
  fa.commit.min_latch_s =
      std::clamp(fa.commit.min_latch_s, 0.0F, fa.commit.command_hold_s);
  fa.commit.detection_stale_s =
      std::clamp(fa.commit.detection_stale_s, 0.05F, 5.0F);
  fa.commit.thrust_scalar =
      std::clamp(fa.commit.thrust_scalar, 0.05F, 0.95F);
  fa.commit.min_area_ratio =
      std::clamp(fa.commit.min_area_ratio, 0.0F, 1.0F);
  fa.commit.terminal_min_area_ratio =
      std::clamp(fa.commit.terminal_min_area_ratio, 0.0F, 1.0F);
  fa.commit.terminal_max_error_x_px =
      std::clamp(fa.commit.terminal_max_error_x_px, 0.0F, 10000.0F);
  fa.commit.terminal_max_error_y_px =
      std::clamp(fa.commit.terminal_max_error_y_px, 0.0F, 10000.0F);
  fa.commit.recent_centered_handoff_min_area_ratio =
      std::clamp(fa.commit.recent_centered_handoff_min_area_ratio, 0.0F, 1.0F);
  fa.commit.recent_centered_handoff_max_age_s =
      std::clamp(fa.commit.recent_centered_handoff_max_age_s, 0.0F, 5.0F);
  fa.commit.recent_centered_handoff_trigger_error_x_px =
      std::clamp(fa.commit.recent_centered_handoff_trigger_error_x_px, 0.0F, 10000.0F);
  fa.commit.recent_centered_handoff_trigger_error_y_px =
      std::clamp(fa.commit.recent_centered_handoff_trigger_error_y_px, 0.0F, 10000.0F);
  fa.commit.recent_centered_handoff_live_blend =
      std::clamp(fa.commit.recent_centered_handoff_live_blend, 0.0F, 1.0F);
  fa.commit.recent_centered_hold_through_s =
      std::clamp(fa.commit.recent_centered_hold_through_s, 0.0F,
                 fa.commit.command_hold_s);
  fa.commit.thrust_ramp_s =
      std::clamp(fa.commit.thrust_ramp_s, 0.0F, fa.commit.command_hold_s);
  fa.commit.snapshot_max_area_ratio =
      std::clamp(fa.commit.snapshot_max_area_ratio, 0.0F, 1.0F);
  fa.commit.align_max_error_x_px =
      std::clamp(fa.commit.align_max_error_x_px, 0.0F, 10000.0F);
  fa.commit.align_max_error_y_px =
      std::clamp(fa.commit.align_max_error_y_px, 0.0F, 10000.0F);
  fa.commit.align_max_rate_x_px_s =
      std::clamp(fa.commit.align_max_rate_x_px_s, 0.0F, 10000.0F);
  fa.commit.align_max_rate_y_px_s =
      std::clamp(fa.commit.align_max_rate_y_px_s, 0.0F, 10000.0F);
  fa.commit.align_hold_s =
      std::clamp(fa.commit.align_hold_s, 0.0F, 2.0F);
  fa.commit.future_lead_s =
      std::clamp(fa.commit.future_lead_s, 0.0F, 1.0F);
  fa.commit.future_max_error_x_px =
      std::clamp(fa.commit.future_max_error_x_px, 0.0F, 10000.0F);
  fa.commit.future_max_error_y_px =
      std::clamp(fa.commit.future_max_error_y_px, 0.0F, 10000.0F);
  fa.commit.max_tilt_rad =
      std::clamp(fa.commit.max_tilt_rad, 0.0F, 1.5F);
  fa.commit.blind_commit_min_area_ratio =
      std::clamp(fa.commit.blind_commit_min_area_ratio, 0.0F, 1.0F);
  fa.commit.blind_commit_edge_margin_x_px =
      std::clamp(fa.commit.blind_commit_edge_margin_x_px, 0.0F, 10000.0F);
  fa.commit.blind_commit_edge_margin_y_px =
      std::clamp(fa.commit.blind_commit_edge_margin_y_px, 0.0F, 10000.0F);
  fa.commit.blind_commit_edge_max_error_y_px =
      std::clamp(fa.commit.blind_commit_edge_max_error_y_px, 0.0F, 10000.0F);
  fa.commit.blind_commit_edge_max_error_x_px =
      std::clamp(fa.commit.blind_commit_edge_max_error_x_px, 0.0F, 10000.0F);
  fa.commit.blind_commit_edge_max_rate_x_px_s =
      std::clamp(fa.commit.blind_commit_edge_max_rate_x_px_s, 0.0F, 10000.0F);
  fa.commit.blind_commit_edge_max_rate_y_px_s =
      std::clamp(fa.commit.blind_commit_edge_max_rate_y_px_s, 0.0F, 10000.0F);
  fa.commit.blind_commit_min_score =
      std::clamp(fa.commit.blind_commit_min_score, 0.0F, 1.0F);
  fa.commit.blind_commit_trend_lead_s =
      std::clamp(fa.commit.blind_commit_trend_lead_s, 0.0F, 1.0F);
  fa.commit.blind_commit_trend_max_rate_rad_s =
      std::clamp(fa.commit.blind_commit_trend_max_rate_rad_s, 0.0F,
                 std::min(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  fa.commit.predict_lead_s =
      std::clamp(fa.commit.predict_lead_s, 0.0F, 1.0F);
  fa.commit.predict_kp_rate =
      std::clamp(fa.commit.predict_kp_rate, 0.0F, 20.0F);
  fa.commit.predict_kd_rate =
      std::clamp(fa.commit.predict_kd_rate, 0.0F, 10.0F);
  fa.commit.predict_max_rate_rad_s =
      std::clamp(fa.commit.predict_max_rate_rad_s, 0.0F,
                 std::min(max_roll_rate_rad_s, max_pitch_rate_rad_s));
  fa.commit.predict_blend_s =
      std::clamp(fa.commit.predict_blend_s, 0.0F, fa.commit.command_hold_s);
  fa.commit.forward_pitch_rate_rad_s =
      std::clamp(fa.commit.forward_pitch_rate_rad_s, 0.0F,
                 max_pitch_rate_rad_s);
  fa.commit.roll_level_kp =
      std::clamp(fa.commit.roll_level_kp, 0.0F, 20.0F);
  fa.commit.roll_level_max_rad_s =
      std::clamp(fa.commit.roll_level_max_rad_s, 0.0F, max_roll_rate_rad_s);
  fa.commit.roll_level_blend =
      std::clamp(fa.commit.roll_level_blend, 0.0F, 1.0F);
  fa.commit.pitch_level_kp =
      std::clamp(fa.commit.pitch_level_kp, 0.0F, 20.0F);
  fa.commit.pitch_level_max_rad_s =
      std::clamp(fa.commit.pitch_level_max_rad_s, 0.0F,
                 max_pitch_rate_rad_s);
  fa.commit.pitch_level_blend =
      std::clamp(fa.commit.pitch_level_blend, 0.0F, 1.0F);
  fa.commit.min_margin_x_px =
      std::clamp(fa.commit.min_margin_x_px, 0.0F, 10000.0F);
  fa.commit.min_margin_y_px =
      std::clamp(fa.commit.min_margin_y_px, 0.0F, 10000.0F);

  fa.fallback.decay_tau_s = std::max(0.05F, fa.fallback.decay_tau_s);
  fa.fallback.max_s = std::max(0.1F, fa.fallback.max_s);
  fa.fallback.pitch_bias_rad =
      std::clamp(fa.fallback.pitch_bias_rad, -1.0F, 0.0F);

  if (yaw.rate_min_rad_s > yaw.rate_max_rad_s) {
    std::swap(yaw.rate_min_rad_s, yaw.rate_max_rad_s);
  }
  yaw.hold_deadband_rad = std::max(0.0F, yaw.hold_deadband_rad);
  yaw.track_deadband_rad = std::max(0.0F, yaw.track_deadband_rad);
  yaw.rate_lpf_tau_s = std::max(0.0F, yaw.rate_lpf_tau_s);

  target_loss.loss_complete_s =
      std::clamp(target_loss.loss_complete_s,
                 static_cast<float>(detection_stale_s),
                 static_cast<float>(lost_timeout_s));
  target_loss.lost_target_rate_decay_tau_s =
      std::max(0.0F, target_loss.lost_target_rate_decay_tau_s);
  target_loss.complete_max_altitude_gap_m =
      std::clamp(target_loss.complete_max_altitude_gap_m, 0.0F, 1000.0F);

  thrust.scalar_min =
      std::clamp(thrust.scalar_min, kAbsMinThrustScalar, kAbsMaxThrustScalar);
  thrust.scalar_max =
      std::clamp(thrust.scalar_max,
                 thrust.scalar_min + 1.0e-3F, kAbsMaxThrustScalar);
  thrust.tracking_ramp_tau_s =
      std::clamp(thrust.tracking_ramp_tau_s, 0.0F, 2.0F);
  thrust.slew_rate_scalar_s =
      std::clamp(thrust.slew_rate_scalar_s, 0.0F, 5.0F);
  thrust.hover_scalar =
      std::clamp(thrust.hover_scalar, thrust.scalar_min, thrust.scalar_max);
  thrust.constant_scalar =
      std::clamp(thrust.constant_scalar, thrust.scalar_min, thrust.scalar_max);
  waiting.altitude_kp = std::max(0.0F, waiting.altitude_kp);
  waiting.altitude_kd = std::max(0.0F, waiting.altitude_kd);
  waiting.altitude_max_correction =
      std::clamp(waiting.altitude_max_correction, 0.0F, 0.50F);
  thrust.tilt_cos_floor = std::clamp(thrust.tilt_cos_floor, 0.15F, 0.99F);

  armed_latch_ttl_ms = std::clamp<uint32_t>(armed_latch_ttl_ms, 50U, 1500U);
  armed_disarm_debounce_count =
      std::clamp<uint8_t>(armed_disarm_debounce_count, 1U, 50U);

  evaluation.target_altitude_m =
      std::clamp(evaluation.target_altitude_m, 0.0F, 10000.0F);
  evaluation.success_altitude_gap_m =
      std::clamp(evaluation.success_altitude_gap_m, 0.0F, 1000.0F);
  evaluation.relative_distance_max_age_s =
      std::clamp(evaluation.relative_distance_max_age_s, 0.02F, 5.0F);
}

}  // namespace circle::strike
