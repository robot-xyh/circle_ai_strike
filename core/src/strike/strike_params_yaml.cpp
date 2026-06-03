#include "circle/strike/strike_params_yaml.hpp"

#include <string>

#include <yaml-cpp/yaml.h>

namespace circle::strike {

namespace {

template <typename T>
void assignIfPresent(const YAML::Node& node, const char* key, T& out) {
  if (node[key]) {
    out = node[key].as<T>();
  }
}

void loadFilterParams(const YAML::Node& node, vision::DetectionFilterParams& out) {
  assignIfPresent(node, "min_score", out.min_score);
  assignIfPresent(node, "min_bbox_area", out.min_bbox_area);
  assignIfPresent(node, "max_bbox_aspect_ratio", out.max_bbox_aspect_ratio);
  assignIfPresent(node, "target_class_name", out.target_class_name);
  if (node["target_class_names"]) {
    out.target_class_names = node["target_class_names"].as<std::vector<std::string>>();
  }
  if (node["temporal_gating"]) {
    const YAML::Node tg = node["temporal_gating"];
    assignIfPresent(tg, "enabled", out.temporal_gating_enabled);
    assignIfPresent(tg, "gate_radius_px", out.gate_radius_px);
    assignIfPresent(tg, "reacquire_area_ratio", out.reacquire_area_ratio);
  }
}

void loadDkfParams(const YAML::Node& node, DelayedPixelKalman::Params& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "process_accel_noise", out.process_accel_noise);
  assignIfPresent(node, "meas_noise_px", out.meas_noise_px);
  assignIfPresent(node, "predict_extra_delay_s", out.predict_extra_delay_s);
  assignIfPresent(node, "max_cov_trace", out.max_cov_trace);
}

GuidanceMode parseGuidanceMode(const std::string& value) {
  if (value == "paper_png" || value == "png") {
    return GuidanceMode::PaperPng;
  }
  return GuidanceMode::LegacyPd;
}

void loadVisualPngGuidanceParams(const YAML::Node& node,
                                 VisualPngGuidanceParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "nav_ratio_x", out.nav_ratio_x);
  assignIfPresent(node, "nav_ratio_y", out.nav_ratio_y);
  assignIfPresent(node, "fov_trim_kp_rate", out.fov_trim_kp_rate);
  assignIfPresent(node, "fov_trim_kd_rate", out.fov_trim_kd_rate);
  assignIfPresent(node, "derotate_body_rates", out.derotate_body_rates);
  assignIfPresent(node, "derotate_pitch_to_x_gain", out.derotate_pitch_to_x_gain);
  assignIfPresent(node, "derotate_roll_to_y_gain", out.derotate_roll_to_y_gain);
  assignIfPresent(node, "residual_rate_limit_rad_s", out.residual_rate_limit_rad_s);
  assignIfPresent(node, "closure_base_scale", out.closure_base_scale);
  assignIfPresent(node, "closure_rho_dot_gain", out.closure_rho_dot_gain);
  assignIfPresent(node, "closure_area_gain", out.closure_area_gain);
  assignIfPresent(node, "closure_min_scale", out.closure_min_scale);
  assignIfPresent(node, "closure_max_scale", out.closure_max_scale);
  assignIfPresent(node, "max_feedforward_rad_s", out.max_feedforward_rad_s);
  assignIfPresent(node, "blend", out.blend);
}

void loadRhoRateWindowParams(const YAML::Node& node,
                             RhoRateWindowParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "window_samples", out.window_samples);
  assignIfPresent(node, "lpf_tau_s", out.lpf_tau_s);
}

void loadTrackerFallbackParams(const YAML::Node& node, TrackerFallbackParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "after_s", out.after_s);
  assignIfPresent(node, "max_s", out.max_s);
  assignIfPresent(node, "max_cov_trace", out.max_cov_trace);
  assignIfPresent(node, "min_score", out.min_score);
}

void loadConfidenceParams(const YAML::Node& node, StrikeConfidenceParams& out) {
  assignIfPresent(node, "gate_enable", out.gate_enable);
  assignIfPresent(node, "min_score", out.min_score);
  assignIfPresent(node, "max_cov_trace", out.max_cov_trace);
}

void loadImageLeadParams(const YAML::Node& node, ImageLeadParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "time_s", out.time_s);
  assignIfPresent(node, "max_px", out.max_px);
}

void loadTrackingStartSmoothingParams(const YAML::Node& node, TrackingStartSmoothingParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "smoothing_s", out.smoothing_s);
  assignIfPresent(node, "kp_scale_initial", out.kp_scale_initial);
  assignIfPresent(node, "lead_scale_initial", out.lead_scale_initial);
  assignIfPresent(node, "kd_scale_initial", out.kd_scale_initial);
}

void loadRhoScaleParams(const YAML::Node& node, RhoScaleParams& out) {
  assignIfPresent(node, "desired_bbox_area_px", out.desired_bbox_area_px);
  assignIfPresent(node, "scale_near", out.scale_near);
  assignIfPresent(node, "scale_far", out.scale_far);
  assignIfPresent(node, "near_ratio", out.near_ratio);
  assignIfPresent(node, "far_ratio", out.far_ratio);
}

void loadApproachDriveParams(const YAML::Node& node, ApproachDriveParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "e_rho_deadband", out.e_rho_deadband);
  assignIfPresent(node, "pitch_rate_gain", out.pitch_rate_gain);
  assignIfPresent(node, "pitch_rate_max_rad_s", out.pitch_rate_max_rad_s);
  assignIfPresent(node, "pitch_output_sign", out.pitch_output_sign);
  assignIfPresent(node, "fov_gate_enable", out.fov_gate_enable);
  assignIfPresent(node, "fov_gate_high_error_px",
                  out.fov_gate_high_error_px);
  assignIfPresent(node, "fov_gate_release_error_px",
                  out.fov_gate_release_error_px);
  assignIfPresent(node, "fov_gate_min_scale", out.fov_gate_min_scale);
}

void loadSpeedGovernorParams(const YAML::Node& node, SpeedGovernorParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "start_m_s", out.start_m_s);
  assignIfPresent(node, "full_m_s", out.full_m_s);
  assignIfPresent(node, "min_image_scale", out.min_image_scale);
  assignIfPresent(node, "level_kp", out.level_kp);
  assignIfPresent(node, "level_max_rad_s", out.level_max_rad_s);
  assignIfPresent(node, "fa_roll_level_blend_max", out.fa_roll_level_blend_max);
  assignIfPresent(node, "fa_pitch_level_blend_max", out.fa_pitch_level_blend_max);
  assignIfPresent(node, "fa_gate_enable", out.fa_gate_enable);
  assignIfPresent(node, "fa_max_vxy_m_s", out.fa_max_vxy_m_s);
}

void loadPreclimbParams(const YAML::Node& node, PreclimbParams& out) {
  assignIfPresent(node, "xy_gate_enable", out.xy_gate_enable);
  assignIfPresent(node, "xy_error_gate_x_px", out.xy_error_gate_x_px);
  assignIfPresent(node, "xy_error_gate_y_px", out.xy_error_gate_y_px);
  assignIfPresent(node, "xy_rate_gate_x_px_s", out.xy_rate_gate_x_px_s);
  assignIfPresent(node, "xy_rate_gate_y_px_s", out.xy_rate_gate_y_px_s);
  assignIfPresent(node, "clear_margin_x_px", out.clear_margin_x_px);
  assignIfPresent(node, "clear_margin_y_px", out.clear_margin_y_px);
  assignIfPresent(node, "level_gate_enable", out.level_gate_enable);
  assignIfPresent(node, "level_max_roll_rad", out.level_max_roll_rad);
  assignIfPresent(node, "level_max_pitch_rad", out.level_max_pitch_rad);
  assignIfPresent(node, "xy_hold_s", out.xy_hold_s);
  assignIfPresent(node, "level_assist_enable", out.level_assist_enable);
  assignIfPresent(node, "level_assist_band_rad", out.level_assist_band_rad);
  assignIfPresent(node, "level_assist_kp", out.level_assist_kp);
  assignIfPresent(node, "level_assist_max_rad_s", out.level_assist_max_rad_s);
  assignIfPresent(node, "safe_hold_enable", out.safe_hold_enable);
  assignIfPresent(node, "safe_hold_level_kp", out.safe_hold_level_kp);
  assignIfPresent(node, "safe_hold_level_max_rad_s", out.safe_hold_level_max_rad_s);
  assignIfPresent(node, "safe_hold_image_rate_scale", out.safe_hold_image_rate_scale);
  assignIfPresent(node, "safe_hold_max_rate_rad_s", out.safe_hold_max_rate_rad_s);
  assignIfPresent(node, "safe_hold_tilt_start_rad", out.safe_hold_tilt_start_rad);
  assignIfPresent(node, "safe_hold_tilt_full_rad", out.safe_hold_tilt_full_rad);
  assignIfPresent(node, "thrust_excess_scale", out.thrust_excess_scale);
  assignIfPresent(node, "min_thrust_scalar", out.min_thrust_scalar);
  assignIfPresent(node, "thrust_hard_cap_enable", out.thrust_hard_cap_enable);
  assignIfPresent(node, "release_slowdown_enable", out.release_slowdown_enable);
  assignIfPresent(node, "release_slowdown_s", out.release_slowdown_s);
  assignIfPresent(node, "release_thrust_excess_scale", out.release_thrust_excess_scale);
  assignIfPresent(node, "release_min_thrust_scalar", out.release_min_thrust_scalar);
  assignIfPresent(node, "release_thrust_hard_cap_enable", out.release_thrust_hard_cap_enable);
}

void loadAscentDampingParams(const YAML::Node& node, AscentDampingParams& out) {
  assignIfPresent(node, "image_velocity_damping_enable", out.image_velocity_damping_enable);
  assignIfPresent(node, "image_velocity_damping_start_px_s", out.image_velocity_damping_start_px_s);
  assignIfPresent(node, "image_velocity_damping_min_scale", out.image_velocity_damping_min_scale);
}

void loadTrackingDeadbandPriorityParams(const YAML::Node& node, TrackingDeadbandPriorityParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "error_start_scale", out.error_start_scale);
  assignIfPresent(node, "error_full_scale", out.error_full_scale);
  assignIfPresent(node, "rate_start_px_s", out.rate_start_px_s);
  assignIfPresent(node, "rate_full_px_s", out.rate_full_px_s);
  assignIfPresent(node, "min_excess_scale", out.min_excess_scale);
  assignIfPresent(node, "hard_cap_enable", out.hard_cap_enable);
}

void loadFAGateParams(const YAML::Node& node, FAGateParams& out) {
  assignIfPresent(node, "boost_enable", out.boost_enable);
  assignIfPresent(node, "area_ratio_enter", out.area_ratio_enter);
  assignIfPresent(node, "area_ratio_exit", out.area_ratio_exit);
  assignIfPresent(node, "altitude_gate_enable", out.altitude_gate_enable);
  assignIfPresent(node, "altitude_gate_max_gap_m", out.altitude_gate_max_gap_m);
  assignIfPresent(node, "area_quality_gate_enable", out.area_quality_gate_enable);
  assignIfPresent(node, "area_quality_error_x_px", out.area_quality_error_x_px);
  assignIfPresent(node, "area_quality_error_y_px", out.area_quality_error_y_px);
  assignIfPresent(node, "area_quality_rate_x_px_s", out.area_quality_rate_x_px_s);
  assignIfPresent(node, "area_quality_rate_y_px_s", out.area_quality_rate_y_px_s);
  assignIfPresent(node, "area_quality_max_tilt_rad", out.area_quality_max_tilt_rad);
  assignIfPresent(node, "tilt_gate_enable", out.tilt_gate_enable);
  assignIfPresent(node, "tilt_gate_min_area_ratio", out.tilt_gate_min_area_ratio);
  assignIfPresent(node, "tilt_gate_start_rad", out.tilt_gate_start_rad);
  assignIfPresent(node, "tilt_gate_error_x_px", out.tilt_gate_error_x_px);
  assignIfPresent(node, "tilt_gate_error_y_px", out.tilt_gate_error_y_px);
  assignIfPresent(node, "tilt_gate_rate_x_px_s", out.tilt_gate_rate_x_px_s);
  assignIfPresent(node, "tilt_gate_rate_y_px_s", out.tilt_gate_rate_y_px_s);
  assignIfPresent(node, "stable_gate_enable", out.stable_gate_enable);
  assignIfPresent(node, "stable_gate_min_area_ratio", out.stable_gate_min_area_ratio);
  assignIfPresent(node, "stable_gate_max_area_ratio", out.stable_gate_max_area_ratio);
  assignIfPresent(node, "stable_gate_error_x_px", out.stable_gate_error_x_px);
  assignIfPresent(node, "stable_gate_error_y_px", out.stable_gate_error_y_px);
  assignIfPresent(node, "stable_gate_rate_x_px_s", out.stable_gate_rate_x_px_s);
  assignIfPresent(node, "stable_gate_rate_y_px_s", out.stable_gate_rate_y_px_s);
  assignIfPresent(node, "stable_gate_max_tilt_rad", out.stable_gate_max_tilt_rad);
  assignIfPresent(node, "stable_gate_hold_s", out.stable_gate_hold_s);
  assignIfPresent(node, "hold_s", out.hold_s);
}

void loadFAScalingParams(const YAML::Node& node, FAScalingParams& out) {
  assignIfPresent(node, "jerk_scale", out.jerk_scale);
  assignIfPresent(node, "roll_rate_scale", out.roll_rate_scale);
  assignIfPresent(node, "kp_scale", out.kp_scale);
  assignIfPresent(node, "kd_scale", out.kd_scale);
  assignIfPresent(node, "pixel_dot_lpf_scale", out.pixel_dot_lpf_scale);
  assignIfPresent(node, "rate_lpf_scale", out.rate_lpf_scale);
  assignIfPresent(node, "deadband_scale", out.deadband_scale);
  assignIfPresent(node, "kp_proximity_gain", out.kp_proximity_gain);
  assignIfPresent(node, "kd_proximity_gain", out.kd_proximity_gain);
  assignIfPresent(node, "proximity_error_norm", out.proximity_error_norm);
  assignIfPresent(node, "pitch_ff_erho_gain", out.pitch_ff_erho_gain);
}

void loadFALevelingParams(const YAML::Node& node, FALevelingParams& out) {
  assignIfPresent(node, "roll_level_start_ratio", out.roll_level_start_ratio);
  assignIfPresent(node, "roll_level_end_ratio", out.roll_level_end_ratio);
  assignIfPresent(node, "roll_level_kp", out.roll_level_kp);
  assignIfPresent(node, "roll_level_max_rad_s", out.roll_level_max_rad_s);
  assignIfPresent(node, "pitch_level_start_ratio", out.pitch_level_start_ratio);
  assignIfPresent(node, "pitch_level_end_ratio", out.pitch_level_end_ratio);
  assignIfPresent(node, "pitch_level_kp", out.pitch_level_kp);
  assignIfPresent(node, "pitch_level_max_rad_s", out.pitch_level_max_rad_s);
}

void loadFAPitchChatterGuardParams(const YAML::Node& node, FAPitchChatterGuardParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "max_area_ratio", out.max_area_ratio);
  assignIfPresent(node, "max_error_y_px", out.max_error_y_px);
  assignIfPresent(node, "max_rate_y_px_s", out.max_rate_y_px_s);
  assignIfPresent(node, "prev_min_rate_rad_s", out.prev_min_rate_rad_s);
  assignIfPresent(node, "max_reversal_rate_rad_s", out.max_reversal_rate_rad_s);
}

void loadFATiltAimCompParams(const YAML::Node& node, FATiltAimCompParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "start_ratio", out.start_ratio);
  assignIfPresent(node, "end_ratio", out.end_ratio);
  assignIfPresent(node, "gain", out.gain);
  assignIfPresent(node, "max_px", out.max_px);
  assignIfPresent(node, "roll_sign", out.roll_sign);
  assignIfPresent(node, "pitch_sign", out.pitch_sign);
}

void loadFAThrustParams(const YAML::Node& node, FAThrustParams& out) {
  assignIfPresent(node, "tilt_slowdown_enable", out.tilt_slowdown_enable);
  assignIfPresent(node, "tilt_slowdown_start_rad", out.tilt_slowdown_start_rad);
  assignIfPresent(node, "tilt_slowdown_end_rad", out.tilt_slowdown_end_rad);
  assignIfPresent(node, "tilt_slowdown_min_scale", out.tilt_slowdown_min_scale);
  assignIfPresent(node, "vertical_drift_slowdown_enable", out.vertical_drift_slowdown_enable);
  assignIfPresent(node, "vertical_drift_start_px_s", out.vertical_drift_start_px_s);
  assignIfPresent(node, "vertical_drift_min_scale", out.vertical_drift_min_scale);
  assignIfPresent(node, "ascent_budget_enable", out.ascent_budget_enable);
  assignIfPresent(node, "ascent_budget_tilt_start_rad", out.ascent_budget_tilt_start_rad);
  assignIfPresent(node, "ascent_budget_tilt_full_rad", out.ascent_budget_tilt_full_rad);
  assignIfPresent(node, "ascent_budget_y_rate_start_px_s", out.ascent_budget_y_rate_start_px_s);
  assignIfPresent(node, "ascent_budget_y_rate_full_px_s", out.ascent_budget_y_rate_full_px_s);
  assignIfPresent(node, "ascent_budget_y_error_start_px", out.ascent_budget_y_error_start_px);
  assignIfPresent(node, "ascent_budget_y_error_full_px", out.ascent_budget_y_error_full_px);
  assignIfPresent(node, "ascent_budget_min_excess_scale", out.ascent_budget_min_excess_scale);
  assignIfPresent(node, "ascent_budget_hard_cap_enable", out.ascent_budget_hard_cap_enable);
  assignIfPresent(node, "min_thrust_scalar", out.min_thrust_scalar);
  assignIfPresent(node, "min_thrust_budget_relax_enable", out.min_thrust_budget_relax_enable);
  assignIfPresent(node, "min_thrust_budget_relax_scale", out.min_thrust_budget_relax_scale);
  assignIfPresent(node, "thrust_taper_enable", out.thrust_taper_enable);
  assignIfPresent(node, "thrust_taper_start_ratio", out.thrust_taper_start_ratio);
  assignIfPresent(node, "thrust_taper_end_ratio", out.thrust_taper_end_ratio);
  assignIfPresent(node, "thrust_taper_min_scale", out.thrust_taper_min_scale);
  assignIfPresent(node, "thrust_taper_area_enable", out.thrust_taper_area_enable);
  assignIfPresent(node, "thrust_taper_edge_enable", out.thrust_taper_edge_enable);
  assignIfPresent(node, "thrust_taper_edge_start_score", out.thrust_taper_edge_start_score);
  assignIfPresent(node, "thrust_taper_edge_full_score", out.thrust_taper_edge_full_score);
  assignIfPresent(node, "thrust_taper_tilt_enable", out.thrust_taper_tilt_enable);
  assignIfPresent(node, "thrust_taper_tilt_start_rad", out.thrust_taper_tilt_start_rad);
  assignIfPresent(node, "thrust_taper_tilt_full_rad", out.thrust_taper_tilt_full_rad);
  assignIfPresent(node, "thrust_hard_cap_enable", out.thrust_hard_cap_enable);
  assignIfPresent(node, "unaligned_slowdown_start_ratio", out.unaligned_slowdown_start_ratio);
  assignIfPresent(node, "unaligned_slowdown_end_ratio", out.unaligned_slowdown_end_ratio);
  assignIfPresent(node, "unaligned_thrust_excess_scale", out.unaligned_thrust_excess_scale);
}

void loadFAEdgeProtectParams(const YAML::Node& node, FAEdgeProtectParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "margin_x_px", out.margin_x_px);
  assignIfPresent(node, "margin_y_px", out.margin_y_px);
  assignIfPresent(node, "predict_s", out.predict_s);
  assignIfPresent(node, "roll_kp_rate", out.roll_kp_rate);
  assignIfPresent(node, "pitch_kp_rate", out.pitch_kp_rate);
  assignIfPresent(node, "pitch_boost_max_rad_s", out.pitch_boost_max_rad_s);
  assignIfPresent(node, "thrust_scale", out.thrust_scale);
}

void loadFABottomPitchGuardParams(const YAML::Node& node, FABottomPitchGuardParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "margin_px", out.margin_px);
  assignIfPresent(node, "error_start_px", out.error_start_px);
  assignIfPresent(node, "error_full_px", out.error_full_px);
  assignIfPresent(node, "level_kp", out.level_kp);
  assignIfPresent(node, "max_rad_s", out.max_rad_s);
}

void loadFACommitParams(const YAML::Node& node, FACommitParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "command_hold_s", out.command_hold_s);
  assignIfPresent(node, "detection_stale_s", out.detection_stale_s);
  assignIfPresent(node, "min_latch_s", out.min_latch_s);
  assignIfPresent(node, "thrust_scalar", out.thrust_scalar);
  assignIfPresent(node, "min_area_ratio", out.min_area_ratio);
  assignIfPresent(node, "terminal_min_area_ratio", out.terminal_min_area_ratio);
  assignIfPresent(node, "terminal_max_error_x_px", out.terminal_max_error_x_px);
  assignIfPresent(node, "terminal_max_error_y_px", out.terminal_max_error_y_px);
  assignIfPresent(node, "stable_bypass_area_enable", out.stable_bypass_area_enable);
  assignIfPresent(node, "start_on_terminal_ready_enable", out.start_on_terminal_ready_enable);
  assignIfPresent(node, "recent_centered_handoff_enable", out.recent_centered_handoff_enable);
  assignIfPresent(node, "recent_centered_handoff_min_area_ratio", out.recent_centered_handoff_min_area_ratio);
  assignIfPresent(node, "recent_centered_handoff_max_age_s", out.recent_centered_handoff_max_age_s);
  assignIfPresent(node, "recent_centered_handoff_trigger_error_x_px", out.recent_centered_handoff_trigger_error_x_px);
  assignIfPresent(node, "recent_centered_handoff_trigger_error_y_px", out.recent_centered_handoff_trigger_error_y_px);
  assignIfPresent(node, "recent_centered_handoff_live_blend", out.recent_centered_handoff_live_blend);
  assignIfPresent(node, "recent_centered_hold_through_s", out.recent_centered_hold_through_s);
  assignIfPresent(node, "thrust_ramp_s", out.thrust_ramp_s);
  assignIfPresent(node, "snapshot_max_area_ratio", out.snapshot_max_area_ratio);
  assignIfPresent(node, "align_gate_enable", out.align_gate_enable);
  assignIfPresent(node, "align_max_error_x_px", out.align_max_error_x_px);
  assignIfPresent(node, "align_max_error_y_px", out.align_max_error_y_px);
  assignIfPresent(node, "align_max_rate_x_px_s", out.align_max_rate_x_px_s);
  assignIfPresent(node, "align_max_rate_y_px_s", out.align_max_rate_y_px_s);
  assignIfPresent(node, "align_hold_s", out.align_hold_s);
  assignIfPresent(node, "tilt_gate_enable", out.tilt_gate_enable);
  assignIfPresent(node, "max_tilt_rad", out.max_tilt_rad);
  assignIfPresent(node, "blind_commit_enable", out.blind_commit_enable);
  assignIfPresent(node, "blind_commit_min_area_ratio", out.blind_commit_min_area_ratio);
  assignIfPresent(node, "blind_commit_edge_margin_x_px", out.blind_commit_edge_margin_x_px);
  assignIfPresent(node, "blind_commit_edge_margin_y_px", out.blind_commit_edge_margin_y_px);
  assignIfPresent(node, "blind_commit_edge_vertical_gate_enable", out.blind_commit_edge_vertical_gate_enable);
  assignIfPresent(node, "blind_commit_edge_horizontal_gate_enable", out.blind_commit_edge_horizontal_gate_enable);
  assignIfPresent(node, "blind_commit_edge_max_error_x_px", out.blind_commit_edge_max_error_x_px);
  assignIfPresent(node, "blind_commit_edge_max_rate_x_px_s", out.blind_commit_edge_max_rate_x_px_s);
  assignIfPresent(node, "blind_commit_edge_max_error_y_px", out.blind_commit_edge_max_error_y_px);
  assignIfPresent(node, "blind_commit_edge_max_rate_y_px_s", out.blind_commit_edge_max_rate_y_px_s);
  assignIfPresent(node, "blind_commit_min_score", out.blind_commit_min_score);
  assignIfPresent(node, "blind_commit_trend_lead_s", out.blind_commit_trend_lead_s);
  assignIfPresent(node, "blind_commit_trend_max_rate_rad_s", out.blind_commit_trend_max_rate_rad_s);
  assignIfPresent(node, "predictive_enable", out.predictive_enable);
  assignIfPresent(node, "predict_lead_s", out.predict_lead_s);
  assignIfPresent(node, "predict_kp_rate", out.predict_kp_rate);
  assignIfPresent(node, "predict_kd_rate", out.predict_kd_rate);
  assignIfPresent(node, "predict_max_rate_rad_s", out.predict_max_rate_rad_s);
  assignIfPresent(node, "predict_blend_s", out.predict_blend_s);
  assignIfPresent(node, "freeze_on_edge_protect", out.freeze_on_edge_protect);
  assignIfPresent(node, "forward_pitch_rate_rad_s", out.forward_pitch_rate_rad_s);
  assignIfPresent(node, "roll_level_kp", out.roll_level_kp);
  assignIfPresent(node, "roll_level_max_rad_s", out.roll_level_max_rad_s);
  assignIfPresent(node, "roll_level_blend", out.roll_level_blend);
  assignIfPresent(node, "pitch_level_kp", out.pitch_level_kp);
  assignIfPresent(node, "pitch_level_max_rad_s", out.pitch_level_max_rad_s);
  assignIfPresent(node, "pitch_level_blend", out.pitch_level_blend);
  assignIfPresent(node, "yaw_lock_enabled", out.yaw_lock_enabled);
  assignIfPresent(node, "min_margin_x_px", out.min_margin_x_px);
  assignIfPresent(node, "min_margin_y_px", out.min_margin_y_px);
}

void loadFAFallbackParams(const YAML::Node& node, FAFallbackParams& out) {
  assignIfPresent(node, "decay_tau_s", out.decay_tau_s);
  assignIfPresent(node, "max_s", out.max_s);
  assignIfPresent(node, "pitch_bias_rad", out.pitch_bias_rad);
}

void loadFinalApproachParams(const YAML::Node& node, FinalApproachParams& out) {
  if (node["gate"]) loadFAGateParams(node["gate"], out.gate);
  if (node["scaling"]) loadFAScalingParams(node["scaling"], out.scaling);
  if (node["leveling"]) loadFALevelingParams(node["leveling"], out.leveling);
  if (node["pitch_chatter_guard"]) loadFAPitchChatterGuardParams(node["pitch_chatter_guard"], out.pitch_chatter_guard);
  if (node["tilt_aim_comp"]) loadFATiltAimCompParams(node["tilt_aim_comp"], out.tilt_aim_comp);
  if (node["thrust"]) loadFAThrustParams(node["thrust"], out.thrust);
  if (node["edge_protect"]) loadFAEdgeProtectParams(node["edge_protect"], out.edge_protect);
  if (node["bottom_pitch_guard"]) loadFABottomPitchGuardParams(node["bottom_pitch_guard"], out.bottom_pitch_guard);
  if (node["commit"]) loadFACommitParams(node["commit"], out.commit);
  if (node["fallback"]) loadFAFallbackParams(node["fallback"], out.fallback);
}

void loadYawParams(const YAML::Node& node, YawParams& out) {
  assignIfPresent(node, "bearing_kp", out.bearing_kp);
  assignIfPresent(node, "hold_gain", out.hold_gain);
  assignIfPresent(node, "rate_min_rad_s", out.rate_min_rad_s);
  assignIfPresent(node, "rate_max_rad_s", out.rate_max_rad_s);
  assignIfPresent(node, "hold_deadband_rad", out.hold_deadband_rad);
  assignIfPresent(node, "track_deadband_rad", out.track_deadband_rad);
  assignIfPresent(node, "rate_lpf_tau_s", out.rate_lpf_tau_s);
  assignIfPresent(node, "lock_enabled", out.lock_enabled);
}

void loadWaitingParams(const YAML::Node& node, WaitingParams& out) {
  assignIfPresent(node, "level_hold_enabled", out.level_hold_enabled);
  assignIfPresent(node, "level_kp", out.level_kp);
  assignIfPresent(node, "level_deadband_rad", out.level_deadband_rad);
  assignIfPresent(node, "altitude_kp", out.altitude_kp);
  assignIfPresent(node, "altitude_kd", out.altitude_kd);
  assignIfPresent(node, "altitude_max_correction", out.altitude_max_correction);
}

void loadForceLevelParams(const YAML::Node& node, ForceLevelParams& out) {
  assignIfPresent(node, "hard_level_kp", out.hard_level_kp);
  assignIfPresent(node, "min_hold_ms", out.min_hold_ms);
}

void loadTiltCapParams(const YAML::Node& node, TiltCapParams& out) {
  assignIfPresent(node, "max_roll_angle_rad", out.max_roll_angle_rad);
  assignIfPresent(node, "max_pitch_angle_rad", out.max_pitch_angle_rad);
  assignIfPresent(node, "softcap_band_rad", out.softcap_band_rad);
  assignIfPresent(node, "hardcap_margin_rad", out.hardcap_margin_rad);
}

void loadThrustParams(const YAML::Node& node, ThrustParams& out) {
  assignIfPresent(node, "enable_tilt_compensation", out.enable_tilt_compensation);
  assignIfPresent(node, "hover_scalar", out.hover_scalar);
  assignIfPresent(node, "constant_scalar", out.constant_scalar);
  assignIfPresent(node, "tracking_ramp_tau_s", out.tracking_ramp_tau_s);
  assignIfPresent(node, "slew_rate_scalar_s", out.slew_rate_scalar_s);
  assignIfPresent(node, "tilt_cos_floor", out.tilt_cos_floor);
  assignIfPresent(node, "scalar_min", out.scalar_min);
  assignIfPresent(node, "scalar_max", out.scalar_max);
}

void loadTargetLossParams(const YAML::Node& node, TargetLossParams& out) {
  assignIfPresent(node, "complete_on_loss_enable", out.complete_on_loss_enable);
  assignIfPresent(node, "loss_complete_s", out.loss_complete_s);
  assignIfPresent(node, "lost_target_rate_decay_tau_s", out.lost_target_rate_decay_tau_s);
  assignIfPresent(node, "complete_altitude_gate_enable", out.complete_altitude_gate_enable);
  assignIfPresent(node, "complete_max_altitude_gap_m", out.complete_max_altitude_gap_m);
}

void loadEvaluationParams(const YAML::Node& node, EvaluationParams& out) {
  assignIfPresent(node, "target_altitude_enable", out.target_altitude_enable);
  assignIfPresent(node, "target_altitude_m", out.target_altitude_m);
  assignIfPresent(node, "success_altitude_gap_m", out.success_altitude_gap_m);
  assignIfPresent(node, "relative_distance_enable", out.relative_distance_enable);
  assignIfPresent(node, "relative_distance_max_age_s", out.relative_distance_max_age_s);
  assignIfPresent(node, "target_origin_offset_x_m", out.target_origin_offset_x_m);
  assignIfPresent(node, "target_origin_offset_y_m", out.target_origin_offset_y_m);
}

void loadTiltGuardParams(const YAML::Node& node, TiltGuardParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "max_tilt_rad", out.max_tilt_rad);
  assignIfPresent(node, "recovery_kp", out.recovery_kp);
}

void loadDirectionalDiveParams(const YAML::Node& node, DirectionalDiveParams& out) {
  assignIfPresent(node, "enable", out.enable);
  assignIfPresent(node, "start_edge_ratio", out.start_edge_ratio);
  assignIfPresent(node, "end_edge_ratio", out.end_edge_ratio);
  assignIfPresent(node, "dive_roll_gain", out.dive_roll_gain);
  assignIfPresent(node, "dive_pitch_gain", out.dive_pitch_gain);
  assignIfPresent(node, "climb_thrust", out.climb_thrust);
  assignIfPresent(node, "descend_thrust", out.descend_thrust);
  assignIfPresent(node, "cruise_thrust", out.cruise_thrust);
  assignIfPresent(node, "lead_time_s", out.lead_time_s);
  assignIfPresent(node, "detection_max_age_s", out.detection_max_age_s);
  assignIfPresent(node, "min_horizontal_boost", out.min_horizontal_boost);
  assignIfPresent(node, "tilt_comp_threshold", out.tilt_comp_threshold);
  assignIfPresent(node, "tilt_comp_gain", out.tilt_comp_gain);
}

}  // namespace

StrikeParams loadStrikeParamsFromYaml(const std::string& path) {
  StrikeParams params;
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node strike = root["strike"] ? root["strike"] : root;

  loadFilterParams(strike, params.filter);
  assignIfPresent(strike, "detection_stale_s", params.detection_stale_s);
  assignIfPresent(strike, "lost_timeout_s", params.lost_timeout_s);
  assignIfPresent(strike, "dkf_enable", params.dkf_enable);
  if (strike["guidance_mode"]) {
    params.guidance_mode =
        parseGuidanceMode(strike["guidance_mode"].as<std::string>());
  }

  if (strike["dkf"]) {
    loadDkfParams(strike["dkf"], params.dkf);
  }

  if (strike["visual_png"]) {
    loadVisualPngGuidanceParams(strike["visual_png"], params.visual_png);
  }
  if (strike["rho_rate_window"]) {
    loadRhoRateWindowParams(strike["rho_rate_window"], params.rho_rate_window);
  }

  if (strike["tracker_fallback"]) {
    loadTrackerFallbackParams(strike["tracker_fallback"], params.tracker_fallback);
  }
  if (strike["confidence"]) {
    loadConfidenceParams(strike["confidence"], params.confidence);
  }
  if (strike["image_lead"]) {
    loadImageLeadParams(strike["image_lead"], params.image_lead);
  }
  if (strike["tracking_start_smoothing"]) {
    loadTrackingStartSmoothingParams(strike["tracking_start_smoothing"], params.tracking_start_smoothing);
  }
  if (strike["rho_scale"]) {
    loadRhoScaleParams(strike["rho_scale"], params.rho_scale);
  }
  if (strike["approach_drive"]) {
    loadApproachDriveParams(strike["approach_drive"], params.approach_drive);
  }
  if (strike["speed_governor"]) {
    loadSpeedGovernorParams(strike["speed_governor"], params.speed_governor);
  }
  if (strike["preclimb"]) {
    loadPreclimbParams(strike["preclimb"], params.preclimb);
  }
  if (strike["ascent_damping"]) {
    loadAscentDampingParams(strike["ascent_damping"], params.ascent_damping);
  }
  if (strike["tracking_deadband_priority"]) {
    loadTrackingDeadbandPriorityParams(strike["tracking_deadband_priority"], params.tracking_deadband_priority);
  }
  if (strike["final_approach"]) {
    loadFinalApproachParams(strike["final_approach"], params.final_approach);
  }
  if (strike["yaw"]) {
    loadYawParams(strike["yaw"], params.yaw);
  }
  if (strike["waiting"]) {
    loadWaitingParams(strike["waiting"], params.waiting);
  }
  if (strike["force_level"]) {
    loadForceLevelParams(strike["force_level"], params.force_level);
  }
  if (strike["tilt_cap"]) {
    loadTiltCapParams(strike["tilt_cap"], params.tilt_cap);
  }
  if (strike["thrust"]) {
    loadThrustParams(strike["thrust"], params.thrust);
  }
  if (strike["target_loss"]) {
    loadTargetLossParams(strike["target_loss"], params.target_loss);
  }
  if (strike["evaluation"]) {
    loadEvaluationParams(strike["evaluation"], params.evaluation);
  }
  if (strike["tilt_guard"]) {
    loadTiltGuardParams(strike["tilt_guard"], params.tilt_guard);
  }
  if (strike["directional_dive"]) {
    loadDirectionalDiveParams(strike["directional_dive"], params.directional_dive);
  }

  // Top-level parameters
  assignIfPresent(strike, "lateral_output_sign", params.lateral_output_sign);
  assignIfPresent(strike, "longitudinal_output_sign", params.longitudinal_output_sign);
  assignIfPresent(strike, "lateral_kp_rate", params.lateral_kp_rate);
  assignIfPresent(strike, "lateral_kd_rate", params.lateral_kd_rate);
  assignIfPresent(strike, "longitudinal_kp_rate", params.longitudinal_kp_rate);
  assignIfPresent(strike, "longitudinal_kd_rate", params.longitudinal_kd_rate);
  assignIfPresent(strike, "x_deadband", params.x_deadband);
  assignIfPresent(strike, "y_deadband", params.y_deadband);
  assignIfPresent(strike, "x_deadband_px", params.x_deadband_px);
  assignIfPresent(strike, "y_deadband_px", params.y_deadband_px);
  assignIfPresent(strike, "aim_offset_x_px", params.aim_offset_x_px);
  assignIfPresent(strike, "aim_offset_y_px", params.aim_offset_y_px);
  assignIfPresent(strike, "pixel_dot_lpf_tau_s", params.pixel_dot_lpf_tau_s);
  assignIfPresent(strike, "rate_lpf_tau_s", params.rate_lpf_tau_s);
  assignIfPresent(strike, "max_jerk_rad_s2", params.max_jerk_rad_s2);
  assignIfPresent(strike, "max_roll_rate_rad_s", params.max_roll_rate_rad_s);
  assignIfPresent(strike, "max_pitch_rate_rad_s", params.max_pitch_rate_rad_s);
  assignIfPresent(strike, "rate_shaper_diag_log", params.rate_shaper_diag_log);
  assignIfPresent(strike, "require_armed_to_command", params.require_armed_to_command);
  assignIfPresent(strike, "dry_run", params.dry_run);
  assignIfPresent(strike, "armed_latch_ttl_ms", params.armed_latch_ttl_ms);
  assignIfPresent(strike, "armed_disarm_debounce_count", params.armed_disarm_debounce_count);

  return params;
}

}  // namespace circle::strike
