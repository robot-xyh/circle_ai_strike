#include "circle/strike_png/strike_png_params_yaml.hpp"

#include <algorithm>

#include <yaml-cpp/yaml.h>

namespace circle::strike_png {

namespace {

template <typename T>
void assignIfPresent(const YAML::Node& node, const char* key, T& out) {
  if (node[key]) {
    out = node[key].as<T>();
  }
}

void assignFloatIfPresent(const YAML::Node& node, const char* key, float& out) {
  if (node[key]) {
    out = static_cast<float>(node[key].as<double>());
  }
}

void loadController(const YAML::Node& n, StrikePngParams& c) {
  assignIfPresent(n, "enable", c.enable);
  assignFloatIfPresent(n, "max_roll_rate_rad_s", c.max_roll_rate_rad_s);
  assignFloatIfPresent(n, "max_pitch_rate_rad_s", c.max_pitch_rate_rad_s);
  assignFloatIfPresent(n, "pixel_dot_lpf_tau_s", c.pixel_dot_lpf_tau_s);
  assignFloatIfPresent(n, "nav_ratio_x", c.nav_ratio_x);
  assignFloatIfPresent(n, "nav_ratio_y", c.nav_ratio_y);
  assignIfPresent(n, "derotate_body_rates", c.derotate_body_rates);
  assignFloatIfPresent(n, "derotate_pitch_to_x_gain", c.derotate_pitch_to_x_gain);
  assignFloatIfPresent(n, "derotate_roll_to_y_gain", c.derotate_roll_to_y_gain);
  assignFloatIfPresent(n, "residual_rate_limit_rad_s", c.residual_rate_limit_rad_s);
  assignFloatIfPresent(n, "closure_base_scale", c.closure_base_scale);
  assignFloatIfPresent(n, "closure_area_gain", c.closure_area_gain);
  assignFloatIfPresent(n, "max_feedforward_rad_s", c.max_feedforward_rad_s);
  assignFloatIfPresent(n, "fov_trim_kp_rate", c.fov_trim_kp_rate);
  assignFloatIfPresent(n, "fov_trim_fade_area_ratio_start",
                       c.fov_trim_fade_area_ratio_start);
  assignFloatIfPresent(n, "fov_trim_fade_area_ratio_full",
                       c.fov_trim_fade_area_ratio_full);
  assignFloatIfPresent(n, "vertical_aim_ey", c.vertical_aim_ey);
  assignFloatIfPresent(n, "terminal_tilt_aim_area_ratio_start",
                       c.terminal_tilt_aim_area_ratio_start);
  assignFloatIfPresent(n, "terminal_tilt_aim_area_ratio_full",
                       c.terminal_tilt_aim_area_ratio_full);
  assignFloatIfPresent(n, "terminal_tilt_aim_roll_gain",
                       c.terminal_tilt_aim_roll_gain);
  assignFloatIfPresent(n, "terminal_tilt_aim_pitch_gain",
                       c.terminal_tilt_aim_pitch_gain);
  assignFloatIfPresent(n, "terminal_tilt_aim_max_offset_norm",
                       c.terminal_tilt_aim_max_offset_norm);
  assignFloatIfPresent(n, "los_rate_hold_tau_s", c.los_rate_hold_tau_s);
  assignIfPresent(n, "visual_prediction_enable", c.visual_prediction_enable);
  assignFloatIfPresent(n, "visual_prediction_max_age_s",
                       c.visual_prediction_max_age_s);
  assignFloatIfPresent(n, "visual_prediction_max_offset_norm",
                       c.visual_prediction_max_offset_norm);
  assignIfPresent(n, "edge_guard_enable", c.edge_guard_enable);
  assignFloatIfPresent(n, "edge_guard_start_norm", c.edge_guard_start_norm);
  assignFloatIfPresent(n, "edge_guard_full_norm", c.edge_guard_full_norm);
  assignFloatIfPresent(n, "edge_guard_kp_rate", c.edge_guard_kp_rate);
  assignFloatIfPresent(n, "edge_guard_min_rate_rad_s", c.edge_guard_min_rate_rad_s);
  assignFloatIfPresent(n, "edge_guard_max_rate_rad_s", c.edge_guard_max_rate_rad_s);
  assignFloatIfPresent(n, "edge_guard_pitch_scale", c.edge_guard_pitch_scale);
  assignIfPresent(n, "pursuit_fallback_enable", c.pursuit_fallback_enable);
  assignFloatIfPresent(n, "pursuit_fallback_kp_rate", c.pursuit_fallback_kp_rate);
  assignFloatIfPresent(n, "pursuit_fallback_start_norm", c.pursuit_fallback_start_norm);
  assignFloatIfPresent(n, "pursuit_fallback_full_norm", c.pursuit_fallback_full_norm);
  assignFloatIfPresent(n, "pursuit_fallback_min_rate_rad_s",
                       c.pursuit_fallback_min_rate_rad_s);
  assignFloatIfPresent(n, "pursuit_fallback_max_rate_rad_s",
                       c.pursuit_fallback_max_rate_rad_s);
  assignFloatIfPresent(n, "pursuit_fallback_png_weak_rate_rad_s",
                       c.pursuit_fallback_png_weak_rate_rad_s);
  assignIfPresent(n, "terminal_stale_lateral_trim_enable",
                  c.terminal_stale_lateral_trim_enable);
  assignFloatIfPresent(n, "terminal_stale_lateral_trim_area_ratio_start",
                       c.terminal_stale_lateral_trim_area_ratio_start);
  assignFloatIfPresent(n, "terminal_stale_lateral_trim_area_ratio_full",
                       c.terminal_stale_lateral_trim_area_ratio_full);
  assignFloatIfPresent(n, "terminal_stale_lateral_trim_stale_s_start",
                       c.terminal_stale_lateral_trim_stale_s_start);
  assignFloatIfPresent(n, "terminal_stale_lateral_trim_stale_s_full",
                       c.terminal_stale_lateral_trim_stale_s_full);
  assignFloatIfPresent(n, "terminal_stale_lateral_trim_kp_rate",
                       c.terminal_stale_lateral_trim_kp_rate);
  assignFloatIfPresent(n, "terminal_stale_lateral_trim_max_rate_rad_s",
                       c.terminal_stale_lateral_trim_max_rate_rad_s);
  assignIfPresent(n, "terminal_intercept_enable", c.terminal_intercept_enable);
  assignFloatIfPresent(n, "terminal_intercept_area_ratio_start",
                       c.terminal_intercept_area_ratio_start);
  assignFloatIfPresent(n, "terminal_intercept_area_ratio_full",
                       c.terminal_intercept_area_ratio_full);
  assignFloatIfPresent(n, "terminal_intercept_lead_s", c.terminal_intercept_lead_s);
  assignFloatIfPresent(n, "terminal_intercept_kp_rate", c.terminal_intercept_kp_rate);
  assignFloatIfPresent(n, "terminal_intercept_max_rate_rad_s",
                       c.terminal_intercept_max_rate_rad_s);
  assignIfPresent(n, "terminal_crossing_enable", c.terminal_crossing_enable);
  assignFloatIfPresent(n, "terminal_crossing_area_ratio_start",
                       c.terminal_crossing_area_ratio_start);
  assignFloatIfPresent(n, "terminal_crossing_area_ratio_full",
                       c.terminal_crossing_area_ratio_full);
  assignFloatIfPresent(n, "terminal_crossing_rate_start_norm_s",
                       c.terminal_crossing_rate_start_norm_s);
  assignFloatIfPresent(n, "terminal_crossing_rate_full_norm_s",
                       c.terminal_crossing_rate_full_norm_s);
  assignFloatIfPresent(n, "terminal_crossing_kd_rate", c.terminal_crossing_kd_rate);
  assignFloatIfPresent(n, "terminal_crossing_max_rate_rad_s",
                       c.terminal_crossing_max_rate_rad_s);
  assignIfPresent(n, "terminal_forward_speed_guard_enable",
                  c.terminal_forward_speed_guard_enable);
  assignFloatIfPresent(n, "terminal_forward_speed_guard_area_ratio_start",
                       c.terminal_forward_speed_guard_area_ratio_start);
  assignFloatIfPresent(n, "terminal_forward_speed_guard_area_ratio_full",
                       c.terminal_forward_speed_guard_area_ratio_full);
  assignFloatIfPresent(n, "terminal_forward_speed_guard_start_m_s",
                       c.terminal_forward_speed_guard_start_m_s);
  assignFloatIfPresent(n, "terminal_forward_speed_guard_full_m_s",
                       c.terminal_forward_speed_guard_full_m_s);
  assignFloatIfPresent(n, "terminal_forward_speed_guard_min_positive_pitch_scale",
                       c.terminal_forward_speed_guard_min_positive_pitch_scale);
  assignFloatIfPresent(n, "lateral_output_sign", c.lateral_output_sign);
  assignFloatIfPresent(n, "longitudinal_output_sign", c.longitudinal_output_sign);
  if (n["tilt_cap"]) {
    const YAML::Node t = n["tilt_cap"];
    assignIfPresent(t, "enable", c.tilt_cap.enable);
    assignFloatIfPresent(t, "max_roll_angle_deg", c.tilt_cap.max_roll_angle_deg);
    assignFloatIfPresent(t, "max_pitch_angle_deg", c.tilt_cap.max_pitch_angle_deg);
    assignFloatIfPresent(t, "softcap_band_deg", c.tilt_cap.softcap_band_deg);
    assignFloatIfPresent(t, "hardcap_margin_deg", c.tilt_cap.hardcap_margin_deg);
    assignFloatIfPresent(t, "hardcap_level_kp", c.tilt_cap.hardcap_level_kp);
    assignFloatIfPresent(t, "hardcap_max_level_rate_deg_s",
                         c.tilt_cap.hardcap_max_level_rate_deg_s);
    assignFloatIfPresent(t, "out_lpf_tau_s", c.tilt_cap.out_lpf_tau_s);
    assignFloatIfPresent(t, "out_max_jerk_deg_s2", c.tilt_cap.out_max_jerk_deg_s2);
  }
}

void clampNodeParams(StrikePngNodeParams& p) {
  p.hover_thrust_z = std::clamp(p.hover_thrust_z, 0.0F, 0.99F);
  p.strike_thrust_z = std::clamp(p.strike_thrust_z, 0.0F, 0.99F);
  p.target_lost_hold_delay_s = std::clamp(p.target_lost_hold_delay_s, 0.0, 5.0);
  p.entry_handoff.duration_s = std::clamp(p.entry_handoff.duration_s, 0.0F, 5.0F);
  p.entry_handoff.initial_thrust_z =
      std::clamp(p.entry_handoff.initial_thrust_z, 0.0F, 0.99F);
  p.controller.lateral_output_sign =
      std::clamp(p.controller.lateral_output_sign, -1.0F, 1.0F);
  p.controller.longitudinal_output_sign =
      std::clamp(p.controller.longitudinal_output_sign, -1.0F, 1.0F);
  // Mirror the PX4 executor clamps for the terminal-aim / prediction params so
  // BF and PX4 stay behaviorally aligned.
  p.controller.vertical_aim_ey =
      std::clamp(p.controller.vertical_aim_ey, -1.0F, 1.0F);
  p.controller.visual_prediction_max_age_s =
      std::clamp(p.controller.visual_prediction_max_age_s, 0.0F, 1.0F);
  p.controller.visual_prediction_max_offset_norm =
      std::clamp(p.controller.visual_prediction_max_offset_norm, 0.0F, 1.0F);
  p.controller.fov_trim_fade_area_ratio_start =
      std::clamp(p.controller.fov_trim_fade_area_ratio_start, 0.0F, 1.0F);
  p.controller.fov_trim_fade_area_ratio_full = std::clamp(
      std::max(p.controller.fov_trim_fade_area_ratio_full,
               p.controller.fov_trim_fade_area_ratio_start + 1.0e-6F),
      1.0e-6F, 1.0F);
  p.controller.terminal_tilt_aim_area_ratio_start =
      std::clamp(p.controller.terminal_tilt_aim_area_ratio_start, 0.0F, 1.0F);
  p.controller.terminal_tilt_aim_area_ratio_full = std::clamp(
      std::max(p.controller.terminal_tilt_aim_area_ratio_full,
               p.controller.terminal_tilt_aim_area_ratio_start + 1.0e-6F),
      1.0e-6F, 1.0F);
  p.controller.terminal_tilt_aim_roll_gain =
      std::clamp(p.controller.terminal_tilt_aim_roll_gain, -2.0F, 2.0F);
  p.controller.terminal_tilt_aim_pitch_gain =
      std::clamp(p.controller.terminal_tilt_aim_pitch_gain, -2.0F, 2.0F);
  p.controller.terminal_tilt_aim_max_offset_norm =
      std::clamp(p.controller.terminal_tilt_aim_max_offset_norm, 0.0F, 1.0F);
}

}  // namespace

StrikePngNodeParams loadStrikePngParamsFromYaml(const std::string& path) {
  StrikePngNodeParams p;
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node n = root["strike_png"] ? root["strike_png"] : root;
  if (!n) {
    return p;
  }
  assignIfPresent(n, "dry_run", p.dry_run);
  assignIfPresent(n, "require_armed_to_command", p.require_armed_to_command);
  assignIfPresent(n, "target_class_name", p.target_class_name);
  assignFloatIfPresent(n, "min_score", p.min_score);
  assignIfPresent(n, "detection_stale_s", p.detection_stale_s);
  assignFloatIfPresent(n, "hover_thrust_z", p.hover_thrust_z);
  assignFloatIfPresent(n, "strike_thrust_z", p.strike_thrust_z);
  assignIfPresent(n, "target_lost_hold_enable", p.target_lost_hold_enable);
  assignIfPresent(n, "target_lost_hold_delay_s", p.target_lost_hold_delay_s);
  assignIfPresent(n, "entry_smooth_enable", p.entry_handoff.enable);
  assignFloatIfPresent(n, "entry_smooth_duration_s", p.entry_handoff.duration_s);
  assignFloatIfPresent(n, "entry_smooth_initial_thrust_z",
                       p.entry_handoff.initial_thrust_z);
  loadController(n, p.controller);
  clampNodeParams(p);
  return p;
}

}  // namespace circle::strike_png
