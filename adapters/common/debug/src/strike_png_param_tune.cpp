#include "circle/debug_common/strike_png_param_tune.hpp"

#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "circle/strike_png/strike_png_params_yaml.hpp"
#endif

namespace circle::debug_common {

namespace {

using circle::strike_png::StrikePngNodeParams;
using circle::strike_png::StrikePngParams;

struct PngDesc {
  const char* key;
  double min_v;
  double max_v;
  double step;
  bool is_bool;
};

constexpr PngDesc kPngDescs[] = {
    // Node-level gating / thrust / loss-hold / entry handoff.
    {"dry_run", 0.0, 1.0, 1.0, true},
    {"require_armed_to_command", 0.0, 1.0, 1.0, true},
    {"min_score", 0.0, 1.0, 0.01, false},
    {"detection_stale_s", 0.05, 2.0, 0.05, false},
    {"hover_thrust_z", 0.0, 0.99, 0.005, false},
    {"strike_thrust_z", 0.0, 0.99, 0.005, false},
    {"target_lost_hold_enable", 0.0, 1.0, 1.0, true},
    {"target_lost_hold_delay_s", 0.0, 5.0, 0.05, false},
    {"derotate_history_enable", 0.0, 1.0, 1.0, true},
    {"camera_exposure_midpoint_offset_ns", -50000000.0, 50000000.0, 1000000.0, false},
    {"fc_serial_latency_ns", 0.0, 50000000.0, 1000000.0, false},
    {"max_derotate_interpolation_gap_s", 0.001, 0.2, 0.001, false},
    {"body_rate_observer_enable", 0.0, 1.0, 1.0, true},
    {"entry_smooth_enable", 0.0, 1.0, 1.0, true},
    {"entry_smooth_duration_s", 0.0, 5.0, 0.05, false},
    {"entry_smooth_initial_thrust_z", 0.0, 0.99, 0.005, false},
    // StrikePngController params.
    {"enable", 0.0, 1.0, 1.0, true},
    {"max_roll_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"max_pitch_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"pixel_dot_lpf_tau_s", 0.0, 0.6, 0.005, false},
    {"dkf_enable", 0.0, 1.0, 1.0, true},
    {"dkf.enable", 0.0, 1.0, 1.0, true},
    {"dkf.process_accel_noise", 0.0, 200.0, 1.0, false},
    {"dkf.meas_noise_px", 0.1, 80.0, 0.5, false},
    {"dkf.predict_extra_delay_s", 0.0, 0.2, 0.005, false},
    {"dkf.max_cov_trace", 0.0001, 10.0, 0.01, false},
    {"nav_ratio_x", 0.0, 8.0, 0.1, false},
    {"nav_ratio_y", 0.0, 8.0, 0.1, false},
    {"derotate_body_rates", 0.0, 1.0, 1.0, true},
    {"derotate_pitch_to_x_gain", -2.0, 2.0, 0.05, false},
    {"derotate_roll_to_y_gain", -2.0, 2.0, 0.05, false},
    {"residual_rate_limit_rad_s", 0.0, 6.9813, 0.05, false},
    {"closure_base_scale", 0.0, 2.0, 0.05, false},
    {"closure_area_gain", 0.0, 2.0, 0.05, false},
    {"max_feedforward_rad_s", 0.0, 6.9813, 0.05, false},
    {"fov_trim_kp_rate", 0.0, 4.0, 0.05, false},
    {"fov_trim_fade_area_ratio_start", 0.0, 1.0, 0.01, false},
    {"fov_trim_fade_area_ratio_full", 0.0, 1.0, 0.01, false},
    {"vertical_aim_ey", -1.0, 1.0, 0.01, false},
    {"terminal_tilt_aim_area_ratio_start", 0.0, 1.0, 0.01, false},
    {"terminal_tilt_aim_area_ratio_full", 0.0, 1.0, 0.01, false},
    {"terminal_tilt_aim_roll_gain", -2.0, 2.0, 0.05, false},
    {"terminal_tilt_aim_pitch_gain", -2.0, 2.0, 0.05, false},
    {"terminal_tilt_aim_max_offset_norm", 0.0, 1.0, 0.01, false},
    {"los_rate_hold_tau_s", 0.0, 2.0, 0.01, false},
    {"visual_prediction_enable", 0.0, 1.0, 1.0, true},
    {"visual_prediction_max_age_s", 0.0, 1.0, 0.01, false},
    {"visual_prediction_max_offset_norm", 0.0, 1.0, 0.01, false},
    {"edge_guard_enable", 0.0, 1.0, 1.0, true},
    {"edge_guard_start_norm", 0.0, 1.0, 0.01, false},
    {"edge_guard_full_norm", 0.0, 1.0, 0.01, false},
    {"edge_guard_kp_rate", 0.0, 4.0, 0.05, false},
    {"edge_guard_min_rate_rad_s", 0.0, 4.0, 0.01, false},
    {"edge_guard_max_rate_rad_s", 0.0, 6.9813, 0.05, false},
    {"edge_guard_pitch_scale", 0.0, 2.0, 0.05, false},
    {"pursuit_fallback_enable", 0.0, 1.0, 1.0, true},
    {"pursuit_fallback_kp_rate", 0.0, 4.0, 0.05, false},
    {"pursuit_fallback_start_norm", 0.0, 1.0, 0.01, false},
    {"pursuit_fallback_full_norm", 0.0, 1.0, 0.01, false},
    {"pursuit_fallback_min_rate_rad_s", 0.0, 4.0, 0.01, false},
    {"pursuit_fallback_max_rate_rad_s", 0.0, 6.9813, 0.05, false},
    {"pursuit_fallback_png_weak_rate_rad_s", 0.0, 4.0, 0.01, false},
    {"terminal_stale_lateral_trim_enable", 0.0, 1.0, 1.0, true},
    {"terminal_stale_lateral_trim_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"terminal_stale_lateral_trim_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"terminal_stale_lateral_trim_stale_s_start", 0.0, 2.0, 0.01, false},
    {"terminal_stale_lateral_trim_stale_s_full", 0.0, 2.0, 0.01, false},
    {"terminal_stale_lateral_trim_kp_rate", 0.0, 10.0, 0.1, false},
    {"terminal_stale_lateral_trim_max_rate_rad_s", 0.0, 4.0, 0.05, false},
    {"terminal_intercept_enable", 0.0, 1.0, 1.0, true},
    {"terminal_intercept_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"terminal_intercept_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"terminal_intercept_lead_s", 0.0, 1.0, 0.01, false},
    {"terminal_intercept_kp_rate", 0.0, 10.0, 0.1, false},
    {"terminal_intercept_max_rate_rad_s", 0.0, 4.0, 0.05, false},
    {"terminal_crossing_enable", 0.0, 1.0, 1.0, true},
    {"terminal_crossing_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"terminal_crossing_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"terminal_crossing_rate_start_norm_s", 0.0, 2.0, 0.01, false},
    {"terminal_crossing_rate_full_norm_s", 0.0, 2.0, 0.01, false},
    {"terminal_crossing_kd_rate", 0.0, 4.0, 0.05, false},
    {"terminal_crossing_max_rate_rad_s", 0.0, 4.0, 0.05, false},
    {"terminal_forward_speed_guard_enable", 0.0, 1.0, 1.0, true},
    {"terminal_forward_speed_guard_area_ratio_start", 0.0, 0.1, 0.001, false},
    {"terminal_forward_speed_guard_area_ratio_full", 0.0, 0.1, 0.001, false},
    {"terminal_forward_speed_guard_start_m_s", 0.0, 80.0, 0.5, false},
    {"terminal_forward_speed_guard_full_m_s", 0.0, 80.0, 0.5, false},
    {"terminal_forward_speed_guard_min_positive_pitch_scale", 0.0, 1.0, 0.05, false},
    {"lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"longitudinal_output_sign", -1.0, 1.0, 1.0, false},
    // Tilt safety envelope (nested sub-struct; dotted keys).
    {"tilt_cap.enable", 0.0, 1.0, 1.0, true},
    {"tilt_cap.max_roll_angle_deg", 0.0, 60.0, 1.0, false},
    {"tilt_cap.max_pitch_angle_deg", 0.0, 60.0, 1.0, false},
    {"tilt_cap.softcap_band_deg", 0.0, 30.0, 1.0, false},
    {"tilt_cap.hardcap_margin_deg", 0.0, 30.0, 1.0, false},
    {"tilt_cap.hardcap_level_kp", 0.0, 10.0, 0.1, false},
    {"tilt_cap.hardcap_max_level_rate_deg_s", 0.0, 400.0, 1.0, false},
    {"tilt_cap.out_lpf_tau_s", 0.0, 0.5, 0.005, false},
    {"tilt_cap.out_max_jerk_deg_s2", 0.0, 4000.0, 10.0, false},
};

const std::unordered_map<std::string, float StrikePngParams::*>& pngFloatMembers() {
  static const std::unordered_map<std::string, float StrikePngParams::*> m = {
      {"max_roll_rate_rad_s", &StrikePngParams::max_roll_rate_rad_s},
      {"max_pitch_rate_rad_s", &StrikePngParams::max_pitch_rate_rad_s},
      {"pixel_dot_lpf_tau_s", &StrikePngParams::pixel_dot_lpf_tau_s},
      {"nav_ratio_x", &StrikePngParams::nav_ratio_x},
      {"nav_ratio_y", &StrikePngParams::nav_ratio_y},
      {"derotate_pitch_to_x_gain", &StrikePngParams::derotate_pitch_to_x_gain},
      {"derotate_roll_to_y_gain", &StrikePngParams::derotate_roll_to_y_gain},
      {"residual_rate_limit_rad_s", &StrikePngParams::residual_rate_limit_rad_s},
      {"closure_base_scale", &StrikePngParams::closure_base_scale},
      {"closure_area_gain", &StrikePngParams::closure_area_gain},
      {"max_feedforward_rad_s", &StrikePngParams::max_feedforward_rad_s},
      {"fov_trim_kp_rate", &StrikePngParams::fov_trim_kp_rate},
      {"fov_trim_fade_area_ratio_start",
       &StrikePngParams::fov_trim_fade_area_ratio_start},
      {"fov_trim_fade_area_ratio_full",
       &StrikePngParams::fov_trim_fade_area_ratio_full},
      {"vertical_aim_ey", &StrikePngParams::vertical_aim_ey},
      {"terminal_tilt_aim_area_ratio_start",
       &StrikePngParams::terminal_tilt_aim_area_ratio_start},
      {"terminal_tilt_aim_area_ratio_full",
       &StrikePngParams::terminal_tilt_aim_area_ratio_full},
      {"terminal_tilt_aim_roll_gain",
       &StrikePngParams::terminal_tilt_aim_roll_gain},
      {"terminal_tilt_aim_pitch_gain",
       &StrikePngParams::terminal_tilt_aim_pitch_gain},
      {"terminal_tilt_aim_max_offset_norm",
       &StrikePngParams::terminal_tilt_aim_max_offset_norm},
      {"los_rate_hold_tau_s", &StrikePngParams::los_rate_hold_tau_s},
      {"visual_prediction_max_age_s",
       &StrikePngParams::visual_prediction_max_age_s},
      {"visual_prediction_max_offset_norm",
       &StrikePngParams::visual_prediction_max_offset_norm},
      {"edge_guard_start_norm", &StrikePngParams::edge_guard_start_norm},
      {"edge_guard_full_norm", &StrikePngParams::edge_guard_full_norm},
      {"edge_guard_kp_rate", &StrikePngParams::edge_guard_kp_rate},
      {"edge_guard_min_rate_rad_s", &StrikePngParams::edge_guard_min_rate_rad_s},
      {"edge_guard_max_rate_rad_s", &StrikePngParams::edge_guard_max_rate_rad_s},
      {"edge_guard_pitch_scale", &StrikePngParams::edge_guard_pitch_scale},
      {"pursuit_fallback_kp_rate", &StrikePngParams::pursuit_fallback_kp_rate},
      {"pursuit_fallback_start_norm", &StrikePngParams::pursuit_fallback_start_norm},
      {"pursuit_fallback_full_norm", &StrikePngParams::pursuit_fallback_full_norm},
      {"pursuit_fallback_min_rate_rad_s",
       &StrikePngParams::pursuit_fallback_min_rate_rad_s},
      {"pursuit_fallback_max_rate_rad_s",
       &StrikePngParams::pursuit_fallback_max_rate_rad_s},
      {"pursuit_fallback_png_weak_rate_rad_s",
       &StrikePngParams::pursuit_fallback_png_weak_rate_rad_s},
      {"terminal_stale_lateral_trim_area_ratio_start",
       &StrikePngParams::terminal_stale_lateral_trim_area_ratio_start},
      {"terminal_stale_lateral_trim_area_ratio_full",
       &StrikePngParams::terminal_stale_lateral_trim_area_ratio_full},
      {"terminal_stale_lateral_trim_stale_s_start",
       &StrikePngParams::terminal_stale_lateral_trim_stale_s_start},
      {"terminal_stale_lateral_trim_stale_s_full",
       &StrikePngParams::terminal_stale_lateral_trim_stale_s_full},
      {"terminal_stale_lateral_trim_kp_rate",
       &StrikePngParams::terminal_stale_lateral_trim_kp_rate},
      {"terminal_stale_lateral_trim_max_rate_rad_s",
       &StrikePngParams::terminal_stale_lateral_trim_max_rate_rad_s},
      {"terminal_intercept_area_ratio_start",
       &StrikePngParams::terminal_intercept_area_ratio_start},
      {"terminal_intercept_area_ratio_full",
       &StrikePngParams::terminal_intercept_area_ratio_full},
      {"terminal_intercept_lead_s", &StrikePngParams::terminal_intercept_lead_s},
      {"terminal_intercept_kp_rate", &StrikePngParams::terminal_intercept_kp_rate},
      {"terminal_intercept_max_rate_rad_s",
       &StrikePngParams::terminal_intercept_max_rate_rad_s},
      {"terminal_crossing_area_ratio_start",
       &StrikePngParams::terminal_crossing_area_ratio_start},
      {"terminal_crossing_area_ratio_full",
       &StrikePngParams::terminal_crossing_area_ratio_full},
      {"terminal_crossing_rate_start_norm_s",
       &StrikePngParams::terminal_crossing_rate_start_norm_s},
      {"terminal_crossing_rate_full_norm_s",
       &StrikePngParams::terminal_crossing_rate_full_norm_s},
      {"terminal_crossing_kd_rate", &StrikePngParams::terminal_crossing_kd_rate},
      {"terminal_crossing_max_rate_rad_s",
       &StrikePngParams::terminal_crossing_max_rate_rad_s},
      {"terminal_forward_speed_guard_area_ratio_start",
       &StrikePngParams::terminal_forward_speed_guard_area_ratio_start},
      {"terminal_forward_speed_guard_area_ratio_full",
       &StrikePngParams::terminal_forward_speed_guard_area_ratio_full},
      {"terminal_forward_speed_guard_start_m_s",
       &StrikePngParams::terminal_forward_speed_guard_start_m_s},
      {"terminal_forward_speed_guard_full_m_s",
       &StrikePngParams::terminal_forward_speed_guard_full_m_s},
      {"terminal_forward_speed_guard_min_positive_pitch_scale",
       &StrikePngParams::terminal_forward_speed_guard_min_positive_pitch_scale},
      {"lateral_output_sign", &StrikePngParams::lateral_output_sign},
      {"longitudinal_output_sign", &StrikePngParams::longitudinal_output_sign},
  };
  return m;
}

const std::unordered_map<std::string, bool StrikePngParams::*>& pngBoolMembers() {
  static const std::unordered_map<std::string, bool StrikePngParams::*> m = {
      {"enable", &StrikePngParams::enable},
      {"visual_prediction_enable", &StrikePngParams::visual_prediction_enable},
      {"derotate_body_rates", &StrikePngParams::derotate_body_rates},
      {"edge_guard_enable", &StrikePngParams::edge_guard_enable},
      {"pursuit_fallback_enable", &StrikePngParams::pursuit_fallback_enable},
      {"terminal_stale_lateral_trim_enable",
       &StrikePngParams::terminal_stale_lateral_trim_enable},
      {"terminal_intercept_enable", &StrikePngParams::terminal_intercept_enable},
      {"terminal_crossing_enable", &StrikePngParams::terminal_crossing_enable},
      {"terminal_forward_speed_guard_enable",
       &StrikePngParams::terminal_forward_speed_guard_enable},
  };
  return m;
}

double pngParamValue(const StrikePngNodeParams& p, const std::string& key) {
  if (key == "dry_run") return p.dry_run ? 1.0 : 0.0;
  if (key == "require_armed_to_command") {
    return p.require_armed_to_command ? 1.0 : 0.0;
  }
  if (key == "min_score") return p.min_score;
  if (key == "detection_stale_s") return p.detection_stale_s;
  if (key == "hover_thrust_z") return p.hover_thrust_z;
  if (key == "strike_thrust_z") return p.strike_thrust_z;
  if (key == "target_lost_hold_enable") return p.target_lost_hold_enable ? 1.0 : 0.0;
  if (key == "target_lost_hold_delay_s") return p.target_lost_hold_delay_s;
  if (key == "derotate_history_enable") return p.derotate_history_enable ? 1.0 : 0.0;
  if (key == "camera_exposure_midpoint_offset_ns") {
    return static_cast<double>(p.camera_exposure_midpoint_offset_ns);
  }
  if (key == "fc_serial_latency_ns") {
    return static_cast<double>(p.fc_serial_latency_ns);
  }
  if (key == "max_derotate_interpolation_gap_s") {
    return p.max_derotate_interpolation_gap_s;
  }
  if (key == "body_rate_observer_enable") {
    return p.body_rate_observer_enable ? 1.0 : 0.0;
  }
  if (key == "entry_smooth_enable") return p.entry_handoff.enable ? 1.0 : 0.0;
  if (key == "entry_smooth_duration_s") return p.entry_handoff.duration_s;
  if (key == "entry_smooth_initial_thrust_z") {
    return p.entry_handoff.initial_thrust_z;
  }
  if (key == "dkf_enable") return p.controller.dkf_enable ? 1.0 : 0.0;
  if (key == "dkf.enable") return p.controller.dkf.enable ? 1.0 : 0.0;
  if (key == "dkf.process_accel_noise") {
    return p.controller.dkf.process_accel_noise;
  }
  if (key == "dkf.meas_noise_px") return p.controller.dkf.meas_noise_px;
  if (key == "dkf.predict_extra_delay_s") {
    return p.controller.dkf.predict_extra_delay_s;
  }
  if (key == "dkf.max_cov_trace") return p.controller.dkf.max_cov_trace;
  if (key == "tilt_cap.enable") return p.controller.tilt_cap.enable ? 1.0 : 0.0;
  if (key == "tilt_cap.max_roll_angle_deg") {
    return p.controller.tilt_cap.max_roll_angle_deg;
  }
  if (key == "tilt_cap.max_pitch_angle_deg") {
    return p.controller.tilt_cap.max_pitch_angle_deg;
  }
  if (key == "tilt_cap.softcap_band_deg") {
    return p.controller.tilt_cap.softcap_band_deg;
  }
  if (key == "tilt_cap.hardcap_margin_deg") {
    return p.controller.tilt_cap.hardcap_margin_deg;
  }
  if (key == "tilt_cap.hardcap_level_kp") {
    return p.controller.tilt_cap.hardcap_level_kp;
  }
  if (key == "tilt_cap.hardcap_max_level_rate_deg_s") {
    return p.controller.tilt_cap.hardcap_max_level_rate_deg_s;
  }
  if (key == "tilt_cap.out_lpf_tau_s") {
    return p.controller.tilt_cap.out_lpf_tau_s;
  }
  if (key == "tilt_cap.out_max_jerk_deg_s2") {
    return p.controller.tilt_cap.out_max_jerk_deg_s2;
  }
  const auto& fm = pngFloatMembers();
  if (auto it = fm.find(key); it != fm.end()) {
    return p.controller.*(it->second);
  }
  const auto& bm = pngBoolMembers();
  if (auto it = bm.find(key); it != bm.end()) {
    return p.controller.*(it->second) ? 1.0 : 0.0;
  }
  return 0.0;
}

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
bool pngSetValue(StrikePngNodeParams& p, const std::string& key, double value) {
  const bool b = value != 0.0;
  if (key == "dry_run") {
    p.dry_run = b;
  } else if (key == "require_armed_to_command") {
    p.require_armed_to_command = b;
  } else if (key == "min_score") {
    p.min_score = static_cast<float>(value);
  } else if (key == "detection_stale_s") {
    p.detection_stale_s = value;
  } else if (key == "hover_thrust_z") {
    p.hover_thrust_z = static_cast<float>(value);
  } else if (key == "strike_thrust_z") {
    p.strike_thrust_z = static_cast<float>(value);
  } else if (key == "target_lost_hold_enable") {
    p.target_lost_hold_enable = b;
  } else if (key == "target_lost_hold_delay_s") {
    p.target_lost_hold_delay_s = value;
  } else if (key == "derotate_history_enable") {
    p.derotate_history_enable = b;
  } else if (key == "camera_exposure_midpoint_offset_ns") {
    p.camera_exposure_midpoint_offset_ns = static_cast<int64_t>(value);
  } else if (key == "fc_serial_latency_ns") {
    p.fc_serial_latency_ns = static_cast<int64_t>(value);
  } else if (key == "max_derotate_interpolation_gap_s") {
    p.max_derotate_interpolation_gap_s = static_cast<float>(value);
  } else if (key == "body_rate_observer_enable") {
    p.body_rate_observer_enable = b;
  } else if (key == "entry_smooth_enable") {
    p.entry_handoff.enable = b;
  } else if (key == "entry_smooth_duration_s") {
    p.entry_handoff.duration_s = static_cast<float>(value);
  } else if (key == "entry_smooth_initial_thrust_z") {
    p.entry_handoff.initial_thrust_z = static_cast<float>(value);
  } else if (key == "dkf_enable") {
    p.controller.dkf_enable = b;
  } else if (key == "dkf.enable") {
    p.controller.dkf.enable = b;
  } else if (key == "dkf.process_accel_noise") {
    p.controller.dkf.process_accel_noise = static_cast<float>(value);
  } else if (key == "dkf.meas_noise_px") {
    p.controller.dkf.meas_noise_px = static_cast<float>(value);
  } else if (key == "dkf.predict_extra_delay_s") {
    p.controller.dkf.predict_extra_delay_s = static_cast<float>(value);
  } else if (key == "dkf.max_cov_trace") {
    p.controller.dkf.max_cov_trace = static_cast<float>(value);
  } else if (key == "tilt_cap.enable") {
    p.controller.tilt_cap.enable = b;
  } else if (key == "tilt_cap.max_roll_angle_deg") {
    p.controller.tilt_cap.max_roll_angle_deg = static_cast<float>(value);
  } else if (key == "tilt_cap.max_pitch_angle_deg") {
    p.controller.tilt_cap.max_pitch_angle_deg = static_cast<float>(value);
  } else if (key == "tilt_cap.softcap_band_deg") {
    p.controller.tilt_cap.softcap_band_deg = static_cast<float>(value);
  } else if (key == "tilt_cap.hardcap_margin_deg") {
    p.controller.tilt_cap.hardcap_margin_deg = static_cast<float>(value);
  } else if (key == "tilt_cap.hardcap_level_kp") {
    p.controller.tilt_cap.hardcap_level_kp = static_cast<float>(value);
  } else if (key == "tilt_cap.hardcap_max_level_rate_deg_s") {
    p.controller.tilt_cap.hardcap_max_level_rate_deg_s = static_cast<float>(value);
  } else if (key == "tilt_cap.out_lpf_tau_s") {
    p.controller.tilt_cap.out_lpf_tau_s = static_cast<float>(value);
  } else if (key == "tilt_cap.out_max_jerk_deg_s2") {
    p.controller.tilt_cap.out_max_jerk_deg_s2 = static_cast<float>(value);
  } else {
    const auto& fm = pngFloatMembers();
    if (auto it = fm.find(key); it != fm.end()) {
      p.controller.*(it->second) = static_cast<float>(value);
      return true;
    }
    const auto& bm = pngBoolMembers();
    if (auto it = bm.find(key); it != bm.end()) {
      p.controller.*(it->second) = b;
      return true;
    }
    return false;
  }
  return true;
}

std::string stripPngPrefix(std::string name) {
  constexpr const char* kPngPrefix = "target_strike_png.";
  constexpr const char* kShortPrefix = "strike_png.";
  if (name.rfind(kPngPrefix, 0) == 0) {
    name.erase(0, std::strlen(kPngPrefix));
  } else if (name.rfind(kShortPrefix, 0) == 0) {
    name.erase(0, std::strlen(kShortPrefix));
  }
  return name;
}
#endif

}  // namespace

std::vector<ParamEntry> strikePngTunableParams(const StrikePngNodeParams& params,
                                               const char* prefix) {
  std::vector<ParamEntry> out;
  out.reserve(sizeof(kPngDescs) / sizeof(kPngDescs[0]));
  for (const auto& d : kPngDescs) {
    ParamEntry e;
    e.name = std::string(prefix) + d.key;
    e.is_bool = d.is_bool;
    e.value = pngParamValue(params, d.key);
    e.min = d.min_v;
    e.max = d.max_v;
    e.step = d.step;
    out.push_back(std::move(e));
  }
  return out;
}

std::string strikePngParamsJson(const StrikePngNodeParams& params,
                                const char* prefix, const char* tune_mode) {
  const auto entries = strikePngTunableParams(params, prefix);
  std::ostringstream oss;
  oss << R"({"ok":true,"tune_mode":")" << tune_mode << R"(","params":[)";
  bool first = true;
  for (const auto& p : entries) {
    if (!first) {
      oss << ',';
    }
    first = false;
    oss << R"({"name":")" << p.name << R"(","value":)";
    if (p.is_bool) {
      oss << (p.value != 0.0 ? "true" : "false");
      oss << R"(,"type":"bool")";
    } else {
      oss << p.value;
      oss << R"(,"type":"double")";
    }
    oss << R"(,"min":)" << p.min << R"(,"max":)" << p.max << R"(,"step":)"
        << p.step << '}';
  }
  oss << "]}";
  return oss.str();
}

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML

bool applyStrikePngParamUpdate(StrikePngNodeParams& params,
                               const std::string& json) {
  const YAML::Node root = YAML::Load(json);
  if (!root["name"] || !root["value"]) {
    return false;
  }
  const std::string key = stripPngPrefix(root["name"].as<std::string>());
  const YAML::Node value = root["value"];
  double numeric = 0.0;
  try {
    numeric = value.as<double>();
  } catch (...) {
    try {
      numeric = value.as<bool>() ? 1.0 : 0.0;
    } catch (...) {
      return false;
    }
  }
  return pngSetValue(params, key, numeric);
}

bool strikePngParamsFromTunableJson(const std::string& yaml_path,
                                    const std::string& tunable_json,
                                    StrikePngNodeParams& out,
                                    std::string* error_out) {
  try {
    out = circle::strike_png::loadStrikePngParamsFromYaml(yaml_path);
  } catch (const std::exception& ex) {
    if (error_out) {
      *error_out = std::string("yaml load failed: ") + ex.what();
    }
    return false;
  }
  const YAML::Node root = YAML::Load(tunable_json);
  if (!root["params"] || !root["params"].IsSequence()) {
    if (error_out) {
      *error_out = "params snapshot missing params[]";
    }
    return false;
  }
  for (const auto& item : root["params"]) {
    if (!item["name"] || !item["value"]) {
      continue;
    }
    std::ostringstream oss;
    oss << "{\"name\":\"" << item["name"].as<std::string>() << "\",\"value\":";
    if (item["type"] && item["type"].as<std::string>() == "bool") {
      oss << (item["value"].as<bool>() ? "true" : "false");
    } else {
      oss << item["value"].as<double>();
    }
    oss << '}';
    if (!applyStrikePngParamUpdate(out, oss.str())) {
      if (error_out) {
        *error_out = "unknown param in snapshot: " + item["name"].as<std::string>();
      }
      return false;
    }
  }
  return true;
}

bool saveStrikePngTunableParamsToYaml(const std::string& source_yaml_path,
                                      const std::string& output_yaml_path,
                                      const StrikePngNodeParams& params,
                                      std::string* error_out) {
  namespace fs = std::filesystem;
  try {
    YAML::Node root = YAML::LoadFile(source_yaml_path);
    if (!root["strike_png"]) {
      root["strike_png"] = YAML::Node(YAML::NodeType::Map);
    }
    YAML::Node png = root["strike_png"];
    for (const auto& d : kPngDescs) {
      const double v = pngParamValue(params, d.key);
      YAML::Node leaf = d.is_bool ? YAML::Node(v != 0.0) : YAML::Node(v);
      const std::string key(d.key);
      const auto dot = key.find('.');
      if (dot == std::string::npos) {
        png[key] = leaf;
      } else {
        png[key.substr(0, dot)][key.substr(dot + 1)] = leaf;
      }
    }

    const fs::path out_path(output_yaml_path);
    if (out_path.has_parent_path()) {
      std::error_code ec;
      fs::create_directories(out_path.parent_path(), ec);
    }
    const fs::path tmp = output_yaml_path + ".tmp";
    {
      std::ofstream out(tmp);
      if (!out) {
        if (error_out) {
          *error_out = "failed to open temp file for write";
        }
        return false;
      }
      out << root;
      if (!out.good()) {
        if (error_out) {
          *error_out = "failed to write temp yaml";
        }
        std::error_code ec;
        fs::remove(tmp, ec);
        return false;
      }
    }
    std::error_code ec;
    fs::rename(tmp, out_path, ec);
    if (ec) {
      if (error_out) {
        *error_out = "failed to rename temp yaml: " + ec.message();
      }
      return false;
    }
    return true;
  } catch (const std::exception& ex) {
    if (error_out) {
      *error_out = ex.what();
    }
    return false;
  }
}

#endif

}  // namespace circle::debug_common
