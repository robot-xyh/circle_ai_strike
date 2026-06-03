#include "circle/debug_common/strike_param_tune.hpp"

#include <fstream>
#include <sstream>
#include <cstring>

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "circle/strike/strike_params_yaml.hpp"
#endif

namespace circle::debug_common {

namespace {

struct CoreDesc {
  const char* key;
  double min_v;
  double max_v;
  double step;
  bool is_bool;
};

constexpr CoreDesc kCoreDescs[] = {
    {"dry_run", 0.0, 1.0, 1.0, true},
    {"require_armed_to_command", 0.0, 1.0, 1.0, true},
    {"min_score", 0.0, 1.0, 0.01, false},
    {"min_bbox_area", 0.0, 50000.0, 10.0, false},
    {"max_bbox_aspect_ratio", 0.5, 10.0, 0.1, false},
    {"detection_stale_s", 0.05, 2.0, 0.05, false},
    {"lost_timeout_s", 0.2, 10.0, 0.1, false},
    {"dkf_enable", 0.0, 1.0, 1.0, true},
    {"dkf_process_accel_noise", 0.0, 50.0, 0.5, false},
    {"dkf_meas_noise_px", 0.0, 50.0, 0.5, false},
    {"dkf_predict_extra_delay_s", 0.0, 0.5, 0.005, false},
    {"dkf_max_cov_trace", 0.0, 2.0, 0.01, false},
    {"lateral_output_sign", -1.0, 1.0, 1.0, false},
    {"longitudinal_output_sign", -1.0, 1.0, 1.0, false},
    {"lateral_kp_rate", 0.0, 12.0, 0.1, false},
    {"lateral_kd_rate", 0.0, 4.0, 0.02, false},
    {"longitudinal_kp_rate", 0.0, 12.0, 0.1, false},
    {"longitudinal_kd_rate", 0.0, 4.0, 0.02, false},
    {"x_deadband", 0.0, 0.20, 0.002, false},
    {"y_deadband", 0.0, 0.20, 0.002, false},
    {"x_deadband_px", 0.0, 200.0, 1.0, false},
    {"y_deadband_px", 0.0, 200.0, 1.0, false},
    {"aim_offset_x_px", -160.0, 160.0, 1.0, false},
    {"aim_offset_y_px", -160.0, 160.0, 1.0, false},
    {"pixel_dot_lpf_tau_s", 0.0, 0.6, 0.005, false},
    {"rate_lpf_tau_s", 0.0, 0.30, 0.005, false},
    {"max_jerk_rad_s2", 0.0, 200.0, 1.0, false},
    {"max_roll_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"max_pitch_rate_rad_s", 0.05, 6.9813, 0.05, false},
    {"level_hold_enabled", 0.0, 1.0, 1.0, true},
    {"level_kp_rate", 0.0, 10.0, 0.05, false},
    {"hard_level_kp_rate", 0.0, 15.0, 0.1, false},
    {"hover_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"constant_thrust_scalar", 0.05, 0.95, 0.005, false},
    {"max_roll_angle_rad", 0.05, 1.5, 0.01, false},
    {"max_pitch_angle_rad", 0.05, 1.5, 0.01, false},
    {"tilt_hardcap_margin_rad", 0.0, 0.6, 0.005, false},
    // Final approach parameters
    {"final_approach.gate.boost_enable", 0.0, 1.0, 1.0, true},
    {"final_approach.gate.area_ratio_enter", 0.0, 1.0, 0.01, false},
    {"final_approach.gate.area_ratio_exit", 0.0, 1.0, 0.01, false},
    {"final_approach.scaling.kp_scale", 0.0, 5.0, 0.1, false},
    {"final_approach.scaling.kd_scale", 0.0, 5.0, 0.1, false},
    {"final_approach.scaling.deadband_scale", 0.0, 1.0, 0.05, false},
    {"final_approach.scaling.kp_proximity_gain", 0.0, 5.0, 0.1, false},
    {"final_approach.scaling.kd_proximity_gain", 0.0, 5.0, 0.1, false},
    {"final_approach.commit.enable", 0.0, 1.0, 1.0, true},
    {"final_approach.commit.command_hold_s", 0.0, 5.0, 0.1, false},
    {"final_approach.commit.thrust_scalar", 0.0, 1.0, 0.01, false},
    {"final_approach.commit.align_hold_s", 0.0, 2.0, 0.05, false},
    {"final_approach.edge_protect.enable", 0.0, 1.0, 1.0, true},
    {"final_approach.edge_protect.margin_x_px", 0.0, 200.0, 5.0, false},
    {"final_approach.edge_protect.margin_y_px", 0.0, 200.0, 5.0, false},
    {"final_approach.edge_protect.thrust_scale", 0.0, 1.0, 0.05, false},
    {"final_approach.thrust.tilt_slowdown_enable", 0.0, 1.0, 1.0, true},
    {"final_approach.thrust.tilt_slowdown_min_scale", 0.0, 1.0, 0.05, false},
    {"final_approach.thrust.thrust_taper_enable", 0.0, 1.0, 1.0, true},
    {"final_approach.thrust.thrust_taper_min_scale", 0.0, 1.0, 0.05, false},
    {"final_approach.fallback.decay_tau_s", 0.1, 5.0, 0.1, false},
    {"final_approach.fallback.max_s", 0.5, 10.0, 0.5, false},
    {"final_approach.fallback.pitch_bias_rad", -1.0, 0.0, 0.05, false},
    // Speed governor parameters
    {"speed_governor.enable", 0.0, 1.0, 1.0, true},
    {"speed_governor.start_m_s", 0.0, 30.0, 0.5, false},
    {"speed_governor.full_m_s", 0.0, 50.0, 0.5, false},
    {"speed_governor.min_image_scale", 0.0, 1.0, 0.05, false},
    // Preclimb parameters
    {"preclimb.xy_gate_enable", 0.0, 1.0, 1.0, true},
    {"preclimb.xy_hold_s", 0.0, 5.0, 0.1, false},
    {"preclimb.thrust_excess_scale", 0.0, 1.0, 0.05, false},
    {"preclimb.safe_hold_enable", 0.0, 1.0, 1.0, true},
    {"preclimb.safe_hold_level_kp", 0.0, 10.0, 0.1, false},
    // Yaw parameters
    {"yaw.bearing_kp", 0.0, 5.0, 0.1, false},
    {"yaw.hold_gain", 0.0, 5.0, 0.1, false},
    {"yaw.lock_enabled", 0.0, 1.0, 1.0, true},
};

double paramValue(const circle::strike::StrikeParams& p, const char* key) {
  if (std::string(key) == "dry_run") return p.dry_run ? 1.0 : 0.0;
  if (std::string(key) == "require_armed_to_command") {
    return p.require_armed_to_command ? 1.0 : 0.0;
  }
  if (std::string(key) == "min_score") return p.filter.min_score;
  if (std::string(key) == "min_bbox_area") return p.filter.min_bbox_area;
  if (std::string(key) == "max_bbox_aspect_ratio") {
    return p.filter.max_bbox_aspect_ratio;
  }
  if (std::string(key) == "detection_stale_s") return p.detection_stale_s;
  if (std::string(key) == "lost_timeout_s") return p.lost_timeout_s;
  if (std::string(key) == "dkf_enable") return p.dkf.enable ? 1.0 : 0.0;
  if (std::string(key) == "dkf_process_accel_noise") {
    return p.dkf.process_accel_noise;
  }
  if (std::string(key) == "dkf_meas_noise_px") return p.dkf.meas_noise_px;
  if (std::string(key) == "dkf_predict_extra_delay_s") {
    return p.dkf.predict_extra_delay_s;
  }
  if (std::string(key) == "dkf_max_cov_trace") return p.dkf.max_cov_trace;
  if (std::string(key) == "lateral_output_sign") return p.lateral_output_sign;
  if (std::string(key) == "longitudinal_output_sign") {
    return p.longitudinal_output_sign;
  }
  if (std::string(key) == "lateral_kp_rate") return p.lateral_kp_rate;
  if (std::string(key) == "lateral_kd_rate") return p.lateral_kd_rate;
  if (std::string(key) == "longitudinal_kp_rate") return p.longitudinal_kp_rate;
  if (std::string(key) == "longitudinal_kd_rate") return p.longitudinal_kd_rate;
  if (std::string(key) == "x_deadband") return p.x_deadband;
  if (std::string(key) == "y_deadband") return p.y_deadband;
  if (std::string(key) == "x_deadband_px") return p.x_deadband_px;
  if (std::string(key) == "y_deadband_px") return p.y_deadband_px;
  if (std::string(key) == "aim_offset_x_px") return p.aim_offset_x_px;
  if (std::string(key) == "aim_offset_y_px") return p.aim_offset_y_px;
  if (std::string(key) == "pixel_dot_lpf_tau_s") return p.pixel_dot_lpf_tau_s;
  if (std::string(key) == "rate_lpf_tau_s") return p.rate_lpf_tau_s;
  if (std::string(key) == "max_jerk_rad_s2") return p.max_jerk_rad_s2;
  if (std::string(key) == "max_roll_rate_rad_s") return p.max_roll_rate_rad_s;
  if (std::string(key) == "max_pitch_rate_rad_s") return p.max_pitch_rate_rad_s;
  if (std::string(key) == "level_hold_enabled") {
    return p.waiting.level_hold_enabled ? 1.0 : 0.0;
  }
  if (std::string(key) == "level_kp_rate") return p.waiting.level_kp;
  if (std::string(key) == "hard_level_kp_rate") return p.force_level.hard_level_kp;
  if (std::string(key) == "hover_thrust_scalar") return p.thrust.hover_scalar;
  if (std::string(key) == "constant_thrust_scalar") {
    return p.thrust.constant_scalar;
  }
  if (std::string(key) == "max_roll_angle_rad") return p.tilt_cap.max_roll_angle_rad;
  if (std::string(key) == "max_pitch_angle_rad") return p.tilt_cap.max_pitch_angle_rad;
  if (std::string(key) == "tilt_hardcap_margin_rad") {
    return p.tilt_cap.hardcap_margin_rad;
  }
  // Final approach parameters
  if (std::string(key) == "final_approach.gate.boost_enable") {
    return p.final_approach.gate.boost_enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "final_approach.gate.area_ratio_enter") {
    return p.final_approach.gate.area_ratio_enter;
  }
  if (std::string(key) == "final_approach.gate.area_ratio_exit") {
    return p.final_approach.gate.area_ratio_exit;
  }
  if (std::string(key) == "final_approach.scaling.kp_scale") {
    return p.final_approach.scaling.kp_scale;
  }
  if (std::string(key) == "final_approach.scaling.kd_scale") {
    return p.final_approach.scaling.kd_scale;
  }
  if (std::string(key) == "final_approach.scaling.deadband_scale") {
    return p.final_approach.scaling.deadband_scale;
  }
  if (std::string(key) == "final_approach.scaling.kp_proximity_gain") {
    return p.final_approach.scaling.kp_proximity_gain;
  }
  if (std::string(key) == "final_approach.scaling.kd_proximity_gain") {
    return p.final_approach.scaling.kd_proximity_gain;
  }
  if (std::string(key) == "final_approach.commit.enable") {
    return p.final_approach.commit.enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "final_approach.commit.command_hold_s") {
    return p.final_approach.commit.command_hold_s;
  }
  if (std::string(key) == "final_approach.commit.thrust_scalar") {
    return p.final_approach.commit.thrust_scalar;
  }
  if (std::string(key) == "final_approach.commit.align_hold_s") {
    return p.final_approach.commit.align_hold_s;
  }
  if (std::string(key) == "final_approach.edge_protect.enable") {
    return p.final_approach.edge_protect.enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "final_approach.edge_protect.margin_x_px") {
    return p.final_approach.edge_protect.margin_x_px;
  }
  if (std::string(key) == "final_approach.edge_protect.margin_y_px") {
    return p.final_approach.edge_protect.margin_y_px;
  }
  if (std::string(key) == "final_approach.edge_protect.thrust_scale") {
    return p.final_approach.edge_protect.thrust_scale;
  }
  if (std::string(key) == "final_approach.thrust.tilt_slowdown_enable") {
    return p.final_approach.thrust.tilt_slowdown_enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "final_approach.thrust.tilt_slowdown_min_scale") {
    return p.final_approach.thrust.tilt_slowdown_min_scale;
  }
  if (std::string(key) == "final_approach.thrust.thrust_taper_enable") {
    return p.final_approach.thrust.thrust_taper_enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "final_approach.thrust.thrust_taper_min_scale") {
    return p.final_approach.thrust.thrust_taper_min_scale;
  }
  if (std::string(key) == "final_approach.fallback.decay_tau_s") {
    return p.final_approach.fallback.decay_tau_s;
  }
  if (std::string(key) == "final_approach.fallback.max_s") {
    return p.final_approach.fallback.max_s;
  }
  if (std::string(key) == "final_approach.fallback.pitch_bias_rad") {
    return p.final_approach.fallback.pitch_bias_rad;
  }
  // Speed governor parameters
  if (std::string(key) == "speed_governor.enable") {
    return p.speed_governor.enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "speed_governor.start_m_s") {
    return p.speed_governor.start_m_s;
  }
  if (std::string(key) == "speed_governor.full_m_s") {
    return p.speed_governor.full_m_s;
  }
  if (std::string(key) == "speed_governor.min_image_scale") {
    return p.speed_governor.min_image_scale;
  }
  // Preclimb parameters
  if (std::string(key) == "preclimb.xy_gate_enable") {
    return p.preclimb.xy_gate_enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "preclimb.xy_hold_s") {
    return p.preclimb.xy_hold_s;
  }
  if (std::string(key) == "preclimb.thrust_excess_scale") {
    return p.preclimb.thrust_excess_scale;
  }
  if (std::string(key) == "preclimb.safe_hold_enable") {
    return p.preclimb.safe_hold_enable ? 1.0 : 0.0;
  }
  if (std::string(key) == "preclimb.safe_hold_level_kp") {
    return p.preclimb.safe_hold_level_kp;
  }
  // Yaw parameters
  if (std::string(key) == "yaw.bearing_kp") {
    return p.yaw.bearing_kp;
  }
  if (std::string(key) == "yaw.hold_gain") {
    return p.yaw.hold_gain;
  }
  if (std::string(key) == "yaw.lock_enabled") {
    return p.yaw.lock_enabled ? 1.0 : 0.0;
  }
  return 0.0;
}

std::string stripParamPrefix(std::string name) {
  constexpr const char* kTargetPrefix = "target_strike.";
  constexpr const char* kStrikePrefix = "strike.";
  if (name.rfind(kTargetPrefix, 0) == 0) {
    name.erase(0, std::strlen(kTargetPrefix));
  } else if (name.rfind(kStrikePrefix, 0) == 0) {
    name.erase(0, std::strlen(kStrikePrefix));
  }
  return name;
}

}  // namespace

std::vector<ParamEntry> strikeCoreTunableParams(
    const circle::strike::StrikeParams& params, const char* prefix) {
  std::vector<ParamEntry> out;
  out.reserve(sizeof(kCoreDescs) / sizeof(kCoreDescs[0]));
  for (const auto& d : kCoreDescs) {
    ParamEntry e;
    e.name = std::string(prefix) + d.key;
    e.is_bool = d.is_bool;
    e.value = paramValue(params, d.key);
    e.min = d.min_v;
    e.max = d.max_v;
    e.step = d.step;
    out.push_back(std::move(e));
  }
  return out;
}

std::string strikeCoreParamsJson(const circle::strike::StrikeParams& params,
                                 const char* prefix, const char* tune_mode) {
  const auto entries = strikeCoreTunableParams(params, prefix);
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

bool applyStrikeParamUpdate(circle::strike::StrikeParams& params,
                            const std::string& json) {
  const YAML::Node root = YAML::Load(json);
  if (!root["name"] || !root["value"]) {
    return false;
  }
  const std::string key = stripParamPrefix(root["name"].as<std::string>());
  const YAML::Node value = root["value"];

  if (key == "min_score") {
    params.filter.min_score = value.as<double>();
  } else if (key == "min_bbox_area" || key == "min_bbox_area_px") {
    params.filter.min_bbox_area = value.as<double>();
  } else if (key == "max_bbox_aspect_ratio") {
    params.filter.max_bbox_aspect_ratio = value.as<double>();
  } else if (key == "detection_stale_s") {
    params.detection_stale_s = value.as<double>();
  } else if (key == "lost_timeout_s") {
    params.lost_timeout_s = value.as<double>();
  } else if (key == "dkf_enable") {
    params.dkf.enable = value.as<bool>();
  } else if (key == "dkf_process_accel_noise") {
    params.dkf.process_accel_noise = value.as<double>();
  } else if (key == "dkf_meas_noise_px") {
    params.dkf.meas_noise_px = value.as<double>();
  } else if (key == "dkf_predict_extra_delay_s") {
    params.dkf.predict_extra_delay_s = value.as<double>();
  } else if (key == "dkf_max_cov_trace") {
    params.dkf.max_cov_trace = value.as<double>();
  } else if (key == "lateral_output_sign") {
    params.lateral_output_sign = value.as<float>();
  } else if (key == "longitudinal_output_sign") {
    params.longitudinal_output_sign = value.as<float>();
  } else if (key == "lateral_kp_rate") {
    params.lateral_kp_rate = value.as<float>();
  } else if (key == "lateral_kd_rate") {
    params.lateral_kd_rate = value.as<float>();
  } else if (key == "longitudinal_kp_rate") {
    params.longitudinal_kp_rate = value.as<float>();
  } else if (key == "longitudinal_kd_rate") {
    params.longitudinal_kd_rate = value.as<float>();
  } else if (key == "x_deadband") {
    params.x_deadband = value.as<float>();
  } else if (key == "y_deadband") {
    params.y_deadband = value.as<float>();
  } else if (key == "x_deadband_px") {
    params.x_deadband_px = value.as<float>();
  } else if (key == "y_deadband_px") {
    params.y_deadband_px = value.as<float>();
  } else if (key == "aim_offset_x_px") {
    params.aim_offset_x_px = value.as<float>();
  } else if (key == "aim_offset_y_px") {
    params.aim_offset_y_px = value.as<float>();
  } else if (key == "pixel_dot_lpf_tau_s") {
    params.pixel_dot_lpf_tau_s = value.as<float>();
  } else if (key == "rate_lpf_tau_s") {
    params.rate_lpf_tau_s = value.as<float>();
  } else if (key == "max_jerk_rad_s2") {
    params.max_jerk_rad_s2 = value.as<float>();
  } else if (key == "max_roll_rate_rad_s") {
    params.max_roll_rate_rad_s = value.as<float>();
  } else if (key == "max_pitch_rate_rad_s") {
    params.max_pitch_rate_rad_s = value.as<float>();
  } else if (key == "level_hold_enabled") {
    params.waiting.level_hold_enabled = value.as<bool>();
  } else if (key == "level_kp_rate") {
    params.waiting.level_kp = value.as<float>();
  } else if (key == "hard_level_kp_rate") {
    params.force_level.hard_level_kp = value.as<float>();
  } else if (key == "hover_thrust_scalar") {
    params.thrust.hover_scalar = value.as<float>();
  } else if (key == "constant_thrust_scalar") {
    params.thrust.constant_scalar = value.as<float>();
  } else if (key == "max_roll_angle_rad") {
    params.tilt_cap.max_roll_angle_rad = value.as<float>();
  } else if (key == "max_pitch_angle_rad") {
    params.tilt_cap.max_pitch_angle_rad = value.as<float>();
  } else if (key == "tilt_hardcap_margin_rad") {
    params.tilt_cap.hardcap_margin_rad = value.as<float>();
  } else if (key == "require_armed_to_command") {
    params.require_armed_to_command = value.as<bool>();
  } else if (key == "dry_run") {
    params.dry_run = value.as<bool>();
  // Final approach parameters
  } else if (key == "final_approach.gate.boost_enable") {
    params.final_approach.gate.boost_enable = value.as<bool>();
  } else if (key == "final_approach.gate.area_ratio_enter") {
    params.final_approach.gate.area_ratio_enter = value.as<float>();
  } else if (key == "final_approach.gate.area_ratio_exit") {
    params.final_approach.gate.area_ratio_exit = value.as<float>();
  } else if (key == "final_approach.scaling.kp_scale") {
    params.final_approach.scaling.kp_scale = value.as<float>();
  } else if (key == "final_approach.scaling.kd_scale") {
    params.final_approach.scaling.kd_scale = value.as<float>();
  } else if (key == "final_approach.scaling.deadband_scale") {
    params.final_approach.scaling.deadband_scale = value.as<float>();
  } else if (key == "final_approach.scaling.kp_proximity_gain") {
    params.final_approach.scaling.kp_proximity_gain = value.as<float>();
  } else if (key == "final_approach.scaling.kd_proximity_gain") {
    params.final_approach.scaling.kd_proximity_gain = value.as<float>();
  } else if (key == "final_approach.commit.enable") {
    params.final_approach.commit.enable = value.as<bool>();
  } else if (key == "final_approach.commit.command_hold_s") {
    params.final_approach.commit.command_hold_s = value.as<float>();
  } else if (key == "final_approach.commit.thrust_scalar") {
    params.final_approach.commit.thrust_scalar = value.as<float>();
  } else if (key == "final_approach.commit.align_hold_s") {
    params.final_approach.commit.align_hold_s = value.as<float>();
  } else if (key == "final_approach.edge_protect.enable") {
    params.final_approach.edge_protect.enable = value.as<bool>();
  } else if (key == "final_approach.edge_protect.margin_x_px") {
    params.final_approach.edge_protect.margin_x_px = value.as<float>();
  } else if (key == "final_approach.edge_protect.margin_y_px") {
    params.final_approach.edge_protect.margin_y_px = value.as<float>();
  } else if (key == "final_approach.edge_protect.thrust_scale") {
    params.final_approach.edge_protect.thrust_scale = value.as<float>();
  } else if (key == "final_approach.thrust.tilt_slowdown_enable") {
    params.final_approach.thrust.tilt_slowdown_enable = value.as<bool>();
  } else if (key == "final_approach.thrust.tilt_slowdown_min_scale") {
    params.final_approach.thrust.tilt_slowdown_min_scale = value.as<float>();
  } else if (key == "final_approach.thrust.thrust_taper_enable") {
    params.final_approach.thrust.thrust_taper_enable = value.as<bool>();
  } else if (key == "final_approach.thrust.thrust_taper_min_scale") {
    params.final_approach.thrust.thrust_taper_min_scale = value.as<float>();
  } else if (key == "final_approach.fallback.decay_tau_s") {
    params.final_approach.fallback.decay_tau_s = value.as<float>();
  } else if (key == "final_approach.fallback.max_s") {
    params.final_approach.fallback.max_s = value.as<float>();
  } else if (key == "final_approach.fallback.pitch_bias_rad") {
    params.final_approach.fallback.pitch_bias_rad = value.as<float>();
  // Speed governor parameters
  } else if (key == "speed_governor.enable") {
    params.speed_governor.enable = value.as<bool>();
  } else if (key == "speed_governor.start_m_s") {
    params.speed_governor.start_m_s = value.as<float>();
  } else if (key == "speed_governor.full_m_s") {
    params.speed_governor.full_m_s = value.as<float>();
  } else if (key == "speed_governor.min_image_scale") {
    params.speed_governor.min_image_scale = value.as<float>();
  // Preclimb parameters
  } else if (key == "preclimb.xy_gate_enable") {
    params.preclimb.xy_gate_enable = value.as<bool>();
  } else if (key == "preclimb.xy_hold_s") {
    params.preclimb.xy_hold_s = value.as<float>();
  } else if (key == "preclimb.thrust_excess_scale") {
    params.preclimb.thrust_excess_scale = value.as<float>();
  } else if (key == "preclimb.safe_hold_enable") {
    params.preclimb.safe_hold_enable = value.as<bool>();
  } else if (key == "preclimb.safe_hold_level_kp") {
    params.preclimb.safe_hold_level_kp = value.as<float>();
  // Yaw parameters
  } else if (key == "yaw.bearing_kp") {
    params.yaw.bearing_kp = value.as<float>();
  } else if (key == "yaw.hold_gain") {
    params.yaw.hold_gain = value.as<float>();
  } else if (key == "yaw.lock_enabled") {
    params.yaw.lock_enabled = value.as<bool>();
  } else {
    return false;
  }
  params.clamp();
  return true;
}

namespace {

void setYamlAtPath(YAML::Node node, const std::string& path, const YAML::Node& val) {
  const size_t dot = path.find('.');
  if (dot == std::string::npos) {
    node[path] = val;
    return;
  }
  const std::string head = path.substr(0, dot);
  if (!node[head]) {
    node[head] = YAML::Node(YAML::NodeType::Map);
  }
  setYamlAtPath(node[head], path.substr(dot + 1), val);
}

std::string yamlPathForCoreKey(const std::string& key) {
  if (key == "min_score") return "filter.min_score";
  if (key == "min_bbox_area") return "filter.min_bbox_area";
  if (key == "max_bbox_aspect_ratio") return "filter.max_bbox_aspect_ratio";
  if (key == "dkf_process_accel_noise") return "dkf.process_accel_noise";
  if (key == "dkf_meas_noise_px") return "dkf.meas_noise_px";
  if (key == "dkf_predict_extra_delay_s") return "dkf.predict_extra_delay_s";
  if (key == "dkf_max_cov_trace") return "dkf.max_cov_trace";
  if (key == "level_hold_enabled") return "waiting.level_hold_enabled";
  if (key == "level_kp_rate") return "waiting.level_kp";
  if (key == "hard_level_kp_rate") return "force_level.hard_level_kp";
  if (key == "hover_thrust_scalar") return "thrust.hover_scalar";
  if (key == "constant_thrust_scalar") return "thrust.constant_scalar";
  if (key == "max_roll_angle_rad") return "tilt_cap.max_roll_angle_rad";
  if (key == "max_pitch_angle_rad") return "tilt_cap.max_pitch_angle_rad";
  if (key == "tilt_hardcap_margin_rad") return "tilt_cap.hardcap_margin_rad";
  return key;
}

void writeStrikeTunableParam(YAML::Node strike, const char* key, double value,
                             bool is_bool) {
  const YAML::Node yaml_value =
      is_bool ? YAML::Node(value != 0.0) : YAML::Node(value);
  const std::string k(key);
  const std::string path = yamlPathForCoreKey(k);
  setYamlAtPath(strike, path, yaml_value);
  if (k == "dkf_enable") {
    setYamlAtPath(strike, "dkf.enable", yaml_value);
  }
}

}  // namespace

bool strikeParamsFromTunableJson(const std::string& yaml_path,
                                 const std::string& tunable_json,
                                 circle::strike::StrikeParams& out,
                                 std::string* error_out) {
  try {
    out = circle::strike::loadStrikeParamsFromYaml(yaml_path);
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
    if (!applyStrikeParamUpdate(out, oss.str())) {
      if (error_out) {
        *error_out = "unknown param in snapshot: " + item["name"].as<std::string>();
      }
      return false;
    }
  }
  return true;
}

bool saveStrikeTunableParamsToYaml(const std::string& source_yaml_path,
                                   const std::string& output_yaml_path,
                                   const circle::strike::StrikeParams& params,
                                   std::string* error_out) {
  namespace fs = std::filesystem;
  try {
    YAML::Node root = YAML::LoadFile(source_yaml_path);
    if (!root["strike"]) {
      root["strike"] = YAML::Node(YAML::NodeType::Map);
    }
    YAML::Node strike = root["strike"];
    for (const auto& d : kCoreDescs) {
      writeStrikeTunableParam(strike, d.key, paramValue(params, d.key), d.is_bool);
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
