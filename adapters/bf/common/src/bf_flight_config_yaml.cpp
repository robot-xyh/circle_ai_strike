#include "circle/bf/bf_flight_config_yaml.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML

#include <yaml-cpp/yaml.h>

namespace circle::bf {

void loadBfFlightMspFromYaml(const YAML::Node& msp, BfFlightMspConfig& cfg) {
  if (!msp || !msp.IsMap()) {
    return;
  }
  if (msp["device"]) {
    cfg.device = msp["device"].as<std::string>();
  }
  if (msp["baud"]) {
    cfg.baud = msp["baud"].as<int>();
  }
  if (msp["passthrough_in_dry_run"]) {
    cfg.passthrough_in_dry_run = msp["passthrough_in_dry_run"].as<bool>();
  }
  if (msp["passthrough_log"]) {
    cfg.passthrough_log = msp["passthrough_log"].as<bool>();
  }
  if (msp["passthrough_log_interval_s"]) {
    cfg.passthrough_log_interval_s =
        msp["passthrough_log_interval_s"].as<double>();
  }
  if (msp["passthrough_throttle_jump_pwm"]) {
    cfg.passthrough_throttle_jump_pwm =
        msp["passthrough_throttle_jump_pwm"].as<uint16_t>();
  }
  if (msp["passthrough_hz"]) {
    cfg.passthrough_hz = msp["passthrough_hz"].as<double>();
  }
  if (msp["live_publish_hz"]) {
    cfg.live_publish_hz = msp["live_publish_hz"].as<double>();
  }
  if (msp["attitude_poll_divisor"]) {
    cfg.attitude_poll_divisor = msp["attitude_poll_divisor"].as<uint32_t>();
  }
  if (msp["status_poll_divisor"]) {
    cfg.status_poll_divisor = msp["status_poll_divisor"].as<uint32_t>();
  }
  if (msp["override_grace_hold_s"]) {
    cfg.override_grace_hold_s = msp["override_grace_hold_s"].as<double>();
  }
  if (msp["override_mode_flag"]) {
    cfg.override_mode_flag = msp["override_mode_flag"].as<uint32_t>();
  }
  if (msp["override_mode_flag_auto"]) {
    cfg.override_mode_flag_auto = msp["override_mode_flag_auto"].as<bool>();
  }
  if (msp["override_channels_mask"]) {
    cfg.override_channels_mask = msp["override_channels_mask"].as<uint32_t>();
  }
  if (msp["passthrough_channel_count"]) {
    cfg.passthrough_channel_count =
        msp["passthrough_channel_count"].as<uint16_t>();
  }
}

void loadBfFlightRcFromYaml(const YAML::Node& rc, BfRcMapperConfig& cfg) {
  if (!rc || !rc.IsMap()) {
    return;
  }
  if (rc["rc_min"]) {
    cfg.rc_min = rc["rc_min"].as<uint16_t>();
  }
  if (rc["rc_mid"]) {
    cfg.rc_mid = rc["rc_mid"].as<uint16_t>();
  }
  if (rc["rc_max"]) {
    cfg.rc_max = rc["rc_max"].as<uint16_t>();
  }
  if (rc["roll_channel"]) {
    cfg.roll_channel = rc["roll_channel"].as<uint16_t>();
  }
  if (rc["pitch_channel"]) {
    cfg.pitch_channel = rc["pitch_channel"].as<uint16_t>();
  }
  if (rc["throttle_channel"]) {
    cfg.throttle_channel = rc["throttle_channel"].as<uint16_t>();
  }
  if (rc["yaw_channel"]) {
    cfg.yaw_channel = rc["yaw_channel"].as<uint16_t>();
  }
  if (rc["aux_arm_channel"]) {
    cfg.aux_arm_channel = rc["aux_arm_channel"].as<uint16_t>();
  }
  if (rc["max_roll_rate_rad_s"]) {
    cfg.max_roll_rate_rad_s = rc["max_roll_rate_rad_s"].as<float>();
  }
  if (rc["max_pitch_rate_rad_s"]) {
    cfg.max_pitch_rate_rad_s = rc["max_pitch_rate_rad_s"].as<float>();
  }
  if (rc["max_yaw_rate_rad_s"]) {
    cfg.max_yaw_rate_rad_s = rc["max_yaw_rate_rad_s"].as<float>();
  }
  sanitizeRcMapperConfig(cfg);
}

BfFlightYamlSections loadBfFlightYamlSectionsFromFile(
    const std::string& path) {
  BfFlightYamlSections out;
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node bf = root["bf_flight"] ? root["bf_flight"] : root;
  if (bf["msp"]) {
    loadBfFlightMspFromYaml(bf["msp"], out.msp);
  }
  if (bf["rc"]) {
    loadBfFlightRcFromYaml(bf["rc"], out.rc);
  }
  return out;
}

}  // namespace circle::bf

#endif
