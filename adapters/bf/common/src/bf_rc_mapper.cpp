#include "circle/bf/bf_rc_mapper.hpp"

#include <algorithm>

namespace circle::bf {

void sanitizeRcMapperConfig(BfRcMapperConfig& cfg) {
  cfg.rc_min = std::max(cfg.rc_min, kRcPwmHardMin);
  cfg.rc_max = std::min(cfg.rc_max, kRcPwmHardMax);
  if (cfg.rc_max < cfg.rc_min) {
    cfg.rc_max = cfg.rc_min;
  }
  cfg.rc_mid = clampRcPwm(cfg.rc_mid);
  cfg.arm_value = clampRcPwm(cfg.arm_value);
  cfg.disarm_value = clampRcPwm(cfg.disarm_value);
}

BfRcMapper::BfRcMapper(BfRcMapperConfig config) : config_(std::move(config)) {
  sanitizeRcMapperConfig(config_);
}

uint16_t BfRcMapper::rateToPwm(float rate, float max_rate) const {
  if (max_rate <= 1.0e-6F) {
    return config_.rc_mid;
  }
  const float norm = std::clamp(rate / max_rate, -1.0F, 1.0F);
  const float span = static_cast<float>(config_.rc_max - config_.rc_min) * 0.5F;
  return clampRcPwm(
      static_cast<uint16_t>(std::round(config_.rc_mid + norm * span)));
}

std::vector<uint16_t> BfRcMapper::mapRates(const circle::types::RateCommand& cmd,
                                           bool armed) const {
  std::vector<uint16_t> channels(config_.channel_count, config_.rc_mid);
  if (config_.roll_channel < channels.size()) {
    channels[config_.roll_channel] =
        rateToPwm(cmd.roll_rate_rad_s, config_.max_roll_rate_rad_s);
  }
  if (config_.pitch_channel < channels.size()) {
    channels[config_.pitch_channel] =
        rateToPwm(cmd.pitch_rate_rad_s, config_.max_pitch_rate_rad_s);
  }
  if (config_.yaw_channel < channels.size()) {
    channels[config_.yaw_channel] =
        rateToPwm(cmd.yaw_rate_rad_s, config_.max_yaw_rate_rad_s);
  }
  if (config_.throttle_channel < channels.size()) {
    const float thrust = std::clamp(cmd.thrust_z, 0.0F, 1.0F);
    channels[config_.throttle_channel] = clampRcPwm(static_cast<uint16_t>(
        std::round(config_.rc_min +
                   thrust * static_cast<float>(config_.rc_max - config_.rc_min))));
  }
  if (config_.aux_arm_channel < channels.size()) {
    channels[config_.aux_arm_channel] = armed ? config_.arm_value : config_.disarm_value;
  }
  return channels;
}

}  // namespace circle::bf
