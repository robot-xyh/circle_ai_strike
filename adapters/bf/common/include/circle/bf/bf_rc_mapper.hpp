#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "circle/types/rate_command.hpp"

namespace circle::bf {

inline constexpr uint16_t kRcPwmHardMin = 1000;
inline constexpr uint16_t kRcPwmHardMax = 2000;

/** Clamp raw PWM to [1000, 2000]; tolerates TX low/high-end drift (e.g. 988). */
inline uint16_t clampRcPwm(uint16_t pwm) {
  return static_cast<uint16_t>(std::clamp(
      static_cast<int>(pwm), static_cast<int>(kRcPwmHardMin),
      static_cast<int>(kRcPwmHardMax)));
}

struct BfRcMapperConfig {
  uint16_t channel_count{8};
  uint16_t roll_channel{0};
  uint16_t pitch_channel{1};
  uint16_t throttle_channel{2};
  uint16_t yaw_channel{3};
  uint16_t aux_arm_channel{4};
  uint16_t rc_min{1000};
  uint16_t rc_mid{1500};
  uint16_t rc_max{2000};
  float max_roll_rate_rad_s{2.0F};
  float max_pitch_rate_rad_s{2.0F};
  float max_yaw_rate_rad_s{0.7F};
  uint16_t arm_value{1800};
  uint16_t disarm_value{1000};
};

class BfRcMapper {
 public:
  explicit BfRcMapper(BfRcMapperConfig config = {});

  std::vector<uint16_t> mapRates(const circle::types::RateCommand& cmd,
                                 bool armed) const;

  const BfRcMapperConfig& config() const { return config_; }

 private:
  uint16_t rateToPwm(float rate, float max_rate) const;

  BfRcMapperConfig config_;
};

void sanitizeRcMapperConfig(BfRcMapperConfig& cfg);

}  // namespace circle::bf
