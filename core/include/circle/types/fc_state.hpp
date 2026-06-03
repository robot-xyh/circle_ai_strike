#pragma once

#include <cstdint>

#include "circle/types/time.hpp"

namespace circle::types {

enum class FcBackend : uint8_t {
  Px4 = 0,
  Betaflight = 1,
};

struct FcState {
  TimestampNs stamp_ns{0};
  bool valid{false};
  bool armed{false};
  float roll_rad{0.0F};
  float pitch_rad{0.0F};
  float yaw_rad{0.0F};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float yaw_rate_rad_s{0.0F};
  /** RC throttle PWM (1000-2000) from MSP_RC when available. */
  float throttle_pwm{0.0F};
  /** Throttle normalized to [0,1] from rc_min/rc_max. */
  float throttle_norm{0.0F};
  bool position_z_valid{false};
  float position_ned_z{0.0F};
  float velocity_ned_z{0.0F};
  bool velocity_xy_valid{false};
  float velocity_ned_x{0.0F};
  float velocity_ned_y{0.0F};
  float velocity_xy_m_s{0.0F};
};

}  // namespace circle::types
