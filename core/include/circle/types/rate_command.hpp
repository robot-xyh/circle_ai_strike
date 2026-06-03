#pragma once

namespace circle::types {

/** Controller output: body rates + normalized thrust scalar [0,1]. */
struct RateCommand {
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float yaw_rate_rad_s{0.0F};
  float thrust_z{0.0F};
};

struct SafetyContext {
  bool dry_run{false};
  bool require_armed_to_command{true};
  bool armed{false};
};

}  // namespace circle::types
