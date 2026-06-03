#pragma once

#include "circle/strike/strike_params.hpp"

namespace circle::strike {

struct SpeedGovernorOutput {
  bool active{false};
  float blend{0.0F};
  float scale{1.0F};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
};

class SpeedGovernor {
 public:
  SpeedGovernorOutput compute(
      const SpeedGovernorParams& params,
      bool final_approach_active,
      bool vehicle_velocity_valid,
      float vehicle_vxy_m_s,
      float vehicle_roll_rad,
      float vehicle_pitch_rad,
      float roll_rate_des,
      float pitch_rate_des) const;
};

}  // namespace circle::strike
