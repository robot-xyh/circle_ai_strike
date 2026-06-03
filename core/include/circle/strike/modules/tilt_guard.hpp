#pragma once

#include "circle/strike/strike_params.hpp"
#include "circle/types/fc_state.hpp"

namespace circle::strike {

struct TiltGuardOutput {
  bool active{false};
  float roll_rate_correction{0.0F};
  float pitch_rate_correction{0.0F};
  float tilt_angle{0.0F};
  float max_edge_ratio{0.0F};
};

class TiltGuard {
 public:
  TiltGuardOutput compute(
      const TiltGuardParams& params,
      const circle::types::FcState& vehicle,
      float current_roll_rate,
      float current_pitch_rate,
      float max_edge_ratio) const;
};

}  // namespace circle::strike
