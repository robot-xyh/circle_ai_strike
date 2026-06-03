#pragma once

#include "circle/strike/strike_params.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/detection.hpp"

namespace circle::strike {

struct DirectionalDiveState {
  float dive_blend{0.0F};
};

struct DirectionalDiveOutput {
  bool active{false};
  float dive_blend{0.0F};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float thrust_z{0.0F};
};

class DirectionalDive {
 public:
  void reset();

  DirectionalDiveOutput compute(
      DirectionalDiveState& state,
      const DirectionalDiveParams& params,
      const TiltGuardParams& tilt_guard_params,
      bool final_approach_active,
      float max_edge_ratio,
      float tilt_angle,
      float image_ex,
      float image_ey,
      float ex_dot_filt,
      float ey_dot_filt,
      float vehicle_pitch_rad,
      float vehicle_roll_rad,
      float gov_scale,
      float current_thrust_z,
      float current_roll_rate,
      float current_pitch_rate,
      circle::types::TimestampNs detection_capture_ns,
      circle::types::TimestampNs now_ns,
      float dt_s) const;
};

}  // namespace circle::strike
