#pragma once

#include <optional>

#include "circle/strike/strike_params.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/time.hpp"

namespace circle::strike {

struct PreclimbState {
  bool xy_gate_released{false};
  std::optional<circle::types::TimestampNs> xy_stable_since_ns;
  std::optional<circle::types::TimestampNs> xy_released_since_ns;
};

struct PreclimbOutput {
  bool xy_gate_active{false};
  bool safe_hold_active{false};
  bool level_assist_active{false};
  bool release_slowdown_active{false};

  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float thrust_scale{1.0F};

  float safe_hold_blend{0.0F};
  float level_assist_blend{0.0F};
  float release_slowdown_scale{1.0F};

  bool error_ok{true};
  bool rate_ok{true};
  bool margin_ok{true};
  bool level_ok{true};
  float error_x_px{0.0F};
  float error_y_px{0.0F};
  float rate_x_px_s{0.0F};
  float rate_y_px_s{0.0F};
  double hold_elapsed_s{0.0};
};

class PreclimbModule {
 public:
  void reset();

  PreclimbOutput compute(
      PreclimbState& state,
      const PreclimbParams& params,
      bool final_approach_active,
      bool fresh_detection,
      bool strike_confident,
      float image_ex, float image_ey,
      float ex_dot_filt, float ey_dot_filt,
      float fx, float fy,
      float bbox_margin_x_px, float bbox_margin_y_px,
      float vehicle_roll_rad, float vehicle_pitch_rad,
      float roll_rate_des, float pitch_rate_des,
      circle::types::TimestampNs now_ns) const;

  static float computeReleaseSlowdown(
      const PreclimbParams& params,
      const PreclimbState& state,
      circle::types::TimestampNs now_ns);
};

}  // namespace circle::strike
