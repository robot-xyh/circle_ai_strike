#pragma once

#include <optional>

#include "circle/strike/strike_params.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/time.hpp"

namespace circle::strike {

struct ThrustManagerState {
  float tracking_thrust_scalar_smooth{std::numeric_limits<float>::quiet_NaN()};
  float published_thrust_scalar_smooth{std::numeric_limits<float>::quiet_NaN()};
  std::optional<circle::types::TimestampNs> slew_last_publish_time_ns;
};

struct ThrustManagerOutput {
  float thrust_z{0.0F};
  float base_target{0.0F};
  float thrust_scalar_smooth{0.0F};
  float tilt_denom{1.0F};

  float fa_thrust_taper_scale{1.0F};
  float fa_unaligned_slowdown_scale{1.0F};
  float fa_tilt_slowdown_scale{1.0F};
  float fa_vertical_drift_slowdown_scale{1.0F};
  float fa_ascent_budget_scale{1.0F};
  float fa_ascent_budget_tilt_score{0.0F};
  float fa_ascent_budget_y_rate_score{0.0F};
  float fa_ascent_budget_y_error_score{0.0F};
  bool fa_ascent_budget_active{false};
  bool fa_vertical_drift_slowdown_active{false};

  float preclimb_release_slowdown_scale{1.0F};
  bool preclimb_release_slowdown_active{false};
  bool preclimb_thrust_gated{false};

  float ascent_image_velocity_damping_scale{1.0F};
  bool ascent_image_velocity_damping_active{false};

  float tracking_deadband_priority_scale{1.0F};
  bool tracking_deadband_priority_active{false};
};

class ThrustManager {
 public:
  void reset();

  ThrustManagerOutput compute(
      ThrustManagerState& state,
      const StrikeParams& params,
      bool final_approach_active,
      bool edge_protect_active,
      float edge_thrust_scale,
      float edge_taper_score,
      bool preclimb_xy_gate_active,
      bool preclimb_xy_gate_held,
      const std::optional<circle::types::TimestampNs>& preclimb_xy_released_since_ns,
      bool fresh_detection,
      float bbox_area_ratio,
      float image_ex, float image_ey,
      float ex_dot_filt, float ey_dot_filt,
      float fx, float fy,
      float vehicle_roll_rad, float vehicle_pitch_rad,
      float deadband_eff_half_w_px, float deadband_eff_half_h_px,
      float align_error_x_px, float align_error_y_px,
      bool commit_aligned,
      circle::types::TimestampNs now_ns,
      float dt_s) const;
};

}  // namespace circle::strike
