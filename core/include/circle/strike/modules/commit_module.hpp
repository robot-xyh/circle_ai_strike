#pragma once

#include <optional>

#include "circle/strike/strike_params.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/time.hpp"

namespace circle::strike {

struct CommitState {
  bool active{false};
  bool terminal_ready{false};
  FinalApproachCommitSnapshot snapshot{};
  FinalApproachCommitSnapshot recent_centered_snapshot{};
  std::optional<circle::types::TimestampNs> latch_start_time_ns;
  std::optional<circle::types::TimestampNs> align_since_ns;
};

struct CommitOutput {
  bool should_hold{false};
  bool expired{false};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float yaw_rate_rad_s{0.0F};
  float thrust_z{0.0F};
  float thrust_scalar{0.0F};
  float command_age_s{0.0F};
};

class CommitModule {
 public:
  void reset();

  CommitOutput computeHold(
      CommitState& state,
      const FACommitParams& params,
      const ThrustParams& thrust_params,
      float hover_thrust_scalar,
      float lateral_output_sign,
      float longitudinal_output_sign,
      float vehicle_roll_rad,
      float vehicle_pitch_rad,
      float max_roll_angle_rad,
      float max_pitch_angle_rad,
      float tilt_softcap_band_rad,
      bool yaw_lock_enabled,
      double detection_age_s,
      circle::types::TimestampNs now_ns) const;

  bool shouldCreateSnapshot(
      const CommitState& state,
      const FACommitParams& params,
      bool final_approach_active,
      bool measure_reliable,
      bool strike_confident,
      float bbox_area_ratio,
      float bbox_margin_x_px,
      float bbox_margin_y_px,
      float align_error_x_px,
      float align_error_y_px,
      float align_rate_x_px_s,
      float align_rate_y_px_s,
      float future_error_x_px,
      float future_error_y_px,
      float vehicle_roll_rad,
      float vehicle_pitch_rad,
      bool edge_protect_active,
      circle::types::TimestampNs now_ns) const;

  bool shouldCreateBlindSnapshot(
      const CommitState& state,
      const FACommitParams& params,
      bool final_approach_active,
      bool blind_commit_trigger,
      float bbox_area_ratio,
      float bbox_margin_x_px,
      float bbox_margin_y_px,
      float align_error_x_px,
      float align_error_y_px,
      float align_rate_x_px_s,
      float align_rate_y_px_s,
      float ex_dot_filt,
      float ey_dot_filt,
      float detection_score,
      circle::types::TimestampNs now_ns) const;

  void updateSnapshot(
      CommitState& state,
      bool blind_terminal,
      float roll_rate_sp_rad_s,
      float pitch_rate_sp_rad_s,
      float yaw_rate_sp_rad_s,
      float thrust_z,
      float bbox_area_ratio,
      float bbox_margin_x_px,
      float bbox_margin_y_px,
      float align_error_x_px,
      float align_error_y_px,
      float ex,
      float ey,
      float ex_dot,
      float ey_dot,
      circle::types::TimestampNs now_ns,
      circle::types::TimestampNs detection_stamp_ns) const;

  bool checkTerminalReady(
      const CommitState& state,
      const FACommitParams& params,
      bool final_approach_active,
      float bbox_area_ratio,
      float align_error_x_px,
      float align_error_y_px,
      circle::types::TimestampNs now_ns) const;
};

}  // namespace circle::strike
