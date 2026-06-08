#pragma once

#include <cstdint>
#include <string>

#include "circle/strike_png/entry_handoff.hpp"
#include "circle/strike_png/strike_png_controller.hpp"

namespace circle::strike_png {

/**
 * Node-level PNG parameters mirroring the PX4 target_strike_png node Params:
 * detection gating + thrust policy + target-loss hold + entry handoff + the
 * StrikePngController params. Shared by the BF flight executable and the debug
 * tuning bridge so both stay in lockstep with the ROS2 controller.
 */
struct StrikePngNodeParams {
  bool dry_run{false};
  bool require_armed_to_command{true};
  std::string target_class_name{"UAV"};
  float min_score{0.10F};
  double detection_stale_s{0.35};
  float hover_thrust_z{0.58F};
  float strike_thrust_z{0.78F};
  bool target_lost_hold_enable{true};
  double target_lost_hold_delay_s{0.05};
  bool derotate_history_enable{true};
  int64_t camera_exposure_midpoint_offset_ns{0};
  int64_t fc_serial_latency_ns{0};
  float max_derotate_interpolation_gap_s{0.02F};
  bool body_rate_observer_enable{false};
  EntryHandoffParams entry_handoff{};
  StrikePngParams controller{};
};

}  // namespace circle::strike_png
