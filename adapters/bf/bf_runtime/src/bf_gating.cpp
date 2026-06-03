#include "circle/bf/runtime/bf_gating.hpp"

namespace circle::bf::runtime {

BfPublishMode decideBfPublish(BfControlResult::Command cmd, bool may_command,
                              bool override_active, bool dry_run,
                              bool state_watchdog_tripped, bool require_fresh,
                              bool fresh_seen) {
  const bool base_command_gate =
      may_command && !dry_run && override_active && !state_watchdog_tripped;
  if (!base_command_gate) {
    return BfPublishMode::PhysicalHold;
  }
  const bool fresh_ok = !require_fresh || fresh_seen;
  if (cmd == BfControlResult::Command::Algorithm && fresh_ok) {
    return BfPublishMode::Algorithm;
  }
  // LevelOnly bypasses the fresh-detection gate (anti-flip recovery).
  if (cmd == BfControlResult::Command::LevelOnly) {
    return BfPublishMode::LevelOnly;
  }
  return BfPublishMode::PhysicalHold;
}

float blendThrottleHandover(float from_norm, float target_norm, float alpha) {
  return (1.0F - alpha) * from_norm + alpha * target_norm;
}

}  // namespace circle::bf::runtime
