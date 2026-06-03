#pragma once

#include "circle/bf/runtime/bf_strike_controller_iface.hpp"

namespace circle::bf::runtime {

/** Resolved wire decision for one control tick. */
enum class BfPublishMode { PhysicalHold, Algorithm, LevelOnly };

/**
 * Pure gating decision (no MSP / threads), mirroring bf_flight's
 * algorithm_command_active / force_level_command_active / physical-hold logic.
 *
 *  - PhysicalHold unless OVERRIDE is active, not dry_run, not watchdog-tripped
 *    and the controller is allowed to command (may_command).
 *  - Algorithm when the controller asks for Algorithm AND the fresh-detection
 *    gate is satisfied.
 *  - LevelOnly when the controller asks for LevelOnly (bypasses the fresh gate,
 *    matching the anti-flip ForceLevel path).
 */
BfPublishMode decideBfPublish(BfControlResult::Command cmd, bool may_command,
                              bool override_active, bool dry_run,
                              bool state_watchdog_tripped, bool require_fresh,
                              bool fresh_seen);

/** Linear throttle handover blend; alpha in [0,1]. */
float blendThrottleHandover(float from_norm, float target_norm, float alpha);

}  // namespace circle::bf::runtime
