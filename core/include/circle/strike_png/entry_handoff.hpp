#pragma once

#include <algorithm>
#include <cstdint>

namespace circle::strike_png {

struct EntryHandoffParams {
  bool enable{true};
  float duration_s{0.8F};
  float initial_thrust_z{0.58F};
};

struct EntryHandoffSnapshot {
  uint64_t activation_ns{0};
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float thrust_z{0.58F};
};

struct EntryHandoffCommand {
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float thrust_z{0.0F};
  bool active{false};
};

inline float entryHandoffSmoothstep(float value) {
  const float x = std::clamp(value, 0.0F, 1.0F);
  return x * x * (3.0F - 2.0F * x);
}

inline float entryHandoffLerp(float a, float b, float t) {
  return a + (b - a) * t;
}

inline EntryHandoffCommand applyEntryHandoff(
    const EntryHandoffParams& params,
    const EntryHandoffSnapshot& snapshot,
    const EntryHandoffCommand& target,
    uint64_t now_ns) {
  if (!params.enable || snapshot.activation_ns == 0U ||
      params.duration_s <= 0.0F || now_ns <= snapshot.activation_ns) {
    if (params.enable && snapshot.activation_ns != 0U &&
        params.duration_s > 0.0F && now_ns == snapshot.activation_ns) {
      return EntryHandoffCommand{snapshot.roll_rate_rad_s,
                                 snapshot.pitch_rate_rad_s,
                                 snapshot.thrust_z,
                                 true};
    }
    return target;
  }

  constexpr float kNsToS = 1.0e-9F;
  const float elapsed_s =
      static_cast<float>(now_ns - snapshot.activation_ns) * kNsToS;
  const float progress =
      entryHandoffSmoothstep(elapsed_s / std::max(1.0e-3F, params.duration_s));
  if (progress >= 1.0F) {
    return target;
  }

  return EntryHandoffCommand{
      entryHandoffLerp(snapshot.roll_rate_rad_s,
                       target.roll_rate_rad_s,
                       progress),
      entryHandoffLerp(snapshot.pitch_rate_rad_s,
                       target.pitch_rate_rad_s,
                       progress),
      entryHandoffLerp(snapshot.thrust_z, target.thrust_z, progress),
      true};
}

}  // namespace circle::strike_png
