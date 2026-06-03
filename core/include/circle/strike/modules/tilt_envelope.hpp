#pragma once

namespace circle::strike {

// Generic, reusable tilt safety envelope. Applied on a final vehicle-frame rate
// command using measured attitude. Two stages, both continuous (no step):
//   1. Soft cap: scales the tilt-increasing rate toward 0 across
//      [max_angle - softcap_band, max_angle].
//   2. Hard cap (margin blend): over [max_angle, max_angle + hardcap_margin],
//      smoothstep-blends the command toward a leveling rate (-kp * attitude),
//      reaching full leveling at max_angle + hardcap_margin.
// An optional output LPF + jerk limiter smooths command-side jumps.
// All angle/rate inputs are in radians / rad-s.
struct TiltEnvelopeParams {
  bool enable{false};
  float max_roll_angle_rad{0.5236F};
  float max_pitch_angle_rad{0.6981F};
  float softcap_band_rad{0.1745F};
  float hardcap_margin_rad{0.1047F};
  float hardcap_level_kp{3.0F};
  float max_level_rate_rad_s{1.5F};
  float out_lpf_tau_s{0.0F};       // 0 = disabled
  float out_max_jerk_rad_s2{0.0F}; // 0 = disabled
};

struct TiltEnvelopeState {
  float roll_filt{0.0F};
  float roll_slew{0.0F};
  float pitch_filt{0.0F};
  float pitch_slew{0.0F};
};

struct TiltEnvelopeOutput {
  float roll_rate_rad_s{0.0F};
  float pitch_rate_rad_s{0.0F};
  float roll_softcap_factor{1.0F};
  float pitch_softcap_factor{1.0F};
  float roll_level_weight{0.0F};
  float pitch_level_weight{0.0F};
  bool hardcap_active{false};
};

class TiltEnvelope {
 public:
  void reset();

  // roll_cmd/pitch_cmd: incoming rate command (rad/s). roll_att/pitch_att:
  // measured attitude (rad). dt_s: control step for the output smoother.
  TiltEnvelopeOutput compute(const TiltEnvelopeParams& params, float roll_cmd,
                             float pitch_cmd, float roll_att, float pitch_att,
                             float dt_s);

 private:
  TiltEnvelopeState state_{};
};

}  // namespace circle::strike
