#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace circle::ipc {

struct StrikeTelemetrySample {
  uint64_t seq{0};
  int64_t stamp_ns{0};
  float roll_rate_sp{0.0F};
  float pitch_rate_sp{0.0F};
  float yaw_rate_sp{0.0F};
  float thrust_z{0.0F};
  float ex{0.0F};
  float ey{0.0F};
  int32_t state{0};
  uint8_t has_target{0};
  uint8_t armed{0};
  float vehicle_roll_rad{0.0F};
  float vehicle_pitch_rad{0.0F};
  float vehicle_yaw_rad{0.0F};
  float vehicle_roll_rate_rad_s{0.0F};
  float vehicle_pitch_rate_rad_s{0.0F};
  float vehicle_yaw_rate_rad_s{0.0F};
  float throttle_pwm{0.0F};
  float throttle_norm{0.0F};
  /** Algorithm mapRates throttle (0..1), independent of dry_run / passthrough. */
  float throttle_algo_norm{0.0F};
  /** Last MSP_SET_RAW_RC wire throttle (0..1); NaN when no successful send yet. */
  float throttle_cmd_norm{0.0F};
  uint8_t throttle_cmd_valid{0};
  uint8_t vehicle_valid{0};
  uint8_t dry_run_passthrough{0};
  
  float ex_dot_filt{0.0F};
  float ey_dot_filt{0.0F};
  float e_rho{0.0F};
  float e_rho_dot_filt{0.0F};
  float rho_scale{1.0F};
  float roll_softcap_factor{1.0F};
  float pitch_softcap_factor{1.0F};
  float roll_hard_headroom_rad{0.0F};
  float pitch_hard_headroom_rad{0.0F};
  uint8_t final_approach_active{0};
  float aim_comp_x_px{0.0F};
  float aim_comp_y_px{0.0F};
  float tracking_thrust_scalar_smooth{0.0F};
  float tracking_thrust_scalar_target{0.0F};
  float deadband_eff_half_w_px{0.0F};
  float deadband_eff_half_h_px{0.0F};
  float fa_kp_scale{1.0F};
  float fa_kd_scale{1.0F};
  float bbox_area_ratio{0.0F};
  float detection_score{0.0F};
  uint8_t tracker_fallback_active{0};
  uint8_t strike_confident{0};
  uint8_t preclimb_xy_gate_active{0};
  float speed_governor_blend{0.0F};
  float speed_governor_scale{1.0F};
  float fa_thrust_taper_scale{1.0F};
  /** Per-frame vision detection validity (not coasted has_valid_target). */
  uint8_t detection_valid{0};
  /** MSP OVERRIDE mode active on FC. */
  uint8_t msp_override_active{0};

  /** Controller kind for series schema selection: 0=target_strike, 1=target_strike_png. */
  uint8_t controller_kind{0};

  // --- PNG (target_strike_png) guidance signals; valid only when controller_kind==1 ---
  float png_closure_scale{1.0F};
  float png_ex_dot_inertial{0.0F};
  float png_ey_dot_inertial{0.0F};
  float png_measurement_age_s{0.0F};
  float png_ff_roll_rad_s{0.0F};
  float png_ff_pitch_rad_s{0.0F};
  float png_fov_trim_roll_rad_s{0.0F};
  float png_fov_trim_pitch_rad_s{0.0F};
  float png_edge_guard_roll_rad_s{0.0F};
  float png_edge_guard_pitch_rad_s{0.0F};
  float png_pursuit_roll_rad_s{0.0F};
  float png_pursuit_pitch_rad_s{0.0F};
  float png_stale_trim_roll_rad_s{0.0F};
  float png_intercept_roll_rad_s{0.0F};
  float png_intercept_pitch_rad_s{0.0F};
  float png_crossing_pitch_rad_s{0.0F};
  float png_future_ex{0.0F};
  float png_future_ey{0.0F};
  float png_intercept_lead_s{0.0F};
  float png_crossing_weight{0.0F};
  float png_fwd_guard_scale{1.0F};
  float png_entry_handoff_progress{1.0F};
  float png_tilt_softcap_roll{1.0F};
  float png_tilt_softcap_pitch{1.0F};
  float png_derotate_lookup_age_ms{0.0F};
  float png_derotate_interp_gap_ms{0.0F};
  float png_derotate_roll_rate_rad_s{0.0F};
  float png_derotate_pitch_rate_rad_s{0.0F};
  float png_camera_exposure_midpoint_offset_ns{0.0F};
  float png_fc_serial_latency_ns{0.0F};
  uint8_t png_derotate_lookup_valid{0};
  uint8_t png_intercept_active{0};
  uint8_t png_crossing_active{0};
  uint8_t png_fwd_guard_active{0};
  uint8_t png_loss_hold_latched{0};
  uint8_t png_tilt_hardcap_active{0};
};

class StrikeTelemetryWriter {
 public:
  ~StrikeTelemetryWriter();
  bool open(const std::string& name, size_t sample_capacity = 4096);
  void close();
  void publish(const StrikeTelemetrySample& sample);

 private:
  struct Header;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  Header* header_{nullptr};
  StrikeTelemetrySample* samples_{nullptr};
  std::mutex mu_;
};

class StrikeTelemetryReader {
 public:
  ~StrikeTelemetryReader();
  bool open(const std::string& name);
  void close();
  bool isOpen() const { return header_ != nullptr; }
  bool readLatest(StrikeTelemetrySample& out);
  std::string seriesJson() const;

 private:
  struct Header;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  const Header* header_{nullptr};
  const StrikeTelemetrySample* samples_{nullptr};
  uint64_t last_seq_{0};
};

}  // namespace circle::ipc
