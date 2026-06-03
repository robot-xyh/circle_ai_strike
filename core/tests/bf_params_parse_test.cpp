// Verifies that every parseable `strike.*` parameter in the BF YAML loader
// (loadStrikeParamsFromYaml) actually takes effect, i.e. no key is silently
// dropped to a struct default.
//
// Part 1 (testRealBfConfig): loads the shipped config/strike_bf_flight.yaml and
//   asserts the intended values across every section. Guards against regressions
//   in the real flight config and proves the file parses as intended.
//
// Part 2 (testEveryStrikeKeyTakesEffect): writes a temporary YAML that sets EVERY
//   key the loader reads to a NON-default sentinel value, parses it, and asserts
//   each struct field equals its sentinel. Because each sentinel differs from the
//   struct default, a passing assertion proves the key was genuinely parsed.
//
// Part 3 (testRealBfFlightSections): loads bf_flight.msp + bf_flight.rc from the
//   shipped config and asserts intended values (including override debounce/grace).
//
// Part 4 (testEveryBfFlightSectionKeyTakesEffect): sentinel YAML for every
//   bf_flight.msp.* and bf_flight.rc.* key parsed by bf_flight_config_yaml.
//
// NOTE: other bf_flight.* keys (camera, watchdog, etc.) are parsed inline in
//   adapters/bf/bf_flight/src/main.cpp and are not covered here.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "circle/strike/strike_params.hpp"
#include "circle/strike/strike_params_yaml.hpp"
#include "circle/bf/bf_flight_config_yaml.hpp"

#ifndef STRIKE_BF_CONFIG_PATH
#define STRIKE_BF_CONFIG_PATH ""
#endif

namespace {

int g_failures = 0;

template <typename A, typename B>
void expectEq(const A& a, const B& b, const char* expr, int line) {
  if (!(a == static_cast<A>(b))) {
    std::cerr << "FAIL L" << line << ": " << expr << " got=" << a
              << " want=" << b << "\n";
    ++g_failures;
  }
}

void expectNear(double a, double b, const char* expr, int line) {
  if (std::fabs(a - b) > 1e-6) {
    std::cerr << "FAIL L" << line << ": " << expr << " got=" << a
              << " want=" << b << "\n";
    ++g_failures;
  }
}

#define EXPECT_EQ(actual, want) expectEq((actual), (want), #actual, __LINE__)
#define EXPECT_NEAR(actual, want) \
  expectNear(static_cast<double>(actual), static_cast<double>(want), #actual, __LINE__)

std::string writeTempYaml(const std::string& content) {
  const std::string path = "bf_params_sentinel.yaml";
  std::ofstream ofs(path, std::ios::trunc);
  ofs << content;
  ofs.close();
  return path;
}

// --------------------------------------------------------------------------
// Part 1: the real shipped BF config parses to its intended values.
// --------------------------------------------------------------------------
void testRealBfConfig() {
  const std::string path = STRIKE_BF_CONFIG_PATH;
  if (path.empty()) {
    std::cerr << "FAIL: STRIKE_BF_CONFIG_PATH not defined\n";
    ++g_failures;
    return;
  }
  std::ifstream probe(path);
  if (!probe.good()) {
    std::cerr << "FAIL: cannot open BF config: " << path << "\n";
    ++g_failures;
    return;
  }
  probe.close();

  const auto p = circle::strike::loadStrikeParamsFromYaml(path);

  // filter + temporal_gating (nested)
  EXPECT_NEAR(p.filter.min_score, 0.30);
  EXPECT_NEAR(p.filter.min_bbox_area, 120.0);
  EXPECT_NEAR(p.filter.max_bbox_aspect_ratio, 5.0);
  EXPECT_EQ(p.filter.target_class_name, std::string("UAV"));
  EXPECT_EQ(p.filter.target_class_names.size(), size_t(1));
  EXPECT_EQ(p.filter.temporal_gating_enabled, true);   // nested temporal_gating.enabled
  EXPECT_NEAR(p.filter.gate_radius_px, 220.0);          // nested
  EXPECT_NEAR(p.filter.reacquire_area_ratio, 0.4);      // nested

  EXPECT_NEAR(p.detection_stale_s, 0.8);
  EXPECT_NEAR(p.lost_timeout_s, 2.0);
  EXPECT_EQ(p.dkf_enable, true);                        // top-level switch (the one the controller reads)

  EXPECT_NEAR(p.dkf.process_accel_noise, 4.0);
  EXPECT_NEAR(p.dkf.meas_noise_px, 4.0);
  EXPECT_NEAR(p.dkf.max_cov_trace, 0.25);

  // newly-added / previously-buggy sections (core of the alignment fix)
  EXPECT_NEAR(p.rho_scale.desired_bbox_area_px, 800.0);
  EXPECT_NEAR(p.rho_scale.scale_far, 1.0);              // was defaulting to 0.35
  EXPECT_NEAR(p.rho_scale.near_ratio, 2.01);
  EXPECT_NEAR(p.rho_scale.far_ratio, 0.50);

  EXPECT_EQ(p.preclimb.xy_gate_enable, true);           // the headline fix
  EXPECT_NEAR(p.preclimb.xy_error_gate_x_px, 220.0);
  EXPECT_NEAR(p.preclimb.thrust_excess_scale, 0.18);
  EXPECT_NEAR(p.preclimb.min_thrust_scalar, 0.337);
  EXPECT_EQ(p.preclimb.thrust_hard_cap_enable, true);

  EXPECT_EQ(p.tracking_deadband_priority.enable, true);
  EXPECT_NEAR(p.tracking_deadband_priority.min_excess_scale, 0.35);
  EXPECT_EQ(p.ascent_damping.image_velocity_damping_enable, true);
  EXPECT_NEAR(p.ascent_damping.image_velocity_damping_start_px_s, 180.0);

  EXPECT_EQ(p.approach_drive.enable, true);
  EXPECT_NEAR(p.approach_drive.pitch_output_sign, 1.0);
  EXPECT_EQ(p.image_lead.enable, true);
  EXPECT_NEAR(p.image_lead.time_s, 0.16);
  EXPECT_EQ(p.tracking_start_smoothing.enable, true);
  EXPECT_NEAR(p.tracking_start_smoothing.smoothing_s, 0.30);

  EXPECT_EQ(p.tracker_fallback.enable, true);
  EXPECT_NEAR(p.tracker_fallback.max_cov_trace, 0.30);
  EXPECT_EQ(p.confidence.gate_enable, true);
  EXPECT_NEAR(p.confidence.max_cov_trace, 0.60);

  EXPECT_EQ(p.yaw.lock_enabled, true);
  EXPECT_NEAR(p.yaw.track_deadband_rad, 0.0524);

  // tilt_cap / thrust additions
  EXPECT_NEAR(p.tilt_cap.softcap_band_rad, 0.20);
  EXPECT_NEAR(p.tilt_cap.hardcap_margin_rad, 0.10);
  EXPECT_EQ(p.thrust.enable_tilt_compensation, true);
  EXPECT_NEAR(p.thrust.hover_scalar, 0.283);
  EXPECT_NEAR(p.thrust.constant_scalar, 0.5);
  EXPECT_NEAR(p.thrust.tracking_ramp_tau_s, 0.35);
  EXPECT_NEAR(p.thrust.slew_rate_scalar_s, 1.0);
  EXPECT_NEAR(p.thrust.scalar_min, 0.01);
  EXPECT_NEAR(p.thrust.scalar_max, 0.99);

  // waiting / force_level
  EXPECT_EQ(p.waiting.level_hold_enabled, false);
  EXPECT_NEAR(p.waiting.level_kp, 1.5);
  EXPECT_NEAR(p.waiting.level_deadband_rad, 0.0087);
  EXPECT_EQ(p.force_level.min_hold_ms, 200);

  // target_loss / armed debounce additions
  EXPECT_EQ(p.target_loss.complete_on_loss_enable, true);
  EXPECT_NEAR(p.target_loss.loss_complete_s, 0.60);
  EXPECT_NEAR(p.target_loss.lost_target_rate_decay_tau_s, 0.50);
  EXPECT_EQ(p.armed_latch_ttl_ms, uint32_t(1500));
  EXPECT_EQ(p.armed_disarm_debounce_count, uint8_t(50));

  // directional_dive / tilt_guard
  EXPECT_EQ(p.directional_dive.enable, true);
  EXPECT_NEAR(p.directional_dive.start_edge_ratio, 0.05);
  EXPECT_EQ(p.tilt_guard.enable, false);

  // top-level gains
  EXPECT_NEAR(p.lateral_kp_rate, 4.0);
  EXPECT_NEAR(p.longitudinal_kp_rate, 4.0);
  EXPECT_NEAR(p.max_roll_rate_rad_s, 2.5);
  EXPECT_NEAR(p.max_pitch_rate_rad_s, 3.5);
  EXPECT_NEAR(p.longitudinal_output_sign, 1.0);

  // final_approach representative values (legacy thrust scalars intentionally kept)
  EXPECT_EQ(p.final_approach.gate.stable_gate_enable, true);
  EXPECT_NEAR(p.final_approach.gate.hold_s, 6.0);
  EXPECT_NEAR(p.final_approach.scaling.kp_scale, 1.35);
  EXPECT_NEAR(p.final_approach.commit.thrust_scalar, 0.55);
  EXPECT_NEAR(p.final_approach.thrust.min_thrust_scalar, 0.3);
  EXPECT_NEAR(p.final_approach.fallback.pitch_bias_rad, -0.10);
}

// --------------------------------------------------------------------------
// Part 3: bf_flight.msp + bf_flight.rc from the real shipped BF config.
// --------------------------------------------------------------------------
void testRealBfFlightSections() {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  const std::string path = STRIKE_BF_CONFIG_PATH;
  if (path.empty()) {
    std::cerr << "FAIL: STRIKE_BF_CONFIG_PATH not defined\n";
    ++g_failures;
    return;
  }
  const auto bf = circle::bf::loadBfFlightYamlSectionsFromFile(path);

  EXPECT_EQ(bf.msp.device, std::string("/dev/ttyS1"));
  EXPECT_EQ(bf.msp.baud, 115200);
  EXPECT_EQ(bf.msp.passthrough_in_dry_run, true);
  EXPECT_EQ(bf.msp.passthrough_log, true);
  EXPECT_NEAR(bf.msp.passthrough_log_interval_s, 1.0);
  EXPECT_EQ(bf.msp.passthrough_throttle_jump_pwm, uint16_t(40));
  EXPECT_NEAR(bf.msp.passthrough_hz, 50.0);
  EXPECT_NEAR(bf.msp.live_publish_hz, 100.0);
  EXPECT_NEAR(bf.msp.override_grace_hold_s, 0.35);
  EXPECT_EQ(bf.msp.override_mode_flag_auto, true);
  EXPECT_EQ(bf.msp.override_mode_flag, uint32_t(0x8000000));
  EXPECT_EQ(bf.msp.override_channels_mask, uint32_t(15));
  EXPECT_EQ(bf.msp.passthrough_channel_count, uint16_t(16));

  EXPECT_EQ(bf.rc.rc_min, uint16_t(1000));
  EXPECT_EQ(bf.rc.rc_mid, uint16_t(1500));
  EXPECT_EQ(bf.rc.rc_max, uint16_t(2000));
  EXPECT_EQ(bf.rc.roll_channel, uint16_t(0));
  EXPECT_EQ(bf.rc.pitch_channel, uint16_t(1));
  EXPECT_EQ(bf.rc.yaw_channel, uint16_t(2));
  EXPECT_EQ(bf.rc.throttle_channel, uint16_t(3));
  EXPECT_EQ(bf.rc.aux_arm_channel, uint16_t(4));
  EXPECT_NEAR(bf.rc.max_roll_rate_rad_s, 2.0);
  EXPECT_NEAR(bf.rc.max_pitch_rate_rad_s, 2.0);
  EXPECT_NEAR(bf.rc.max_yaw_rate_rad_s, 0.7);
#else
  std::cerr << "SKIP: CIRCLE_STRIKE_HAS_YAML not defined\n";
#endif
}

// --------------------------------------------------------------------------
// Part 2: every key the loader reads takes effect (sentinel != default).
// --------------------------------------------------------------------------
const char* kSentinelYaml = R"YAML(
strike:
  min_score: 0.111
  min_bbox_area: 222.0
  max_bbox_aspect_ratio: 3.33
  target_class_name: SENTINEL
  target_class_names: ["AA", "BB"]
  temporal_gating:
    enabled: true
    gate_radius_px: 199.0
    reacquire_area_ratio: 0.71
  detection_stale_s: 0.444
  lost_timeout_s: 3.33
  dkf_enable: false
  dkf:
    enable: false
    process_accel_noise: 7.7
    meas_noise_px: 6.6
    predict_extra_delay_s: 0.099
    max_cov_trace: 0.88
  tracker_fallback:
    enable: false
    after_s: 0.21
    max_s: 0.71
    max_cov_trace: 0.91
    min_score: 0.41
  confidence:
    gate_enable: false
    min_score: 0.42
    max_cov_trace: 0.92
  image_lead:
    enable: true
    time_s: 0.17
    max_px: 161.0
  tracking_start_smoothing:
    enable: false
    smoothing_s: 0.31
    kp_scale_initial: 0.86
    lead_scale_initial: 0.71
    kd_scale_initial: 0.72
  rho_scale:
    desired_bbox_area_px: 801.0
    scale_near: 1.11
    scale_far: 0.99
    near_ratio: 2.02
    far_ratio: 0.51
  approach_drive:
    enable: true
    e_rho_deadband: 0.051
    pitch_rate_gain: 0.36
    pitch_rate_max_rad_s: 0.46
    pitch_output_sign: 1.0
  speed_governor:
    enable: true
    start_m_s: 31.5
    full_m_s: 41.5
    min_image_scale: 0.71
    level_kp: 1.01
    level_max_rad_s: 0.56
    fa_roll_level_blend_max: 0.01
    fa_pitch_level_blend_max: 0.81
    fa_gate_enable: true
    fa_max_vxy_m_s: 31.0
  preclimb:
    xy_gate_enable: true
    xy_error_gate_x_px: 221.0
    xy_error_gate_y_px: 191.0
    xy_rate_gate_x_px_s: 301.0
    xy_rate_gate_y_px_s: 361.0
    clear_margin_x_px: 81.0
    clear_margin_y_px: 61.0
    level_gate_enable: true
    level_max_roll_rad: 0.23
    level_max_pitch_rad: 0.25
    xy_hold_s: 0.11
    level_assist_enable: true
    level_assist_band_rad: 0.081
    level_assist_kp: 1.21
    level_assist_max_rad_s: 0.31
    safe_hold_enable: true
    safe_hold_level_kp: 1.22
    safe_hold_level_max_rad_s: 0.32
    safe_hold_image_rate_scale: 0.99
    safe_hold_max_rate_rad_s: 1.41
    safe_hold_tilt_start_rad: 0.17
    safe_hold_tilt_full_rad: 0.33
    thrust_excess_scale: 0.19
    min_thrust_scalar: 0.338
    thrust_hard_cap_enable: true
    release_slowdown_enable: true
    release_slowdown_s: 4.1
    release_thrust_excess_scale: 0.26
    release_min_thrust_scalar: 0.327
    release_thrust_hard_cap_enable: true
  ascent_damping:
    image_velocity_damping_enable: true
    image_velocity_damping_start_px_s: 181.0
    image_velocity_damping_min_scale: 0.56
  tracking_deadband_priority:
    enable: true
    error_start_scale: 1.01
    error_full_scale: 4.01
    rate_start_px_s: 181.0
    rate_full_px_s: 521.0
    min_excess_scale: 0.36
    hard_cap_enable: false
  final_approach:
    gate:
      boost_enable: true
      area_ratio_enter: 0.00221
      area_ratio_exit: 0.00171
      altitude_gate_enable: true
      altitude_gate_max_gap_m: 12.1
      area_quality_gate_enable: true
      area_quality_error_x_px: 121.0
      area_quality_error_y_px: 141.0
      area_quality_rate_x_px_s: 241.0
      area_quality_rate_y_px_s: 301.0
      area_quality_max_tilt_rad: 0.71
      tilt_gate_enable: true
      tilt_gate_min_area_ratio: 0.00201
      tilt_gate_start_rad: 0.261
      tilt_gate_error_x_px: 56.0
      tilt_gate_error_y_px: 71.0
      tilt_gate_rate_x_px_s: 81.0
      tilt_gate_rate_y_px_s: 131.0
      stable_gate_enable: true
      stable_gate_min_area_ratio: 0.00036
      stable_gate_max_area_ratio: 0.00241
      stable_gate_error_x_px: 61.0
      stable_gate_error_y_px: 76.0
      stable_gate_rate_x_px_s: 181.0
      stable_gate_rate_y_px_s: 241.0
      stable_gate_max_tilt_rad: 0.61
      stable_gate_hold_s: 0.121
      hold_s: 6.01
    scaling:
      jerk_scale: 1.11
      roll_rate_scale: 1.12
      kp_scale: 1.36
      kd_scale: 0.76
      pixel_dot_lpf_scale: 0.77
      rate_lpf_scale: 0.78
      deadband_scale: 0.21
      kp_proximity_gain: 0.31
      kd_proximity_gain: 0.32
      proximity_error_norm: false
      pitch_ff_erho_gain: 0.33
    leveling:
      roll_level_start_ratio: 0.026
      roll_level_end_ratio: 0.076
      roll_level_kp: 1.21
      roll_level_max_rad_s: 0.41
      pitch_level_start_ratio: 0.0011
      pitch_level_end_ratio: 0.0041
      pitch_level_kp: 2.01
      pitch_level_max_rad_s: 0.66
    pitch_chatter_guard:
      enable: true
      max_area_ratio: 0.00121
      max_error_y_px: 81.0
      max_rate_y_px_s: 261.0
      prev_min_rate_rad_s: 0.56
      max_reversal_rate_rad_s: 0.57
    tilt_aim_comp:
      enable: true
      start_ratio: 0.0061
      end_ratio: 0.0251
      gain: 0.71
      max_px: 161.0
      roll_sign: 0.5
      pitch_sign: 1.0
    thrust:
      tilt_slowdown_enable: true
      tilt_slowdown_start_rad: 0.451
      tilt_slowdown_end_rad: 0.701
      tilt_slowdown_min_scale: 0.551
      vertical_drift_slowdown_enable: true
      vertical_drift_start_px_s: 161.0
      vertical_drift_min_scale: 0.501
      ascent_budget_enable: true
      ascent_budget_tilt_start_rad: 0.321
      ascent_budget_tilt_full_rad: 0.601
      ascent_budget_y_rate_start_px_s: 181.0
      ascent_budget_y_rate_full_px_s: 421.0
      ascent_budget_y_error_start_px: 81.0
      ascent_budget_y_error_full_px: 221.0
      ascent_budget_min_excess_scale: 0.61
      ascent_budget_hard_cap_enable: false
      min_thrust_scalar: 0.078
      min_thrust_budget_relax_enable: true
      min_thrust_budget_relax_scale: 0.56
      thrust_taper_enable: false
      thrust_taper_start_ratio: 0.036
      thrust_taper_end_ratio: 0.101
      thrust_taper_min_scale: 0.76
      thrust_taper_area_enable: false
      thrust_taper_edge_enable: true
      thrust_taper_edge_start_score: 0.151
      thrust_taper_edge_full_score: 0.99
      thrust_taper_tilt_enable: true
      thrust_taper_tilt_start_rad: 0.381
      thrust_taper_tilt_full_rad: 0.701
      thrust_hard_cap_enable: true
      unaligned_slowdown_start_ratio: 0.011
      unaligned_slowdown_end_ratio: 0.061
      unaligned_thrust_excess_scale: 0.71
    edge_protect:
      enable: false
      margin_x_px: 221.0
      margin_y_px: 211.0
      predict_s: 0.41
      roll_kp_rate: 5.1
      pitch_kp_rate: 5.2
      pitch_boost_max_rad_s: 1.61
      thrust_scale: 0.76
    bottom_pitch_guard:
      enable: true
      margin_px: 121.0
      error_start_px: 71.0
      error_full_px: 141.0
      level_kp: 2.21
      max_rad_s: 0.76
    commit:
      enable: false
      command_hold_s: 2.01
      detection_stale_s: 1.21
      min_latch_s: 0.121
      thrust_scalar: 0.451
      min_area_ratio: 0.00181
      terminal_min_area_ratio: 0.00182
      terminal_max_error_x_px: 51.0
      terminal_max_error_y_px: 66.0
      stable_bypass_area_enable: true
      start_on_terminal_ready_enable: true
      recent_centered_handoff_enable: true
      recent_centered_handoff_min_area_ratio: 0.00101
      recent_centered_handoff_max_age_s: 2.51
      recent_centered_handoff_trigger_error_x_px: 86.0
      recent_centered_handoff_trigger_error_y_px: 91.0
      recent_centered_handoff_live_blend: 0.36
      recent_centered_hold_through_s: 0.01
      thrust_ramp_s: 0.251
      snapshot_max_area_ratio: 0.081
      align_gate_enable: false
      align_max_error_x_px: 61.0
      align_max_error_y_px: 76.0
      align_max_rate_x_px_s: 161.0
      align_max_rate_y_px_s: 221.0
      align_hold_s: 0.061
      tilt_gate_enable: true
      max_tilt_rad: 0.61
      blind_commit_enable: true
      blind_commit_min_area_ratio: 0.001
      blind_commit_edge_margin_x_px: 81.0
      blind_commit_edge_margin_y_px: 82.0
      blind_commit_edge_vertical_gate_enable: true
      blind_commit_edge_horizontal_gate_enable: true
      blind_commit_edge_max_error_x_px: 121.0
      blind_commit_edge_max_rate_x_px_s: 221.0
      blind_commit_edge_max_error_y_px: 161.0
      blind_commit_edge_max_rate_y_px_s: 361.0
      blind_commit_min_score: 0.61
      blind_commit_trend_lead_s: 0.221
      blind_commit_trend_max_rate_rad_s: 0.91
      predictive_enable: true
      predict_lead_s: 0.301
      predict_kp_rate: 3.01
      predict_kd_rate: 0.251
      predict_max_rate_rad_s: 1.01
      predict_blend_s: 0.351
      freeze_on_edge_protect: true
      forward_pitch_rate_rad_s: 0.05
      roll_level_kp: 1.21
      roll_level_max_rad_s: 0.451
      roll_level_blend: 0.11
      pitch_level_kp: 2.01
      pitch_level_max_rad_s: 0.651
      pitch_level_blend: 0.051
      yaw_lock_enabled: false
      min_margin_x_px: 31.0
      min_margin_y_px: 71.0
    fallback:
      decay_tau_s: 1.51
      max_s: 2.51
      pitch_bias_rad: -0.11
  yaw:
    bearing_kp: 0.61
    hold_gain: 0.51
    rate_min_rad_s: -0.71
    rate_max_rad_s: 0.72
    hold_deadband_rad: 0.0088
    track_deadband_rad: 0.0525
    rate_lpf_tau_s: 0.151
    lock_enabled: true
  waiting:
    level_hold_enabled: false
    level_kp: 1.51
    level_deadband_rad: 0.0089
    altitude_kp: 0.21
    altitude_kd: 0.11
    altitude_max_correction: 0.19
  force_level:
    hard_level_kp: 1.52
    min_hold_ms: 201
  tilt_cap:
    max_roll_angle_rad: 1.21
    max_pitch_angle_rad: 1.22
    softcap_band_rad: 0.21
    hardcap_margin_rad: 0.11
  thrust:
    enable_tilt_compensation: true
    hover_scalar: 0.284
    constant_scalar: 0.51
    tracking_ramp_tau_s: 0.36
    slew_rate_scalar_s: 1.01
    tilt_cos_floor: 0.52
    scalar_min: 0.021
    scalar_max: 0.981
  target_loss:
    complete_on_loss_enable: false
    loss_complete_s: 0.61
    lost_target_rate_decay_tau_s: 0.51
    complete_altitude_gate_enable: true
    complete_max_altitude_gap_m: 3.1
  evaluation:
    target_altitude_enable: true
    target_altitude_m: 61.0
    success_altitude_gap_m: 3.2
    relative_distance_enable: false
    relative_distance_max_age_s: 0.26
    target_origin_offset_x_m: 1.5
    target_origin_offset_y_m: 2.5
  tilt_guard:
    enable: true
    max_tilt_rad: 1.05
    recovery_kp: 0.81
  directional_dive:
    enable: true
    start_edge_ratio: 0.051
    end_edge_ratio: 0.101
    dive_roll_gain: 1.11
    dive_pitch_gain: 0.81
    climb_thrust: 0.451
    descend_thrust: 0.381
    cruise_thrust: 0.401
    lead_time_s: 0.101
    detection_max_age_s: 0.301
    min_horizontal_boost: 3.01
    tilt_comp_threshold: 0.016
    tilt_comp_gain: 0.51
  lateral_output_sign: 1.0
  longitudinal_output_sign: 1.0
  lateral_kp_rate: 4.01
  lateral_kd_rate: 0.881
  longitudinal_kp_rate: 5.01
  longitudinal_kd_rate: 0.882
  x_deadband: 0.021
  y_deadband: 0.022
  x_deadband_px: 13.0
  y_deadband_px: 14.0
  aim_offset_x_px: 1.0
  aim_offset_y_px: 2.0
  pixel_dot_lpf_tau_s: 0.081
  rate_lpf_tau_s: 0.051
  max_jerk_rad_s2: 81.0
  max_roll_rate_rad_s: 2.51
  max_pitch_rate_rad_s: 3.51
  rate_shaper_diag_log: true
  require_armed_to_command: false
  dry_run: true
  armed_latch_ttl_ms: 1499
  armed_disarm_debounce_count: 49
)YAML";

void testEveryStrikeKeyTakesEffect() {
  const std::string path = writeTempYaml(kSentinelYaml);
  const auto p = circle::strike::loadStrikeParamsFromYaml(path);
  std::remove(path.c_str());

  // filter + temporal_gating
  EXPECT_NEAR(p.filter.min_score, 0.111);
  EXPECT_NEAR(p.filter.min_bbox_area, 222.0);
  EXPECT_NEAR(p.filter.max_bbox_aspect_ratio, 3.33);
  EXPECT_EQ(p.filter.target_class_name, std::string("SENTINEL"));
  EXPECT_EQ(p.filter.target_class_names.size(), size_t(2));
  EXPECT_EQ(p.filter.temporal_gating_enabled, true);
  EXPECT_NEAR(p.filter.gate_radius_px, 199.0);
  EXPECT_NEAR(p.filter.reacquire_area_ratio, 0.71);

  EXPECT_NEAR(p.detection_stale_s, 0.444);
  EXPECT_NEAR(p.lost_timeout_s, 3.33);
  EXPECT_EQ(p.dkf_enable, false);

  EXPECT_EQ(p.dkf.enable, false);
  EXPECT_NEAR(p.dkf.process_accel_noise, 7.7);
  EXPECT_NEAR(p.dkf.meas_noise_px, 6.6);
  EXPECT_NEAR(p.dkf.predict_extra_delay_s, 0.099);
  EXPECT_NEAR(p.dkf.max_cov_trace, 0.88);

  EXPECT_EQ(p.tracker_fallback.enable, false);
  EXPECT_NEAR(p.tracker_fallback.after_s, 0.21);
  EXPECT_NEAR(p.tracker_fallback.max_s, 0.71);
  EXPECT_NEAR(p.tracker_fallback.max_cov_trace, 0.91);
  EXPECT_NEAR(p.tracker_fallback.min_score, 0.41);

  EXPECT_EQ(p.confidence.gate_enable, false);
  EXPECT_NEAR(p.confidence.min_score, 0.42);
  EXPECT_NEAR(p.confidence.max_cov_trace, 0.92);

  EXPECT_EQ(p.image_lead.enable, true);
  EXPECT_NEAR(p.image_lead.time_s, 0.17);
  EXPECT_NEAR(p.image_lead.max_px, 161.0);

  EXPECT_EQ(p.tracking_start_smoothing.enable, false);
  EXPECT_NEAR(p.tracking_start_smoothing.smoothing_s, 0.31);
  EXPECT_NEAR(p.tracking_start_smoothing.kp_scale_initial, 0.86);
  EXPECT_NEAR(p.tracking_start_smoothing.lead_scale_initial, 0.71);
  EXPECT_NEAR(p.tracking_start_smoothing.kd_scale_initial, 0.72);

  EXPECT_NEAR(p.rho_scale.desired_bbox_area_px, 801.0);
  EXPECT_NEAR(p.rho_scale.scale_near, 1.11);
  EXPECT_NEAR(p.rho_scale.scale_far, 0.99);
  EXPECT_NEAR(p.rho_scale.near_ratio, 2.02);
  EXPECT_NEAR(p.rho_scale.far_ratio, 0.51);

  EXPECT_EQ(p.approach_drive.enable, true);
  EXPECT_NEAR(p.approach_drive.e_rho_deadband, 0.051);
  EXPECT_NEAR(p.approach_drive.pitch_rate_gain, 0.36);
  EXPECT_NEAR(p.approach_drive.pitch_rate_max_rad_s, 0.46);
  EXPECT_NEAR(p.approach_drive.pitch_output_sign, 1.0);

  EXPECT_EQ(p.speed_governor.enable, true);
  EXPECT_NEAR(p.speed_governor.start_m_s, 31.5);
  EXPECT_NEAR(p.speed_governor.full_m_s, 41.5);
  EXPECT_NEAR(p.speed_governor.min_image_scale, 0.71);
  EXPECT_NEAR(p.speed_governor.level_kp, 1.01);
  EXPECT_NEAR(p.speed_governor.level_max_rad_s, 0.56);
  EXPECT_NEAR(p.speed_governor.fa_roll_level_blend_max, 0.01);
  EXPECT_NEAR(p.speed_governor.fa_pitch_level_blend_max, 0.81);
  EXPECT_EQ(p.speed_governor.fa_gate_enable, true);
  EXPECT_NEAR(p.speed_governor.fa_max_vxy_m_s, 31.0);

  EXPECT_EQ(p.preclimb.xy_gate_enable, true);
  EXPECT_NEAR(p.preclimb.xy_error_gate_x_px, 221.0);
  EXPECT_NEAR(p.preclimb.xy_error_gate_y_px, 191.0);
  EXPECT_NEAR(p.preclimb.xy_rate_gate_x_px_s, 301.0);
  EXPECT_NEAR(p.preclimb.xy_rate_gate_y_px_s, 361.0);
  EXPECT_NEAR(p.preclimb.clear_margin_x_px, 81.0);
  EXPECT_NEAR(p.preclimb.clear_margin_y_px, 61.0);
  EXPECT_EQ(p.preclimb.level_gate_enable, true);
  EXPECT_NEAR(p.preclimb.level_max_roll_rad, 0.23);
  EXPECT_NEAR(p.preclimb.level_max_pitch_rad, 0.25);
  EXPECT_NEAR(p.preclimb.xy_hold_s, 0.11);
  EXPECT_EQ(p.preclimb.level_assist_enable, true);
  EXPECT_NEAR(p.preclimb.level_assist_band_rad, 0.081);
  EXPECT_NEAR(p.preclimb.level_assist_kp, 1.21);
  EXPECT_NEAR(p.preclimb.level_assist_max_rad_s, 0.31);
  EXPECT_EQ(p.preclimb.safe_hold_enable, true);
  EXPECT_NEAR(p.preclimb.safe_hold_level_kp, 1.22);
  EXPECT_NEAR(p.preclimb.safe_hold_level_max_rad_s, 0.32);
  EXPECT_NEAR(p.preclimb.safe_hold_image_rate_scale, 0.99);
  EXPECT_NEAR(p.preclimb.safe_hold_max_rate_rad_s, 1.41);
  EXPECT_NEAR(p.preclimb.safe_hold_tilt_start_rad, 0.17);
  EXPECT_NEAR(p.preclimb.safe_hold_tilt_full_rad, 0.33);
  EXPECT_NEAR(p.preclimb.thrust_excess_scale, 0.19);
  EXPECT_NEAR(p.preclimb.min_thrust_scalar, 0.338);
  EXPECT_EQ(p.preclimb.thrust_hard_cap_enable, true);
  EXPECT_EQ(p.preclimb.release_slowdown_enable, true);
  EXPECT_NEAR(p.preclimb.release_slowdown_s, 4.1);
  EXPECT_NEAR(p.preclimb.release_thrust_excess_scale, 0.26);
  EXPECT_NEAR(p.preclimb.release_min_thrust_scalar, 0.327);
  EXPECT_EQ(p.preclimb.release_thrust_hard_cap_enable, true);

  EXPECT_EQ(p.ascent_damping.image_velocity_damping_enable, true);
  EXPECT_NEAR(p.ascent_damping.image_velocity_damping_start_px_s, 181.0);
  EXPECT_NEAR(p.ascent_damping.image_velocity_damping_min_scale, 0.56);

  EXPECT_EQ(p.tracking_deadband_priority.enable, true);
  EXPECT_NEAR(p.tracking_deadband_priority.error_start_scale, 1.01);
  EXPECT_NEAR(p.tracking_deadband_priority.error_full_scale, 4.01);
  EXPECT_NEAR(p.tracking_deadband_priority.rate_start_px_s, 181.0);
  EXPECT_NEAR(p.tracking_deadband_priority.rate_full_px_s, 521.0);
  EXPECT_NEAR(p.tracking_deadband_priority.min_excess_scale, 0.36);
  EXPECT_EQ(p.tracking_deadband_priority.hard_cap_enable, false);

  // final_approach.gate
  const auto& g = p.final_approach.gate;
  EXPECT_EQ(g.boost_enable, true);
  EXPECT_NEAR(g.area_ratio_enter, 0.00221);
  EXPECT_NEAR(g.area_ratio_exit, 0.00171);
  EXPECT_EQ(g.altitude_gate_enable, true);
  EXPECT_NEAR(g.altitude_gate_max_gap_m, 12.1);
  EXPECT_EQ(g.area_quality_gate_enable, true);
  EXPECT_NEAR(g.area_quality_error_x_px, 121.0);
  EXPECT_NEAR(g.area_quality_error_y_px, 141.0);
  EXPECT_NEAR(g.area_quality_rate_x_px_s, 241.0);
  EXPECT_NEAR(g.area_quality_rate_y_px_s, 301.0);
  EXPECT_NEAR(g.area_quality_max_tilt_rad, 0.71);
  EXPECT_EQ(g.tilt_gate_enable, true);
  EXPECT_NEAR(g.tilt_gate_min_area_ratio, 0.00201);
  EXPECT_NEAR(g.tilt_gate_start_rad, 0.261);
  EXPECT_NEAR(g.tilt_gate_error_x_px, 56.0);
  EXPECT_NEAR(g.tilt_gate_error_y_px, 71.0);
  EXPECT_NEAR(g.tilt_gate_rate_x_px_s, 81.0);
  EXPECT_NEAR(g.tilt_gate_rate_y_px_s, 131.0);
  EXPECT_EQ(g.stable_gate_enable, true);
  EXPECT_NEAR(g.stable_gate_min_area_ratio, 0.00036);
  EXPECT_NEAR(g.stable_gate_max_area_ratio, 0.00241);
  EXPECT_NEAR(g.stable_gate_error_x_px, 61.0);
  EXPECT_NEAR(g.stable_gate_error_y_px, 76.0);
  EXPECT_NEAR(g.stable_gate_rate_x_px_s, 181.0);
  EXPECT_NEAR(g.stable_gate_rate_y_px_s, 241.0);
  EXPECT_NEAR(g.stable_gate_max_tilt_rad, 0.61);
  EXPECT_NEAR(g.stable_gate_hold_s, 0.121);
  EXPECT_NEAR(g.hold_s, 6.01);

  // final_approach.scaling
  const auto& sc = p.final_approach.scaling;
  EXPECT_NEAR(sc.jerk_scale, 1.11);
  EXPECT_NEAR(sc.roll_rate_scale, 1.12);
  EXPECT_NEAR(sc.kp_scale, 1.36);
  EXPECT_NEAR(sc.kd_scale, 0.76);
  EXPECT_NEAR(sc.pixel_dot_lpf_scale, 0.77);
  EXPECT_NEAR(sc.rate_lpf_scale, 0.78);
  EXPECT_NEAR(sc.deadband_scale, 0.21);
  EXPECT_NEAR(sc.kp_proximity_gain, 0.31);
  EXPECT_NEAR(sc.kd_proximity_gain, 0.32);
  EXPECT_EQ(sc.proximity_error_norm, false);
  EXPECT_NEAR(sc.pitch_ff_erho_gain, 0.33);

  // final_approach.leveling
  const auto& lv = p.final_approach.leveling;
  EXPECT_NEAR(lv.roll_level_start_ratio, 0.026);
  EXPECT_NEAR(lv.roll_level_end_ratio, 0.076);
  EXPECT_NEAR(lv.roll_level_kp, 1.21);
  EXPECT_NEAR(lv.roll_level_max_rad_s, 0.41);
  EXPECT_NEAR(lv.pitch_level_start_ratio, 0.0011);
  EXPECT_NEAR(lv.pitch_level_end_ratio, 0.0041);
  EXPECT_NEAR(lv.pitch_level_kp, 2.01);
  EXPECT_NEAR(lv.pitch_level_max_rad_s, 0.66);

  // final_approach.pitch_chatter_guard
  const auto& pcg = p.final_approach.pitch_chatter_guard;
  EXPECT_EQ(pcg.enable, true);
  EXPECT_NEAR(pcg.max_area_ratio, 0.00121);
  EXPECT_NEAR(pcg.max_error_y_px, 81.0);
  EXPECT_NEAR(pcg.max_rate_y_px_s, 261.0);
  EXPECT_NEAR(pcg.prev_min_rate_rad_s, 0.56);
  EXPECT_NEAR(pcg.max_reversal_rate_rad_s, 0.57);

  // final_approach.tilt_aim_comp
  const auto& tac = p.final_approach.tilt_aim_comp;
  EXPECT_EQ(tac.enable, true);
  EXPECT_NEAR(tac.start_ratio, 0.0061);
  EXPECT_NEAR(tac.end_ratio, 0.0251);
  EXPECT_NEAR(tac.gain, 0.71);
  EXPECT_NEAR(tac.max_px, 161.0);
  EXPECT_NEAR(tac.roll_sign, 0.5);
  EXPECT_NEAR(tac.pitch_sign, 1.0);

  // final_approach.thrust
  const auto& ft = p.final_approach.thrust;
  EXPECT_EQ(ft.tilt_slowdown_enable, true);
  EXPECT_NEAR(ft.tilt_slowdown_start_rad, 0.451);
  EXPECT_NEAR(ft.tilt_slowdown_end_rad, 0.701);
  EXPECT_NEAR(ft.tilt_slowdown_min_scale, 0.551);
  EXPECT_EQ(ft.vertical_drift_slowdown_enable, true);
  EXPECT_NEAR(ft.vertical_drift_start_px_s, 161.0);
  EXPECT_NEAR(ft.vertical_drift_min_scale, 0.501);
  EXPECT_EQ(ft.ascent_budget_enable, true);
  EXPECT_NEAR(ft.ascent_budget_tilt_start_rad, 0.321);
  EXPECT_NEAR(ft.ascent_budget_tilt_full_rad, 0.601);
  EXPECT_NEAR(ft.ascent_budget_y_rate_start_px_s, 181.0);
  EXPECT_NEAR(ft.ascent_budget_y_rate_full_px_s, 421.0);
  EXPECT_NEAR(ft.ascent_budget_y_error_start_px, 81.0);
  EXPECT_NEAR(ft.ascent_budget_y_error_full_px, 221.0);
  EXPECT_NEAR(ft.ascent_budget_min_excess_scale, 0.61);
  EXPECT_EQ(ft.ascent_budget_hard_cap_enable, false);
  EXPECT_NEAR(ft.min_thrust_scalar, 0.078);
  EXPECT_EQ(ft.min_thrust_budget_relax_enable, true);
  EXPECT_NEAR(ft.min_thrust_budget_relax_scale, 0.56);
  EXPECT_EQ(ft.thrust_taper_enable, false);
  EXPECT_NEAR(ft.thrust_taper_start_ratio, 0.036);
  EXPECT_NEAR(ft.thrust_taper_end_ratio, 0.101);
  EXPECT_NEAR(ft.thrust_taper_min_scale, 0.76);
  EXPECT_EQ(ft.thrust_taper_area_enable, false);
  EXPECT_EQ(ft.thrust_taper_edge_enable, true);
  EXPECT_NEAR(ft.thrust_taper_edge_start_score, 0.151);
  EXPECT_NEAR(ft.thrust_taper_edge_full_score, 0.99);
  EXPECT_EQ(ft.thrust_taper_tilt_enable, true);
  EXPECT_NEAR(ft.thrust_taper_tilt_start_rad, 0.381);
  EXPECT_NEAR(ft.thrust_taper_tilt_full_rad, 0.701);
  EXPECT_EQ(ft.thrust_hard_cap_enable, true);
  EXPECT_NEAR(ft.unaligned_slowdown_start_ratio, 0.011);
  EXPECT_NEAR(ft.unaligned_slowdown_end_ratio, 0.061);
  EXPECT_NEAR(ft.unaligned_thrust_excess_scale, 0.71);

  // final_approach.edge_protect
  const auto& ep = p.final_approach.edge_protect;
  EXPECT_EQ(ep.enable, false);
  EXPECT_NEAR(ep.margin_x_px, 221.0);
  EXPECT_NEAR(ep.margin_y_px, 211.0);
  EXPECT_NEAR(ep.predict_s, 0.41);
  EXPECT_NEAR(ep.roll_kp_rate, 5.1);
  EXPECT_NEAR(ep.pitch_kp_rate, 5.2);
  EXPECT_NEAR(ep.pitch_boost_max_rad_s, 1.61);
  EXPECT_NEAR(ep.thrust_scale, 0.76);

  // final_approach.bottom_pitch_guard
  const auto& bpg = p.final_approach.bottom_pitch_guard;
  EXPECT_EQ(bpg.enable, true);
  EXPECT_NEAR(bpg.margin_px, 121.0);
  EXPECT_NEAR(bpg.error_start_px, 71.0);
  EXPECT_NEAR(bpg.error_full_px, 141.0);
  EXPECT_NEAR(bpg.level_kp, 2.21);
  EXPECT_NEAR(bpg.max_rad_s, 0.76);

  // final_approach.commit
  const auto& cm = p.final_approach.commit;
  EXPECT_EQ(cm.enable, false);
  EXPECT_NEAR(cm.command_hold_s, 2.01);
  EXPECT_NEAR(cm.detection_stale_s, 1.21);
  EXPECT_NEAR(cm.min_latch_s, 0.121);
  EXPECT_NEAR(cm.thrust_scalar, 0.451);
  EXPECT_NEAR(cm.min_area_ratio, 0.00181);
  EXPECT_NEAR(cm.terminal_min_area_ratio, 0.00182);
  EXPECT_NEAR(cm.terminal_max_error_x_px, 51.0);
  EXPECT_NEAR(cm.terminal_max_error_y_px, 66.0);
  EXPECT_EQ(cm.stable_bypass_area_enable, true);
  EXPECT_EQ(cm.start_on_terminal_ready_enable, true);
  EXPECT_EQ(cm.recent_centered_handoff_enable, true);
  EXPECT_NEAR(cm.recent_centered_handoff_min_area_ratio, 0.00101);
  EXPECT_NEAR(cm.recent_centered_handoff_max_age_s, 2.51);
  EXPECT_NEAR(cm.recent_centered_handoff_trigger_error_x_px, 86.0);
  EXPECT_NEAR(cm.recent_centered_handoff_trigger_error_y_px, 91.0);
  EXPECT_NEAR(cm.recent_centered_handoff_live_blend, 0.36);
  EXPECT_NEAR(cm.recent_centered_hold_through_s, 0.01);
  EXPECT_NEAR(cm.thrust_ramp_s, 0.251);
  EXPECT_NEAR(cm.snapshot_max_area_ratio, 0.081);
  EXPECT_EQ(cm.align_gate_enable, false);
  EXPECT_NEAR(cm.align_max_error_x_px, 61.0);
  EXPECT_NEAR(cm.align_max_error_y_px, 76.0);
  EXPECT_NEAR(cm.align_max_rate_x_px_s, 161.0);
  EXPECT_NEAR(cm.align_max_rate_y_px_s, 221.0);
  EXPECT_NEAR(cm.align_hold_s, 0.061);
  EXPECT_EQ(cm.tilt_gate_enable, true);
  EXPECT_NEAR(cm.max_tilt_rad, 0.61);
  EXPECT_EQ(cm.blind_commit_enable, true);
  EXPECT_NEAR(cm.blind_commit_min_area_ratio, 0.001);
  EXPECT_NEAR(cm.blind_commit_edge_margin_x_px, 81.0);
  EXPECT_NEAR(cm.blind_commit_edge_margin_y_px, 82.0);
  EXPECT_EQ(cm.blind_commit_edge_vertical_gate_enable, true);
  EXPECT_EQ(cm.blind_commit_edge_horizontal_gate_enable, true);
  EXPECT_NEAR(cm.blind_commit_edge_max_error_x_px, 121.0);
  EXPECT_NEAR(cm.blind_commit_edge_max_rate_x_px_s, 221.0);
  EXPECT_NEAR(cm.blind_commit_edge_max_error_y_px, 161.0);
  EXPECT_NEAR(cm.blind_commit_edge_max_rate_y_px_s, 361.0);
  EXPECT_NEAR(cm.blind_commit_min_score, 0.61);
  EXPECT_NEAR(cm.blind_commit_trend_lead_s, 0.221);
  EXPECT_NEAR(cm.blind_commit_trend_max_rate_rad_s, 0.91);
  EXPECT_EQ(cm.predictive_enable, true);
  EXPECT_NEAR(cm.predict_lead_s, 0.301);
  EXPECT_NEAR(cm.predict_kp_rate, 3.01);
  EXPECT_NEAR(cm.predict_kd_rate, 0.251);
  EXPECT_NEAR(cm.predict_max_rate_rad_s, 1.01);
  EXPECT_NEAR(cm.predict_blend_s, 0.351);
  EXPECT_EQ(cm.freeze_on_edge_protect, true);
  EXPECT_NEAR(cm.forward_pitch_rate_rad_s, 0.05);
  EXPECT_NEAR(cm.roll_level_kp, 1.21);
  EXPECT_NEAR(cm.roll_level_max_rad_s, 0.451);
  EXPECT_NEAR(cm.roll_level_blend, 0.11);
  EXPECT_NEAR(cm.pitch_level_kp, 2.01);
  EXPECT_NEAR(cm.pitch_level_max_rad_s, 0.651);
  EXPECT_NEAR(cm.pitch_level_blend, 0.051);
  EXPECT_EQ(cm.yaw_lock_enabled, false);
  EXPECT_NEAR(cm.min_margin_x_px, 31.0);
  EXPECT_NEAR(cm.min_margin_y_px, 71.0);

  // final_approach.fallback
  EXPECT_NEAR(p.final_approach.fallback.decay_tau_s, 1.51);
  EXPECT_NEAR(p.final_approach.fallback.max_s, 2.51);
  EXPECT_NEAR(p.final_approach.fallback.pitch_bias_rad, -0.11);

  // yaw
  EXPECT_NEAR(p.yaw.bearing_kp, 0.61);
  EXPECT_NEAR(p.yaw.hold_gain, 0.51);
  EXPECT_NEAR(p.yaw.rate_min_rad_s, -0.71);
  EXPECT_NEAR(p.yaw.rate_max_rad_s, 0.72);
  EXPECT_NEAR(p.yaw.hold_deadband_rad, 0.0088);
  EXPECT_NEAR(p.yaw.track_deadband_rad, 0.0525);
  EXPECT_NEAR(p.yaw.rate_lpf_tau_s, 0.151);
  EXPECT_EQ(p.yaw.lock_enabled, true);

  // waiting
  EXPECT_EQ(p.waiting.level_hold_enabled, false);
  EXPECT_NEAR(p.waiting.level_kp, 1.51);
  EXPECT_NEAR(p.waiting.level_deadband_rad, 0.0089);
  EXPECT_NEAR(p.waiting.altitude_kp, 0.21);
  EXPECT_NEAR(p.waiting.altitude_kd, 0.11);
  EXPECT_NEAR(p.waiting.altitude_max_correction, 0.19);

  // force_level
  EXPECT_NEAR(p.force_level.hard_level_kp, 1.52);
  EXPECT_EQ(p.force_level.min_hold_ms, 201);

  // tilt_cap
  EXPECT_NEAR(p.tilt_cap.max_roll_angle_rad, 1.21);
  EXPECT_NEAR(p.tilt_cap.max_pitch_angle_rad, 1.22);
  EXPECT_NEAR(p.tilt_cap.softcap_band_rad, 0.21);
  EXPECT_NEAR(p.tilt_cap.hardcap_margin_rad, 0.11);

  // thrust
  EXPECT_EQ(p.thrust.enable_tilt_compensation, true);
  EXPECT_NEAR(p.thrust.hover_scalar, 0.284);
  EXPECT_NEAR(p.thrust.constant_scalar, 0.51);
  EXPECT_NEAR(p.thrust.tracking_ramp_tau_s, 0.36);
  EXPECT_NEAR(p.thrust.slew_rate_scalar_s, 1.01);
  EXPECT_NEAR(p.thrust.tilt_cos_floor, 0.52);
  EXPECT_NEAR(p.thrust.scalar_min, 0.021);
  EXPECT_NEAR(p.thrust.scalar_max, 0.981);

  // target_loss
  EXPECT_EQ(p.target_loss.complete_on_loss_enable, false);
  EXPECT_NEAR(p.target_loss.loss_complete_s, 0.61);
  EXPECT_NEAR(p.target_loss.lost_target_rate_decay_tau_s, 0.51);
  EXPECT_EQ(p.target_loss.complete_altitude_gate_enable, true);
  EXPECT_NEAR(p.target_loss.complete_max_altitude_gap_m, 3.1);

  // evaluation
  EXPECT_EQ(p.evaluation.target_altitude_enable, true);
  EXPECT_NEAR(p.evaluation.target_altitude_m, 61.0);
  EXPECT_NEAR(p.evaluation.success_altitude_gap_m, 3.2);
  EXPECT_EQ(p.evaluation.relative_distance_enable, false);
  EXPECT_NEAR(p.evaluation.relative_distance_max_age_s, 0.26);
  EXPECT_NEAR(p.evaluation.target_origin_offset_x_m, 1.5);
  EXPECT_NEAR(p.evaluation.target_origin_offset_y_m, 2.5);

  // tilt_guard
  EXPECT_EQ(p.tilt_guard.enable, true);
  EXPECT_NEAR(p.tilt_guard.max_tilt_rad, 1.05);
  EXPECT_NEAR(p.tilt_guard.recovery_kp, 0.81);

  // directional_dive
  EXPECT_EQ(p.directional_dive.enable, true);
  EXPECT_NEAR(p.directional_dive.start_edge_ratio, 0.051);
  EXPECT_NEAR(p.directional_dive.end_edge_ratio, 0.101);
  EXPECT_NEAR(p.directional_dive.dive_roll_gain, 1.11);
  EXPECT_NEAR(p.directional_dive.dive_pitch_gain, 0.81);
  EXPECT_NEAR(p.directional_dive.climb_thrust, 0.451);
  EXPECT_NEAR(p.directional_dive.descend_thrust, 0.381);
  EXPECT_NEAR(p.directional_dive.cruise_thrust, 0.401);
  EXPECT_NEAR(p.directional_dive.lead_time_s, 0.101);
  EXPECT_NEAR(p.directional_dive.detection_max_age_s, 0.301);
  EXPECT_NEAR(p.directional_dive.min_horizontal_boost, 3.01);
  EXPECT_NEAR(p.directional_dive.tilt_comp_threshold, 0.016);
  EXPECT_NEAR(p.directional_dive.tilt_comp_gain, 0.51);

  // top-level scalars
  EXPECT_NEAR(p.lateral_output_sign, 1.0);
  EXPECT_NEAR(p.longitudinal_output_sign, 1.0);
  EXPECT_NEAR(p.lateral_kp_rate, 4.01);
  EXPECT_NEAR(p.lateral_kd_rate, 0.881);
  EXPECT_NEAR(p.longitudinal_kp_rate, 5.01);
  EXPECT_NEAR(p.longitudinal_kd_rate, 0.882);
  EXPECT_NEAR(p.x_deadband, 0.021);
  EXPECT_NEAR(p.y_deadband, 0.022);
  EXPECT_NEAR(p.x_deadband_px, 13.0);
  EXPECT_NEAR(p.y_deadband_px, 14.0);
  EXPECT_NEAR(p.aim_offset_x_px, 1.0);
  EXPECT_NEAR(p.aim_offset_y_px, 2.0);
  EXPECT_NEAR(p.pixel_dot_lpf_tau_s, 0.081);
  EXPECT_NEAR(p.rate_lpf_tau_s, 0.051);
  EXPECT_NEAR(p.max_jerk_rad_s2, 81.0);
  EXPECT_NEAR(p.max_roll_rate_rad_s, 2.51);
  EXPECT_NEAR(p.max_pitch_rate_rad_s, 3.51);
  EXPECT_EQ(p.rate_shaper_diag_log, true);
  EXPECT_EQ(p.require_armed_to_command, false);
  EXPECT_EQ(p.dry_run, true);
  EXPECT_EQ(p.armed_latch_ttl_ms, uint32_t(1499));
  EXPECT_EQ(p.armed_disarm_debounce_count, uint8_t(49));
}

const char* kBfFlightSentinelYaml = R"YAML(
bf_flight:
  msp:
    device: /dev/ttyS9
    baud: 57600
    passthrough_in_dry_run: false
    passthrough_log: false
    passthrough_log_interval_s: 2.5
    passthrough_throttle_jump_pwm: 77
    passthrough_hz: 33.0
    live_publish_hz: 88.0
    attitude_poll_divisor: 3
    status_poll_divisor: 9
    override_grace_hold_s: 0.41
    override_mode_flag_auto: false
    override_mode_flag: 0x4000000
    override_channels_mask: 7
    passthrough_channel_count: 12
  rc:
    rc_min: 1100
    rc_mid: 1511
    rc_max: 1900
    roll_channel: 1
    pitch_channel: 2
    throttle_channel: 3
    yaw_channel: 4
    aux_arm_channel: 5
    max_roll_rate_rad_s: 2.11
    max_pitch_rate_rad_s: 2.22
    max_yaw_rate_rad_s: 0.77
)YAML";

void testEveryBfFlightSectionKeyTakesEffect() {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  const std::string path = writeTempYaml(kBfFlightSentinelYaml);
  const auto bf = circle::bf::loadBfFlightYamlSectionsFromFile(path);
  std::remove(path.c_str());

  EXPECT_EQ(bf.msp.device, std::string("/dev/ttyS9"));
  EXPECT_EQ(bf.msp.baud, 57600);
  EXPECT_EQ(bf.msp.passthrough_in_dry_run, false);
  EXPECT_EQ(bf.msp.passthrough_log, false);
  EXPECT_NEAR(bf.msp.passthrough_log_interval_s, 2.5);
  EXPECT_EQ(bf.msp.passthrough_throttle_jump_pwm, uint16_t(77));
  EXPECT_NEAR(bf.msp.passthrough_hz, 33.0);
  EXPECT_NEAR(bf.msp.live_publish_hz, 88.0);
  EXPECT_EQ(bf.msp.attitude_poll_divisor, uint32_t(3));
  EXPECT_EQ(bf.msp.status_poll_divisor, uint32_t(9));
  EXPECT_NEAR(bf.msp.override_grace_hold_s, 0.41);
  EXPECT_EQ(bf.msp.override_mode_flag_auto, false);
  EXPECT_EQ(bf.msp.override_mode_flag, uint32_t(0x4000000));
  EXPECT_EQ(bf.msp.override_channels_mask, uint32_t(7));
  EXPECT_EQ(bf.msp.passthrough_channel_count, uint16_t(12));

  EXPECT_EQ(bf.rc.rc_min, uint16_t(1100));
  EXPECT_EQ(bf.rc.rc_mid, uint16_t(1511));
  EXPECT_EQ(bf.rc.rc_max, uint16_t(1900));
  EXPECT_EQ(bf.rc.roll_channel, uint16_t(1));
  EXPECT_EQ(bf.rc.pitch_channel, uint16_t(2));
  EXPECT_EQ(bf.rc.yaw_channel, uint16_t(4));
  EXPECT_EQ(bf.rc.throttle_channel, uint16_t(3));
  EXPECT_EQ(bf.rc.aux_arm_channel, uint16_t(5));
  EXPECT_NEAR(bf.rc.max_roll_rate_rad_s, 2.11);
  EXPECT_NEAR(bf.rc.max_pitch_rate_rad_s, 2.22);
  EXPECT_NEAR(bf.rc.max_yaw_rate_rad_s, 0.77);
#else
  std::cerr << "SKIP: CIRCLE_STRIKE_HAS_YAML not defined\n";
#endif
}

}  // namespace

int main() {
  testRealBfConfig();
  testRealBfFlightSections();
  testEveryStrikeKeyTakesEffect();
  testEveryBfFlightSectionKeyTakesEffect();
  if (g_failures == 0) {
    std::cout << "bf_params_parse_test: ALL PARAMS PARSED OK\n";
    return 0;
  }
  std::cerr << "bf_params_parse_test: " << g_failures << " failure(s)\n";
  return 1;
}
