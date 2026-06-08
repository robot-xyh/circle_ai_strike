#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

#include "circle/strike/strike_controller.hpp"
#include "circle/strike/math_utils.hpp"
#include "circle/strike/modules/rate_shaper.hpp"
#include "circle/strike/modules/edge_protection.hpp"
#include "circle/strike/modules/final_approach_gate.hpp"
#include "circle/strike/modules/speed_governor.hpp"
#include "circle/strike/modules/preclimb_module.hpp"
#include "circle/strike/modules/thrust_manager.hpp"
#include "circle/strike/modules/commit_module.hpp"
#include "circle/strike/modules/terminal_predictor.hpp"
#include "circle/strike/modules/visual_png_guidance.hpp"
#include "circle/strike_png/entry_handoff.hpp"
#include "circle/strike_png/strike_png_controller.hpp"
#include "circle/strike_png/target_loss_hold.hpp"
#include "circle/types/time.hpp"
#include "circle/vision/sot_byte_track.hpp"
#include "circle/vision/yolo_postprocess.hpp"

namespace {

void testYoloPostprocess() {
  const float output[] = {320.0F, 240.0F, 80.0F, 60.0F, 0.9F};
  const uint32_t shape[] = {1, 5};
  const auto dets = circle::vision::yoloPostprocess(
      output, shape, 2, 640, 480, 1.0F, 0, 0, 0.25F, 0.45F, 10);
  assert(dets.size() == 1);
}

circle::types::Detection makeTrackDet(float cx,
                                       float cy,
                                       float w,
                                       float h,
                                       float score = 0.9F) {
  circle::types::Detection d;
  d.cx = cx;
  d.cy = cy;
  d.width = w;
  d.height = h;
  d.score = score;
  d.class_name = "UAV";
  d.class_id = 0;
  return d;
}

void testSotByteTrackSingleTarget() {
  circle::vision::SotByteTrackParams params;
  params.enabled = true;
  params.high_score_threshold = 0.25F;
  params.low_score_threshold = 0.10F;
  params.max_lost_frames = 2;
  params.emit_prediction_max_frames = 2;
  params.max_dt_s = 0.20F;
  circle::vision::SotByteTrack tracker(params);
  circle::vision::DetectionFilterParams filter;
  filter.target_class_name = "UAV";
  filter.min_bbox_area = 20.0;
  filter.max_bbox_aspect_ratio = 5.0;

  const auto first = tracker.update({makeTrackDet(100.0F, 120.0F, 30.0F, 20.0F)},
                                    filter, 1'000'000'000ULL, 640, 512);
  assert(first.has_value());
  assert(first->track_id == 1);
  assert(!first->tracker_predicted);

  const auto second =
      tracker.update({makeTrackDet(104.0F, 121.0F, 30.0F, 20.0F)}, filter,
                     1'033'000'000ULL, 640, 512);
  assert(second.has_value());
  assert(second->track_id == 1);
  assert(!second->tracker_predicted);

  const auto low =
      tracker.update({makeTrackDet(108.0F, 122.0F, 30.0F, 20.0F, 0.12F)},
                     filter, 1'066'000'000ULL, 640, 512);
  assert(low.has_value());
  assert(low->track_id == 1);
  assert(!low->tracker_predicted);
}

void testSotByteTrackPredictionAndExpiry() {
  circle::vision::SotByteTrackParams params;
  params.enabled = true;
  params.max_lost_frames = 2;
  params.emit_prediction_max_frames = 2;
  params.max_dt_s = 0.20F;
  circle::vision::SotByteTrack tracker(params);
  circle::vision::DetectionFilterParams filter;
  filter.target_class_name = "UAV";
  filter.min_bbox_area = 20.0;
  filter.max_bbox_aspect_ratio = 5.0;

  const auto first = tracker.update({makeTrackDet(200.0F, 220.0F, 40.0F, 30.0F)},
                                    filter, 2'000'000'000ULL, 640, 512);
  assert(first.has_value());
  const auto second =
      tracker.update({makeTrackDet(208.0F, 220.0F, 60.0F, 45.0F)}, filter,
                     2'033'000'000ULL, 640, 512);
  assert(second.has_value());

  const auto miss1 = tracker.update({}, filter, 2'066'000'000ULL, 640, 512);
  assert(miss1.has_value());
  assert(miss1->tracker_predicted);
  assert(miss1->tracker_lost_frames == 1);
  assert(std::abs(miss1->width - second->width) < 1.0e-3F);
  assert(std::abs(miss1->height - second->height) < 1.0e-3F);

  const auto miss2 = tracker.update({}, filter, 2'099'000'000ULL, 640, 512);
  assert(miss2.has_value());
  assert(miss2->tracker_predicted);
  assert(miss2->tracker_lost_frames == 2);

  const auto expired = tracker.update({}, filter, 2'132'000'000ULL, 640, 512);
  assert(!expired.has_value());
}

void testSotByteTrackRejectsOutOfOrderFrames() {
  circle::vision::SotByteTrackParams params;
  params.enabled = true;
  circle::vision::SotByteTrack tracker(params);
  circle::vision::DetectionFilterParams filter;
  filter.target_class_name = "UAV";

  const auto first = tracker.update({makeTrackDet(300.0F, 200.0F, 40.0F, 30.0F)},
                                    filter, 3'000'000'000ULL, 640, 512);
  assert(first.has_value());
  const auto stale = tracker.update({makeTrackDet(10.0F, 10.0F, 40.0F, 30.0F)},
                                    filter, 2'999'000'000ULL, 640, 512);
  assert(!stale.has_value());
  const auto next = tracker.update({makeTrackDet(304.0F, 200.0F, 40.0F, 30.0F)},
                                   filter, 3'033'000'000ULL, 640, 512);
  assert(next.has_value());
  assert(next->track_id == 1);
  assert(next->cx > 250.0F);
}

void testMathUtils() {
  assert(std::abs(circle::strike::smoothstep01(0.0F) - 0.0F) < 1e-6F);
  assert(std::abs(circle::strike::smoothstep01(1.0F) - 1.0F) < 1e-6F);
  assert(std::abs(circle::strike::smoothstep01(0.5F) - 0.5F) < 1e-6F);
  
  assert(std::abs(circle::strike::lerp(0.5F, 0.0F, 1.0F, 10.0F, 20.0F) - 15.0F) < 1e-6F);
  
  assert(circle::strike::tiltSoftcapFactor(0.0F, 1.0F, 0.5F, 0.1F) == 1.0F);
  assert(circle::strike::tiltSoftcapFactor(0.6F, 1.0F, 0.5F, 0.1F) < 1.0F);
}

void testRateShaper() {
  circle::strike::RateShaper shaper;
  shaper.reset();
  auto out = shaper.compute(1.0F, 0.5F, 0.3F, 0.1F, 0.05F, 0.5F, 0.5F, 0.1F,
                           0.05F, 20.0F, 2.0F, 2.0F, 0.15F, false,
                           1.0F, 1.0F, 1.0F, 0.02F, 0.02F, false);
  assert(std::isfinite(out.roll_rate_rad_s));
  assert(std::isfinite(out.pitch_rate_rad_s));
  assert(std::isfinite(out.yaw_rate_rad_s));
}

void testEdgeProtection() {
  circle::strike::EdgeProtection ep;
  circle::types::FrameDetection det;
  det.image_width = 640;
  det.image_height = 480;
  det.detection.cx = 320.0F;
  det.detection.cy = 240.0F;
  det.detection.width = 80.0F;
  det.detection.height = 60.0F;
  det.intrinsics.fx = 640.0F;
  det.intrinsics.fy = 640.0F;
  
  circle::types::FcState vehicle;
  vehicle.roll_rad = 0.1F;
  vehicle.pitch_rad = 0.05F;
  
  circle::strike::FAEdgeProtectParams edge_params;
  edge_params.enable = true;
  edge_params.margin_x_px = 50.0F;
  edge_params.margin_y_px = 50.0F;
  
  circle::strike::FABottomPitchGuardParams bottom_params;
  bottom_params.enable = false;
  
  auto out = ep.compute(true, true, det, vehicle, 0.0F, 0.0F, 0.0F, 0.0F,
                        1.0F, -1.0F, edge_params, bottom_params, 60.0F, 50.0F);
  assert(std::isfinite(out.roll_boost));
  assert(std::isfinite(out.pitch_boost));
}

void testFinalApproachGate() {
  circle::strike::FinalApproachGate gate;
  circle::strike::FAGateState state;
  circle::strike::FAGateParams params;
  params.boost_enable = true;
  params.area_ratio_enter = 0.1F;
  params.area_ratio_exit = 0.08F;
  
  circle::strike::SpeedGovernorParams speed_params;
  circle::strike::PreclimbParams preclimb_params;
  circle::strike::EvaluationParams evaluation_params;
  circle::strike::RhoScaleParams rho_params;
  
  circle::types::FrameDetection det;
  det.image_width = 640;
  det.image_height = 480;
  det.detection.width = 200.0F;
  det.detection.height = 150.0F;
  
  circle::types::FcState vehicle;
  vehicle.velocity_xy_valid = false;
  
  auto out = gate.compute(state, params, speed_params, preclimb_params, evaluation_params, rho_params,
                         false, true, true, det, vehicle, 0.0F, 0.0F, 0.0F, 0.0F, 0);
  assert(out.bbox_area_ratio > 0.0F);
}

void testSpeedGovernor() {
  circle::strike::SpeedGovernor gov;
  circle::strike::SpeedGovernorParams params;
  params.enable = true;
  params.start_m_s = 5.0F;
  params.full_m_s = 10.0F;
  params.min_image_scale = 0.5F;
  
  auto out = gov.compute(params, false, true, 7.5F, 0.1F, 0.05F, 1.0F, 0.5F);
  assert(out.scale >= 0.5F && out.scale <= 1.0F);
}

void testPreclimbModule() {
  circle::strike::PreclimbModule preclimb;
  circle::strike::PreclimbState state;
  circle::strike::PreclimbParams params;
  params.xy_gate_enable = true;
  params.xy_hold_s = 0.5F;
  
  auto out = preclimb.compute(state, params, false, true, true,
                             0.0F, 0.0F, 0.0F, 0.0F, 640.0F, 640.0F,
                             100.0F, 100.0F, 0.1F, 0.05F, 1.0F, 0.5F, 0);
  assert(std::isfinite(out.roll_rate_rad_s));
  assert(std::isfinite(out.pitch_rate_rad_s));
}

void testThrustManager() {
  circle::strike::ThrustManager mgr;
  circle::strike::ThrustManagerState state;
  circle::strike::StrikeParams params;
  
  auto out = mgr.compute(state, params, false, false, 1.0F, 0.0F,
                        false, false, {}, true, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F,
                        640.0F, 640.0F, 0.1F, 0.05F, 30.0F, 30.0F, 0.0F, 0.0F,
                        false, 0, 0.02F);
  assert(std::isfinite(out.thrust_z));
  assert(out.thrust_z >= 0.0F);  // Core outputs positive scalar (0.0-1.0)
  assert(out.thrust_z <= 1.0F);
}

void testCommitModule() {
  circle::strike::CommitModule commit;
  circle::strike::CommitState state;
  circle::strike::FACommitParams params;
  params.enable = true;
  params.command_hold_s = 1.0F;
  
  circle::strike::ThrustParams thrust_params;
  
  auto out = commit.computeHold(state, params, thrust_params, 0.5F,
                               1.0F, -1.0F, 0.1F, 0.05F, 0.5F, 0.5F, 0.1F,
                               false, 0.5F, 0);
  assert(std::isfinite(out.roll_rate_rad_s));
  assert(std::isfinite(out.pitch_rate_rad_s));
}

void testCommitFutureGateBlocksProjectedMiss() {
  circle::strike::CommitModule commit;
  circle::strike::CommitState state;
  circle::strike::FACommitParams params;
  params.enable = true;
  params.align_gate_enable = true;
  params.align_hold_s = 0.0F;
  params.align_max_error_x_px = 40.0F;
  params.align_max_error_y_px = 40.0F;
  params.future_gate_enable = true;
  params.future_lead_s = 0.20F;
  params.future_max_error_x_px = 45.0F;
  params.future_max_error_y_px = 45.0F;

  const auto now = circle::types::monotonicNowNs();
  const bool blocked = commit.shouldCreateSnapshot(
      state, params,
      true,   // final_approach_active
      true,   // measure_reliable
      true,   // strike_confident
      0.010F, // bbox_area_ratio
      100.0F, 100.0F,
      20.0F, 20.0F,
      10.0F, 10.0F,
      70.0F, 20.0F,
      0.0F, 0.0F,
      false, now);
  assert(!blocked);

  const bool allowed = commit.shouldCreateSnapshot(
      state, params,
      true, true, true,
      0.010F,
      100.0F, 100.0F,
      20.0F, 20.0F,
      10.0F, 10.0F,
      30.0F, 20.0F,
      0.0F, 0.0F,
      false, now + 1000000ULL);
  assert(allowed);
}

void testTerminalPredictor() {
  circle::strike::TerminalPredictor predictor;
  circle::strike::TerminalPredictorParams params;
  circle::strike::TerminalPredictorInput input;
  input.final_approach_active = true;
  input.fresh_detection = true;
  input.bbox_area_ratio = 0.010F;
  input.ex = 0.05F;
  input.ey = -0.03F;
  input.ex_dot = 0.80F;
  input.ey_dot = -0.40F;
  input.e_rho_dot = -1.20F;
  input.fx = 500.0F;
  input.fy = 500.0F;

  auto out = predictor.compute(params, input);
  assert(!out.active);

  params.enable = true;
  params.min_area_ratio = 0.002F;
  params.min_closing_rate = 0.20F;
  params.lead_s = 0.20F;
  params.max_lead_px = 40.0F;
  params.blend = 1.0F;

  out = predictor.compute(params, input);
  assert(out.active);
  assert(std::abs(out.lead_x_px - 40.0F) < 1.0e-4F);
  assert(std::abs(out.lead_y_px + 40.0F) < 1.0e-4F);
  assert(std::abs(out.predicted_ex - 0.13F) < 1.0e-4F);
  assert(std::abs(out.predicted_ey + 0.11F) < 1.0e-4F);

  input.e_rho_dot = 0.10F;
  out = predictor.compute(params, input);
  assert(!out.active);
}

void testVisualPngGuidanceDerotatesBodyRates() {
  const float ex_dot = circle::strike::VisualPngGuidance::derotateExDot(
      -0.40F, 0.40F, 1.0F);
  const float ey_dot = circle::strike::VisualPngGuidance::derotateEyDot(
      0.30F, 0.30F, 1.0F);
  assert(std::abs(ex_dot) < 1.0e-6F);
  assert(std::abs(ey_dot) < 1.0e-6F);
}

void testVisualPngGuidanceCommandsLosRateSuppression() {
  circle::strike::VisualPngGuidance guidance;
  circle::strike::VisualPngGuidanceParams params;
  params.enable = true;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 3.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.fov_trim_kd_rate = 0.0F;
  params.derotate_body_rates = false;
  params.closure_base_scale = 1.0F;
  params.closure_rho_dot_gain = 0.0F;
  params.closure_area_gain = 0.0F;
  params.closure_min_scale = 0.1F;
  params.closure_max_scale = 5.0F;

  circle::strike::VisualPngGuidanceInput input;
  input.active = true;
  input.ex_dot = 0.0F;
  input.ey_dot = 0.0F;
  input.lateral_output_sign = 1.0F;
  input.longitudinal_output_sign = -1.0F;
  input.max_roll_rate_rad_s = 2.0F;
  input.max_pitch_rate_rad_s = 2.0F;

  auto out = guidance.compute(params, input);
  assert(out.active);
  assert(std::abs(out.roll_rate_rad_s) < 1.0e-6F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);

  input.ex_dot = 0.10F;
  out = guidance.compute(params, input);
  assert(out.active);
  assert(out.roll_png_ff_rad_s > 0.25F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
}


void testRhoRateWindowEstimatesClosingFromAreaGrowth() {
  circle::strike::RhoRateWindowEstimator estimator;
  circle::strike::RhoRateWindowParams params;
  params.enable = true;
  params.window_samples = 5;
  params.lpf_tau_s = 0.0F;

  const uint64_t t0 = circle::types::monotonicNowNs();
  float out = 0.0F;
  for (int i = 0; i < 5; ++i) {
    const float e_rho = -0.20F * static_cast<float>(i);
    out = estimator.addSample(params, e_rho, t0 + static_cast<uint64_t>(i) * 100000000ULL);
  }

  assert(out < -1.5F);
  assert(out > -2.5F);
}

void testControllerStateTransitions() {
  circle::strike::StrikeController controller;
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 330.0F;
  in.detection.detection.cy = 250.0F;
  in.detection.detection.width = 80.0F;
  in.detection.detection.height = 60.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";
  
  auto out = controller.tick(in);
  assert(out.has_valid_target);
  assert(out.state == circle::strike::StrikeState::Tracking);
  assert(std::isfinite(out.rates.roll_rate_rad_s));
  assert(std::isfinite(out.rates.pitch_rate_rad_s));
  assert(std::isfinite(out.rates.yaw_rate_rad_s));
  assert(std::isfinite(out.rates.thrust_z));
  
  in.vehicle.roll_rad = 0.8F;
  in.now_ns += 100000000ULL;
  out = controller.tick(in);
  assert(out.state == circle::strike::StrikeState::ForceLevel);
}

void testControllerPaperPngGuidanceDiffersFromLegacyPd() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.rate_lpf_tau_s = 0.0F;
  params.max_jerk_rad_s2 = 1000.0F;
  params.lateral_kp_rate = 0.20F;
  params.lateral_kd_rate = 0.0F;
  params.longitudinal_kp_rate = 0.20F;
  params.longitudinal_kd_rate = 0.0F;
  params.guidance_mode = circle::strike::GuidanceMode::PaperPng;
  params.visual_png.enable = true;
  params.visual_png.nav_ratio_x = 3.0F;
  params.visual_png.nav_ratio_y = 3.0F;
  params.visual_png.fov_trim_kp_rate = 0.0F;
  params.visual_png.fov_trim_kd_rate = 0.0F;
  params.visual_png.derotate_body_rates = false;
  params.visual_png.closure_base_scale = 1.0F;
  params.visual_png.closure_rho_dot_gain = 0.0F;
  params.visual_png.closure_area_gain = 0.0F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 80.0F;
  in.detection.detection.height = 60.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  auto out = controller.tick(in);
  assert(out.has_valid_target);

  in.now_ns += 20000000ULL;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.detection.cx = 384.0F;
  out = controller.tick(in);

  assert(out.telemetry.png_active);
  assert(out.telemetry.guidance_mode ==
         static_cast<int>(circle::strike::GuidanceMode::PaperPng));
  assert(out.telemetry.png_roll_ff_rad_s > 1.0F);
  assert(out.rates.roll_rate_rad_s > 1.0F);
}

void testApproachDriveFovGateSuppressesHighTargetPitchBias() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.guidance_mode = circle::strike::GuidanceMode::LegacyPd;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.rate_lpf_tau_s = 0.0F;
  params.max_jerk_rad_s2 = 1000.0F;
  params.longitudinal_kp_rate = 0.0F;
  params.longitudinal_kd_rate = 0.0F;
  params.max_pitch_rate_rad_s = 2.0F;
  params.approach_drive.enable = true;
  params.approach_drive.e_rho_deadband = 0.05F;
  params.approach_drive.pitch_rate_gain = 0.20F;
  params.approach_drive.pitch_rate_max_rad_s = 0.20F;
  params.approach_drive.pitch_output_sign = -1.0F;
  params.approach_drive.fov_gate_enable = true;
  params.approach_drive.fov_gate_high_error_px = 150.0F;
  params.approach_drive.fov_gate_release_error_px = 90.0F;
  params.approach_drive.fov_gate_min_scale = 0.0F;

  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 512;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.width = 10.0F;
  in.detection.detection.height = 10.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  in.detection.detection.cy = 20.0F;
  circle::strike::StrikeController high_controller(params);
  auto high = high_controller.tick(in);
  assert(high.has_valid_target);
  assert(std::abs(high.rates.pitch_rate_rad_s) < 0.02F);

  in.detection.detection.cy = 180.0F;
  circle::strike::StrikeController inside_controller(params);
  auto inside = inside_controller.tick(in);
  assert(inside.has_valid_target);
  assert(inside.rates.pitch_rate_rad_s < -0.10F);
}

void testControllerTerminalPredictorTelemetry() {
  circle::strike::StrikeParams params;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.final_approach.gate.boost_enable = true;
  params.final_approach.gate.area_ratio_enter = 0.001F;
  params.final_approach.gate.area_ratio_exit = 0.0008F;
  params.final_approach.gate.area_quality_gate_enable = false;
  params.final_approach.terminal_predictor.enable = true;
  params.final_approach.terminal_predictor.min_area_ratio = 0.001F;
  params.final_approach.terminal_predictor.lead_s = 0.12F;
  params.final_approach.terminal_predictor.max_lead_px = 80.0F;
  params.final_approach.terminal_predictor.blend = 1.0F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 80.0F;
  in.detection.detection.height = 60.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  auto out = controller.tick(in);
  assert(out.state == circle::strike::StrikeState::Tracking);
  assert(out.telemetry.final_approach_active);

  in.now_ns += 20000000ULL;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.detection.cx = 328.0F;
  out = controller.tick(in);
  assert(out.telemetry.terminal_predictor_active);
  assert(out.telemetry.terminal_predictor_lead_x_px > 1.0F);
  assert(std::abs(out.telemetry.terminal_predictor_predicted_ex -
                  out.telemetry.ex) > 1.0e-4F);
}

void testControllerUsesDkfEstimateWhenEnabled() {
  circle::strike::StrikeParams params;
  params.dkf_enable = true;
  params.dkf.process_accel_noise = 100.0F;
  params.dkf.meas_noise_px = 120.0F;
  params.dkf.predict_extra_delay_s = 0.05F;
  params.dkf.max_cov_trace = 10.0F;
  params.pixel_dot_lpf_tau_s = 0.0F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 10.0F;
  in.detection.detection.height = 10.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  auto out = controller.tick(in);
  assert(out.has_valid_target);

  in.now_ns += 20000000ULL;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.detection.cx = 420.0F;
  out = controller.tick(in);

  const float raw_ex = (420.0F - 320.0F) / 640.0F;
  assert(out.has_valid_target);
  assert(std::abs(out.telemetry.ex - raw_ex) > 1.0e-3F);
  assert(out.telemetry.ex > 0.0F);
  assert(out.telemetry.ex < raw_ex);
  assert(out.telemetry.ex_dot_filt > 0.0F);
}

void testControllerAcceptsFreshReceiveStampWithOldCaptureStamp() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.detection_stale_s = 0.35;
  params.filter.min_bbox_area = 100.0;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = 12000000000ULL;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 512;
  in.detection.detection.cx = 188.0F;
  in.detection.detection.cy = 190.0F;
  in.detection.detection.width = 12.0F;
  in.detection.detection.height = 12.0F;
  in.detection.detection.score = 0.65F;
  in.detection.detection.class_name = "UAV";

  const auto out = controller.tick(in);
  assert(out.has_valid_target);
  assert(out.state == circle::strike::StrikeState::Tracking);
}

void testFinalApproachGateUsesCurrentImageRate() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.final_approach.gate.boost_enable = true;
  params.final_approach.gate.area_ratio_enter = 0.001F;
  params.final_approach.gate.area_ratio_exit = 0.0008F;
  params.final_approach.gate.area_quality_gate_enable = true;
  params.final_approach.gate.area_quality_error_x_px = 200.0F;
  params.final_approach.gate.area_quality_error_y_px = 200.0F;
  params.final_approach.gate.area_quality_rate_x_px_s = 100.0F;
  params.final_approach.gate.area_quality_rate_y_px_s = 100.0F;
  params.final_approach.gate.area_quality_max_tilt_rad = 0.50F;
  params.final_approach.gate.hold_s = 0.0F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 10.0F;
  in.detection.detection.height = 10.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  auto out = controller.tick(in);
  assert(!out.telemetry.final_approach_active);

  in.now_ns += 20000000ULL;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.detection.cx = 360.0F;
  in.detection.detection.width = 80.0F;
  in.detection.detection.height = 60.0F;
  out = controller.tick(in);

  assert(out.telemetry.ex_dot_filt * in.detection.intrinsics.fx > 1000.0F);
  assert(!out.telemetry.final_approach_active);
}

void testCloseRangeFinalApproachKeepsRollLeveling() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.lateral_kp_rate = 0.0F;
  params.lateral_kd_rate = 0.0F;
  params.longitudinal_kp_rate = 0.0F;
  params.longitudinal_kd_rate = 0.0F;
  params.rate_lpf_tau_s = 0.0F;
  params.max_jerk_rad_s2 = 1000.0F;
  params.max_roll_rate_rad_s = 2.0F;
  params.final_approach.gate.boost_enable = true;
  params.final_approach.gate.area_ratio_enter = 0.001F;
  params.final_approach.gate.area_ratio_exit = 0.0008F;
  params.final_approach.gate.area_quality_gate_enable = false;
  params.final_approach.leveling.roll_level_start_ratio = 0.001F;
  params.final_approach.leveling.roll_level_end_ratio = 0.002F;
  params.final_approach.leveling.roll_level_kp = 1.2F;
  params.final_approach.leveling.roll_level_max_rad_s = 0.40F;
  params.final_approach.leveling.pitch_level_start_ratio = 0.001F;
  params.final_approach.leveling.pitch_level_end_ratio = 0.002F;
  params.final_approach.leveling.pitch_level_kp = 1.2F;
  params.final_approach.leveling.pitch_level_max_rad_s = 0.40F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.50F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 110.0F;
  in.detection.detection.height = 110.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  const auto out = controller.tick(in);
  assert(out.telemetry.final_approach_active);
  assert(out.telemetry.fa_roll_level_blend > 0.95F);
  assert(out.rates.roll_rate_rad_s < -0.35F);
}

void testTerminalEdgeProtectSurvivesFullRollLeveling() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.lateral_kp_rate = 0.0F;
  params.lateral_kd_rate = 0.0F;
  params.longitudinal_kp_rate = 0.0F;
  params.longitudinal_kd_rate = 0.0F;
  params.rate_lpf_tau_s = 0.0F;
  params.max_jerk_rad_s2 = 1000.0F;
  params.max_roll_rate_rad_s = 2.0F;
  params.final_approach.gate.boost_enable = true;
  params.final_approach.gate.area_ratio_enter = 0.001F;
  params.final_approach.gate.area_ratio_exit = 0.0008F;
  params.final_approach.gate.area_quality_gate_enable = false;
  params.final_approach.leveling.roll_level_start_ratio = 0.001F;
  params.final_approach.leveling.roll_level_end_ratio = 0.002F;
  params.final_approach.leveling.roll_level_kp = 1.2F;
  params.final_approach.leveling.roll_level_max_rad_s = 0.40F;
  params.final_approach.edge_protect.enable = true;
  params.final_approach.edge_protect.margin_x_px = 220.0F;
  params.final_approach.edge_protect.margin_y_px = 0.0F;
  params.final_approach.edge_protect.predict_s = 0.0F;
  params.final_approach.edge_protect.roll_kp_rate = 5.0F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.30F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 520.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 120.0F;
  in.detection.detection.height = 120.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  const auto out = controller.tick(in);
  assert(out.telemetry.final_approach_active);
  assert(out.telemetry.edge_protect_active);
  assert(out.telemetry.fa_roll_level_blend > 0.95F);
  assert(out.rates.roll_rate_rad_s > 0.10F);
}

void testRepeatedDetectionDoesNotKeepTerminalEdgeBoosting() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.detection_stale_s = 0.35;
  params.lateral_kp_rate = 0.0F;
  params.lateral_kd_rate = 0.0F;
  params.longitudinal_kp_rate = 0.0F;
  params.longitudinal_kd_rate = 0.0F;
  params.rate_lpf_tau_s = 0.0F;
  params.max_jerk_rad_s2 = 1000.0F;
  params.max_roll_rate_rad_s = 2.0F;
  params.final_approach.gate.boost_enable = true;
  params.final_approach.gate.area_ratio_enter = 0.001F;
  params.final_approach.gate.area_ratio_exit = 0.0008F;
  params.final_approach.gate.area_quality_gate_enable = false;
  params.final_approach.leveling.roll_level_start_ratio = 0.001F;
  params.final_approach.leveling.roll_level_end_ratio = 0.002F;
  params.final_approach.leveling.roll_level_kp = 0.0F;
  params.final_approach.edge_protect.enable = true;
  params.final_approach.edge_protect.margin_x_px = 220.0F;
  params.final_approach.edge_protect.margin_y_px = 0.0F;
  params.final_approach.edge_protect.predict_s = 0.0F;
  params.final_approach.edge_protect.roll_kp_rate = 5.0F;
  params.final_approach.commit.enable = false;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.roll_rad = 0.0F;
  in.vehicle.pitch_rad = 0.0F;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 520.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 120.0F;
  in.detection.detection.height = 120.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  auto out = controller.tick(in);
  assert(out.telemetry.final_approach_active);
  assert(out.telemetry.edge_protect_active);
  assert(out.rates.roll_rate_rad_s > 0.10F);

  in.now_ns += 120000000ULL;
  out = controller.tick(in);
  assert(out.has_valid_target);
  assert(!out.telemetry.edge_protect_active);
  assert(std::abs(out.rates.roll_rate_rad_s) < 1.0e-5F);
}

void testTargetLossCompletesFromFinalApproachFallback() {
  circle::strike::StrikeParams params;
  params.dkf_enable = false;
  params.detection_stale_s = 0.05;
  params.lost_timeout_s = 0.30;
  params.target_loss.complete_on_loss_enable = true;
  params.target_loss.loss_complete_s = 0.06F;
  params.final_approach.gate.boost_enable = true;
  params.final_approach.gate.area_ratio_enter = 0.001F;
  params.final_approach.gate.area_ratio_exit = 0.0008F;
  params.final_approach.gate.area_quality_gate_enable = false;
  params.final_approach.fallback.max_s = 1.0F;

  circle::strike::StrikeController controller(params);
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.vehicle.position_z_valid = true;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 320.0F;
  in.detection.detection.cy = 240.0F;
  in.detection.detection.width = 80.0F;
  in.detection.detection.height = 80.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";

  auto out = controller.tick(in);
  assert(out.telemetry.final_approach_active);

  in.now_ns += 80000000ULL;
  in.detection.valid = false;
  out = controller.tick(in);

  assert(out.state == circle::strike::StrikeState::Complete);
  assert(out.telemetry.state ==
         static_cast<int>(circle::strike::StrikeState::Complete));
}

void testParameterClamping() {
  circle::strike::StrikeParams params;
  params.max_roll_rate_rad_s = 100.0F;
  params.max_pitch_rate_rad_s = 100.0F;
  params.tilt_cap.max_roll_angle_rad = 10.0F;
  params.tilt_cap.max_pitch_angle_rad = 10.0F;
  params.thrust.hover_scalar = 2.0F;
  params.thrust.constant_scalar = 2.0F;
  params.detection_stale_s = 0.35;
  params.final_approach.commit.detection_stale_s = 0.20F;
  
  params.clamp();
  
  assert(params.max_roll_rate_rad_s <= 6.9813F);
  assert(params.max_pitch_rate_rad_s <= 6.9813F);
  assert(params.tilt_cap.max_roll_angle_rad <= 1.5F);
  assert(params.tilt_cap.max_pitch_angle_rad <= 1.5F);
  assert(params.thrust.hover_scalar <= 0.99F);
  assert(params.thrust.constant_scalar <= 0.99F);
  assert(std::abs(params.final_approach.commit.detection_stale_s - 0.20F) <
         1.0e-6F);
}

void testTelemetryOutput() {
  circle::strike::StrikeController controller;
  circle::strike::StrikeInputs in;
  in.now_ns = circle::types::monotonicNowNs();
  in.mode_active = true;
  in.vehicle.valid = true;
  in.vehicle.armed = true;
  in.detection.valid = true;
  in.detection.capture_ns = in.now_ns;
  in.detection.receive_ns = in.now_ns;
  in.detection.intrinsics.fx = 640.0F;
  in.detection.intrinsics.fy = 640.0F;
  in.detection.intrinsics.cx = 320.0F;
  in.detection.intrinsics.cy = 240.0F;
  in.detection.image_width = 640;
  in.detection.image_height = 480;
  in.detection.detection.cx = 330.0F;
  in.detection.detection.cy = 250.0F;
  in.detection.detection.width = 80.0F;
  in.detection.detection.height = 60.0F;
  in.detection.detection.score = 0.9F;
  in.detection.detection.class_name = "UAV";
  
  auto out = controller.tick(in);
  assert(out.telemetry.valid);
  assert(std::isfinite(out.telemetry.e_rho));
  assert(std::isfinite(out.telemetry.rho_scale));
  assert(std::isfinite(out.telemetry.detection_score));
  assert(out.telemetry.detection_score > 0.0F);
}

void testStrikePngControllerStartsInactiveWithoutTarget() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  circle::strike_png::StrikePngInput input;
  input.now_ns = 1000000000ULL;
  input.detection_valid = false;

  const auto out = controller.tick(params, input);
  assert(!out.has_target);
  assert(!out.png_active);
  assert(std::abs(out.roll_rate_rad_s) < 1.0e-6F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
}

void testStrikePngControllerCommandsLosRateSuppression() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.derotate_body_rates = false;
  params.max_roll_rate_rad_s = 1.2F;
  params.fov_trim_kp_rate = 0.0F;

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  first.bbox_area_ratio = 0.002F;
  first.fx = 500.0F;
  first.fy = 500.0F;
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput second = first;
  second.now_ns += 50000000ULL;
  second.ex = 0.02F;
  const auto out = controller.tick(params, second);

  assert(out.has_target);
  assert(out.png_active);
  assert(std::abs(out.roll_rate_rad_s) > 0.05F);
}

void testStrikePngGuidanceRequiresValidDerotationRate() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = true;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 3.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.max_feedforward_rad_s = 10.0F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.derotate_rate_valid = false;
  input.roll_rate_rad_s = 1.0F;
  input.max_roll_rate_rad_s = 10.0F;
  input.max_pitch_rate_rad_s = 10.0F;
  auto out = guidance.compute(params, input);
  assert(out.active);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
  assert(std::abs(out.ey_dot_inertial) < 1.0e-6F);

  input.derotate_rate_valid = true;
  out = guidance.compute(params, input);
  assert(out.ey_dot_inertial < -0.99F);
  assert(std::abs(out.pitch_rate_rad_s) > 2.0F);
}

void testStrikePngControllerTiltEnvelopeHardCapLevels() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.derotate_body_rates = false;
  params.max_roll_rate_rad_s = 1.2F;
  params.fov_trim_kp_rate = 0.0F;
  params.tilt_cap.enable = true;
  params.tilt_cap.max_roll_angle_deg = 30.0F;
  params.tilt_cap.hardcap_margin_deg = 6.0F;
  params.tilt_cap.hardcap_level_kp = 3.0F;
  params.tilt_cap.hardcap_max_level_rate_deg_s = 86.0F;  // ~1.5 rad/s

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  first.bbox_area_ratio = 0.002F;
  first.fx = 500.0F;
  first.fy = 500.0F;
  first.attitude_valid = true;
  first.vehicle_roll_rad = 1.0F;  // ~57 deg, past 30+6 deg hard threshold
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput second = first;
  second.now_ns += 50000000ULL;
  second.ex = 0.02F;  // would command a positive (tilt-increasing) roll rate
  const auto out = controller.tick(params, second);

  assert(out.tilt_hardcap_active);
  // Hard cap overrides the command with a leveling rate toward 0 attitude
  // (negative for positive roll), clamped to the deg/s limit (~1.5 rad/s).
  assert(out.roll_rate_rad_s < 0.0F);
  assert(out.roll_rate_rad_s >= -1.5F - 1.0e-3F);
}

void testStrikePngControllerTiltEnvelopeSoftCapAttenuates() {
  const float band = 0.2F;
  // Increasing tilt inside the soft band attenuates; reducing tilt does not.
  const float att = 0.45F;        // inside [max-band, max] = [0.32, 0.52]
  const float max_angle = 0.52F;  // ~30 deg
  const float f_increasing = circle::strike::tiltSoftcapFactor(att, 1.0F,
                                                               max_angle, band);
  const float f_reducing = circle::strike::tiltSoftcapFactor(att, -1.0F,
                                                             max_angle, band);
  assert(f_increasing > 0.0F && f_increasing < 1.0F);
  assert(f_reducing == 1.0F);
}

void testStrikePngControllerHoldsLosRateBetweenRepeatedSetpointTicks() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.derotate_body_rates = false;
  params.max_roll_rate_rad_s = 1.2F;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.los_rate_hold_tau_s = 0.20F;
  params.fov_trim_kp_rate = 0.0F;

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.measurement_ns = 1000000000ULL;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput second = first;
  second.now_ns += 10000000ULL;
  second.measurement_ns += 100000000ULL;
  second.ex = 0.02F;
  const auto with_new_measurement = controller.tick(params, second);

  circle::strike_png::StrikePngInput repeated = second;
  repeated.now_ns += 10000000ULL;
  const auto repeated_tick = controller.tick(params, repeated);

  assert(with_new_measurement.ex_dot_filt > 0.15F);
  assert(repeated_tick.ex_dot_filt > 0.10F);
  assert(repeated_tick.ex_dot_filt < with_new_measurement.ex_dot_filt);
  assert(repeated_tick.roll_rate_rad_s > 0.05F);
}

void testStrikePngControllerPredictsDelayedVisualMeasurement() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.derotate_body_rates = false;
  params.max_roll_rate_rad_s = 1.2F;
  params.fov_trim_kp_rate = 1.0F;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.los_rate_hold_tau_s = 0.20F;
  params.edge_guard_enable = false;
  params.pursuit_fallback_enable = false;
  params.terminal_intercept_enable = false;
  params.terminal_crossing_enable = false;

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.measurement_ns = 1000000000ULL;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput delayed = first;
  delayed.now_ns = 1200000000ULL;
  delayed.measurement_ns = 1050000000ULL;
  delayed.ex = 0.05F;
  const auto out = controller.tick(params, delayed);

  assert(out.has_target);
  assert(out.png_active);
  assert(out.ex_dot_filt > 0.90F);
  assert(out.roll_rate_rad_s > 0.12F);
}

void testStrikePngControllerUsesDkfLosEstimate() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.dkf_enable = true;
  params.dkf.enable = true;
  params.dkf.process_accel_noise = 100.0F;
  params.dkf.meas_noise_px = 4.0F;
  params.dkf.predict_extra_delay_s = 0.0F;
  params.dkf.max_cov_trace = 10.0F;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.derotate_body_rates = false;
  params.fov_trim_kp_rate = 0.0F;
  params.edge_guard_enable = false;
  params.pursuit_fallback_enable = false;

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.measurement_ns = first.now_ns;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  first.bbox_area_px = 1600.0F;
  first.detection_score = 0.90F;
  first.fx = 500.0F;
  first.fy = 500.0F;
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput second = first;
  second.now_ns += 50000000ULL;
  second.measurement_ns += 50000000ULL;
  second.ex = 0.05F;
  const auto out = controller.tick(params, second);

  assert(out.has_target);
  assert(out.png_active);
  assert(out.ex_dot_filt > 0.01F);
  assert(out.roll_rate_rad_s > 0.01F);
}

void testStrikePngControllerDkfDisabledKeepsLegacyLosRate() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.dkf_enable = false;
  params.pixel_dot_lpf_tau_s = 0.0F;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.derotate_body_rates = false;
  params.fov_trim_kp_rate = 0.0F;
  params.edge_guard_enable = false;
  params.pursuit_fallback_enable = false;

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.measurement_ns = first.now_ns;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput second = first;
  second.now_ns += 50000000ULL;
  second.measurement_ns += 50000000ULL;
  second.ex = 0.05F;
  const auto out = controller.tick(params, second);

  assert(std::abs(out.ex_dot_filt - 1.0F) < 1.0e-4F);
  assert(out.roll_rate_rad_s > 0.5F);
}

void testStrikePngControllerDkfRejectsOutOfOrderMeasurement() {
  circle::strike_png::StrikePngController controller;
  circle::strike_png::StrikePngParams params;
  params.enable = true;
  params.dkf_enable = true;
  params.dkf.enable = true;
  params.dkf.process_accel_noise = 100.0F;
  params.dkf.meas_noise_px = 4.0F;
  params.dkf.predict_extra_delay_s = 0.0F;
  params.dkf.max_cov_trace = 10.0F;
  params.los_rate_hold_tau_s = 0.0F;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 1.0F;
  params.closure_area_gain = 0.0F;
  params.derotate_body_rates = false;
  params.fov_trim_kp_rate = 0.0F;
  params.edge_guard_enable = false;
  params.pursuit_fallback_enable = false;

  circle::strike_png::StrikePngInput first;
  first.now_ns = 1000000000ULL;
  first.measurement_ns = first.now_ns;
  first.detection_valid = true;
  first.ex = 0.00F;
  first.ey = 0.00F;
  first.bbox_area_px = 1600.0F;
  first.detection_score = 0.90F;
  first.fx = 500.0F;
  first.fy = 500.0F;
  (void)controller.tick(params, first);

  circle::strike_png::StrikePngInput second = first;
  second.now_ns += 50000000ULL;
  second.measurement_ns += 50000000ULL;
  second.ex = 0.05F;
  const auto with_new_measurement = controller.tick(params, second);

  circle::strike_png::StrikePngInput stale = second;
  stale.now_ns += 50000000ULL;
  stale.measurement_ns = first.measurement_ns;
  stale.ex = -0.50F;
  const auto out = controller.tick(params, stale);

  assert(out.ex_dot_filt >= 0.0F);
  assert(out.ex_dot_filt <= with_new_measurement.ex_dot_filt + 1.0e-4F);
  assert(out.roll_rate_rad_s >= 0.0F);
}

void testStrikePngGuidanceGuardsTerminalFovWithoutLosRate() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 2.0F;
  params.closure_base_scale = 0.7F;
  params.closure_area_gain = 0.35F;
  params.max_feedforward_rad_s = 0.9F;
  params.fov_trim_kp_rate = 0.15F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.35F;
  input.ey = 0.0F;
  input.ex_dot = 0.0F;
  input.ey_dot = 0.0F;
  input.bbox_area_ratio = 0.30F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(out.roll_rate_rad_s > 0.20F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
}

void testStrikePngGuidanceOutputSignMirrorsAxes() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 2.0F;
  params.closure_base_scale = 0.7F;
  params.closure_area_gain = 0.35F;
  params.max_feedforward_rad_s = 0.9F;
  params.fov_trim_kp_rate = 0.15F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.30F;
  input.ey = 0.30F;
  input.bbox_area_ratio = 0.30F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  // PX4 baseline: lateral=+1 -> roll follows +ex; longitudinal=-1 -> pitch = -ey.
  const auto base = guidance.compute(params, input);
  assert(base.roll_rate_rad_s > 0.05F);
  assert(base.pitch_rate_rad_s < -0.05F);

  // Flipping each sign mirrors the corresponding axis (BF needs longitudinal=+1).
  params.lateral_output_sign = -1.0F;
  params.longitudinal_output_sign = 1.0F;
  const auto flipped = guidance.compute(params, input);
  assert(std::abs(flipped.roll_rate_rad_s + base.roll_rate_rad_s) < 1.0e-5F);
  assert(std::abs(flipped.pitch_rate_rad_s + base.pitch_rate_rad_s) < 1.0e-5F);
  assert(flipped.pitch_rate_rad_s > 0.05F);
}

void testStrikePngGuidanceKeepsPursuingWhenLosRateIsSparse() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 3.0F;
  params.nav_ratio_y = 2.0F;
  params.closure_base_scale = 0.7F;
  params.closure_area_gain = 0.35F;
  params.max_feedforward_rad_s = 0.9F;
  params.fov_trim_kp_rate = 0.15F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = -0.06F;
  input.ey = 0.30F;
  input.ex_dot = 0.0F;
  input.ey_dot = 0.0F;
  input.bbox_area_ratio = 0.0005F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(out.pitch_rate_rad_s < -0.20F);
}

void testStrikePngGuidanceFadesFovTrimNearTerminalArea() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.40F;
  params.fov_trim_fade_area_ratio_start = 0.002F;
  params.fov_trim_fade_area_ratio_full = 0.010F;
  params.edge_guard_enable = false;
  params.pursuit_fallback_enable = false;
  params.terminal_intercept_enable = false;
  params.terminal_crossing_enable = false;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.20F;
  input.ey = 0.0F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  input.bbox_area_ratio = 0.001F;
  const auto far = guidance.compute(params, input);
  input.bbox_area_ratio = 0.020F;
  const auto close = guidance.compute(params, input);

  assert(far.roll_rate_rad_s > 0.07F);
  assert(std::abs(close.roll_rate_rad_s) < 1.0e-6F);
}

void testStrikePngGuidanceBoostsLateralTrimOnlyForStaleTerminalMeasurements() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.15F;
  params.terminal_stale_lateral_trim_enable = true;
  params.terminal_stale_lateral_trim_area_ratio_start = 0.002F;
  params.terminal_stale_lateral_trim_area_ratio_full = 0.006F;
  params.terminal_stale_lateral_trim_stale_s_start = 0.08F;
  params.terminal_stale_lateral_trim_stale_s_full = 0.20F;
  params.terminal_stale_lateral_trim_kp_rate = 2.0F;
  params.terminal_stale_lateral_trim_max_rate_rad_s = 0.20F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.05F;
  input.ey = 0.0F;
  input.bbox_area_ratio = 0.008F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  input.measurement_age_s = 0.02F;
  const auto fresh = guidance.compute(params, input);

  input.measurement_age_s = 0.22F;
  const auto stale = guidance.compute(params, input);

  assert(fresh.roll_terminal_stale_trim_rad_s == 0.0F);
  assert(stale.roll_terminal_stale_trim_rad_s > 0.05F);
  assert(stale.roll_rate_rad_s > fresh.roll_rate_rad_s + 0.05F);
}

void testStrikePngGuidanceAddsTerminalFutureMissCorrection() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_intercept_enable = true;
  params.terminal_intercept_area_ratio_start = 0.002F;
  params.terminal_intercept_area_ratio_full = 0.006F;
  params.terminal_intercept_lead_s = 0.20F;
  params.terminal_intercept_kp_rate = 2.0F;
  params.terminal_intercept_max_rate_rad_s = 0.30F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.02F;
  input.ey = -0.01F;
  input.ex_dot = 0.50F;
  input.ey_dot = -0.40F;
  input.bbox_area_ratio = 0.008F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(out.terminal_intercept_active);
  assert(out.terminal_intercept_lead_s > 0.19F);
  assert(out.terminal_future_ex > input.ex);
  assert(out.terminal_future_ey < input.ey);
  assert(out.roll_terminal_intercept_rad_s > 0.20F);
  assert(out.pitch_terminal_intercept_rad_s > 0.15F);
  assert(out.roll_rate_rad_s > 0.20F);
  assert(out.pitch_rate_rad_s > 0.15F);
}

void testStrikePngGuidanceAddsTerminalCrossingRateCorrection() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_intercept_enable = false;
  params.terminal_crossing_enable = true;
  params.terminal_crossing_area_ratio_start = 0.001F;
  params.terminal_crossing_area_ratio_full = 0.004F;
  params.terminal_crossing_rate_start_norm_s = 0.05F;
  params.terminal_crossing_rate_full_norm_s = 0.30F;
  params.terminal_crossing_kd_rate = 0.80F;
  params.terminal_crossing_max_rate_rad_s = 0.25F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.01F;
  input.ey = 0.08F;
  input.ex_dot = 0.02F;
  input.ey_dot = -0.35F;
  input.bbox_area_ratio = 0.008F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(out.terminal_crossing_active);
  assert(out.pitch_terminal_crossing_rad_s > 0.20F);
  assert(out.pitch_rate_rad_s > 0.20F);
}

void testStrikePngGuidanceUsesVerticalAimBias() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.20F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.vertical_aim_ey = 0.30F;
  params.terminal_intercept_enable = true;
  params.terminal_intercept_area_ratio_start = 0.001F;
  params.terminal_intercept_area_ratio_full = 0.004F;
  params.terminal_intercept_lead_s = 0.20F;
  params.terminal_intercept_kp_rate = 1.0F;
  params.terminal_intercept_max_rate_rad_s = 0.50F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.0F;
  input.ey = 0.30F;
  input.ex_dot = 0.0F;
  input.ey_dot = 0.0F;
  input.bbox_area_ratio = 0.008F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(out.terminal_intercept_active);
  assert(std::abs(out.terminal_future_ey) < 1.0e-6F);
  assert(std::abs(out.pitch_terminal_intercept_rad_s) < 1.0e-6F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
}

void testStrikePngGuidanceUsesTerminalTiltAimBiasNearTarget() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_tilt_aim_area_ratio_start = 0.02F;
  params.terminal_tilt_aim_area_ratio_full = 0.08F;
  params.terminal_tilt_aim_roll_gain = 0.50F;
  params.terminal_tilt_aim_pitch_gain = 0.25F;
  params.terminal_tilt_aim_max_offset_norm = 0.20F;
  params.terminal_intercept_enable = true;
  params.terminal_intercept_area_ratio_start = 0.01F;
  params.terminal_intercept_area_ratio_full = 0.04F;
  params.terminal_intercept_lead_s = 0.0F;
  params.terminal_intercept_kp_rate = 2.0F;
  params.terminal_intercept_max_rate_rad_s = 0.50F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.20F;
  input.ey = -0.06F;
  input.bbox_area_ratio = 0.10F;
  input.attitude_valid = true;
  input.roll_rad = 0.30F;
  input.pitch_rad = -0.20F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(out.terminal_intercept_active);
  assert(out.terminal_aim_ex > 0.10F);
  assert(out.terminal_aim_ey < -0.02F);
  assert(out.terminal_future_ex < input.ex);
  assert(out.terminal_future_ey > input.ey);
  assert(out.roll_terminal_intercept_rad_s < 0.25F);
}

void testStrikePngGuidanceDoesNotUseTerminalCrossingWhenFar() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_intercept_enable = false;
  params.terminal_crossing_enable = true;
  params.terminal_crossing_area_ratio_start = 0.001F;
  params.terminal_crossing_area_ratio_full = 0.004F;
  params.terminal_crossing_rate_start_norm_s = 0.05F;
  params.terminal_crossing_rate_full_norm_s = 0.30F;
  params.terminal_crossing_kd_rate = 0.80F;
  params.terminal_crossing_max_rate_rad_s = 0.25F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.01F;
  input.ey = 0.08F;
  input.ex_dot = 0.02F;
  input.ey_dot = -0.35F;
  input.bbox_area_ratio = 0.0005F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(!out.terminal_crossing_active);
  assert(std::abs(out.pitch_terminal_crossing_rad_s) < 1.0e-6F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
}

void testStrikePngGuidanceDoesNotUseTerminalFutureMissWhenFar() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_intercept_enable = true;
  params.terminal_intercept_area_ratio_start = 0.002F;
  params.terminal_intercept_area_ratio_full = 0.006F;
  params.terminal_intercept_lead_s = 0.20F;
  params.terminal_intercept_kp_rate = 2.0F;
  params.terminal_intercept_max_rate_rad_s = 0.30F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ex = 0.02F;
  input.ey = -0.01F;
  input.ex_dot = 0.50F;
  input.ey_dot = -0.40F;
  input.bbox_area_ratio = 0.0005F;
  input.max_roll_rate_rad_s = 1.2F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto out = guidance.compute(params, input);
  assert(out.active);
  assert(!out.terminal_intercept_active);
  assert(std::abs(out.roll_terminal_intercept_rad_s) < 1.0e-6F);
  assert(std::abs(out.pitch_terminal_intercept_rad_s) < 1.0e-6F);
  assert(std::abs(out.roll_rate_rad_s) < 1.0e-6F);
  assert(std::abs(out.pitch_rate_rad_s) < 1.0e-6F);
}

void testStrikePngGuidanceScalesPositivePitchAtHighTerminalSpeed() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_crossing_enable = true;
  params.terminal_crossing_area_ratio_start = 0.001F;
  params.terminal_crossing_area_ratio_full = 0.004F;
  params.terminal_crossing_rate_start_norm_s = 0.05F;
  params.terminal_crossing_rate_full_norm_s = 0.30F;
  params.terminal_crossing_kd_rate = 0.80F;
  params.terminal_crossing_max_rate_rad_s = 0.80F;
  params.terminal_forward_speed_guard_enable = true;
  params.terminal_forward_speed_guard_area_ratio_start = 0.001F;
  params.terminal_forward_speed_guard_area_ratio_full = 0.004F;
  params.terminal_forward_speed_guard_start_m_s = 30.0F;
  params.terminal_forward_speed_guard_full_m_s = 40.0F;
  params.terminal_forward_speed_guard_min_positive_pitch_scale = 0.25F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ey_dot = -0.40F;
  input.bbox_area_ratio = 0.008F;
  input.max_pitch_rate_rad_s = 1.2F;

  const auto no_speed = guidance.compute(params, input);
  assert(no_speed.terminal_crossing_active);
  assert(!no_speed.terminal_forward_speed_guard_active);
  assert(no_speed.pitch_rate_rad_s > 0.30F);

  input.ownship_forward_speed_valid = true;
  input.ownship_forward_speed_m_s = 42.0F;
  const auto guarded = guidance.compute(params, input);
  assert(guarded.terminal_forward_speed_guard_active);
  assert(guarded.terminal_forward_speed_guard_weight > 0.99F);
  assert(guarded.terminal_forward_speed_guard_scale < 0.26F);
  assert(guarded.pitch_rate_rad_s < no_speed.pitch_rate_rad_s * 0.30F);
}

void testStrikePngGuidanceDoesNotScaleNegativePitchAtHighTerminalSpeed() {
  circle::strike_png::VisualPngGuidance guidance;
  circle::strike_png::VisualPngGuidanceParams params;
  params.derotate_body_rates = false;
  params.nav_ratio_x = 0.0F;
  params.nav_ratio_y = 0.0F;
  params.closure_base_scale = 0.0F;
  params.closure_area_gain = 0.0F;
  params.max_feedforward_rad_s = 0.0F;
  params.fov_trim_kp_rate = 0.0F;
  params.pursuit_fallback_enable = false;
  params.edge_guard_enable = false;
  params.terminal_crossing_enable = true;
  params.terminal_crossing_area_ratio_start = 0.001F;
  params.terminal_crossing_area_ratio_full = 0.004F;
  params.terminal_crossing_rate_start_norm_s = 0.05F;
  params.terminal_crossing_rate_full_norm_s = 0.30F;
  params.terminal_crossing_kd_rate = 0.80F;
  params.terminal_crossing_max_rate_rad_s = 0.80F;
  params.terminal_forward_speed_guard_enable = true;
  params.terminal_forward_speed_guard_area_ratio_start = 0.001F;
  params.terminal_forward_speed_guard_area_ratio_full = 0.004F;
  params.terminal_forward_speed_guard_start_m_s = 30.0F;
  params.terminal_forward_speed_guard_full_m_s = 40.0F;
  params.terminal_forward_speed_guard_min_positive_pitch_scale = 0.25F;

  circle::strike_png::VisualPngGuidanceInput input;
  input.ey_dot = 0.40F;
  input.bbox_area_ratio = 0.008F;
  input.max_pitch_rate_rad_s = 1.2F;
  input.ownship_forward_speed_valid = true;
  input.ownship_forward_speed_m_s = 42.0F;

  const auto out = guidance.compute(params, input);
  assert(out.terminal_forward_speed_guard_active);
  assert(out.pitch_rate_rad_s < -0.30F);
}

void testStrikePngTargetLossHoldRequestsOnceAfterPostContactLoss() {
  circle::strike_png::TargetLossHoldState state;
  constexpr uint64_t kSecondNs = 1000000000ULL;
  constexpr uint64_t kDelayNs = 200000000ULL;

  assert(!circle::strike_png::updateTargetLossHold(
      state, true, false, kSecondNs, kDelayNs, true));
  assert(!state.has_seen_target);

  assert(!circle::strike_png::updateTargetLossHold(
      state, true, true, kSecondNs + 10000000ULL, kDelayNs, true));
  assert(state.has_seen_target);

  assert(!circle::strike_png::updateTargetLossHold(
      state, true, false, kSecondNs + 50000000ULL, kDelayNs, true));
  assert(!circle::strike_png::updateTargetLossHold(
      state, true, false, kSecondNs + 200000000ULL, kDelayNs, true));
  assert(circle::strike_png::updateTargetLossHold(
      state, true, false, kSecondNs + 260000000ULL, kDelayNs, true));
  assert(!circle::strike_png::updateTargetLossHold(
      state, true, false, kSecondNs + 500000000ULL, kDelayNs, true));

  assert(!circle::strike_png::updateTargetLossHold(
      state, true, true, kSecondNs + 600000000ULL, kDelayNs, true));
  assert(!circle::strike_png::updateTargetLossHold(
      state, false, false, kSecondNs + 900000000ULL, kDelayNs, true));
  assert(!state.has_seen_target);
}

void testStrikePngTargetLossClassifiesImageContext() {
  circle::strike_png::TargetLossImageContext ctx;
  assert(std::string(circle::strike_png::classifyTargetLossImageContext(
             ctx, 24.0F, 0.08F)) == "no_detection");

  ctx.valid = true;
  ctx.image_width_px = 640.0F;
  ctx.image_height_px = 480.0F;
  ctx.cx_px = 320.0F;
  ctx.cy_px = 20.0F;
  ctx.width_px = 100.0F;
  ctx.height_px = 60.0F;
  ctx.bbox_area_ratio = 0.02F;
  assert(std::string(circle::strike_png::classifyTargetLossImageContext(
             ctx, 24.0F, 0.08F)) == "edge_top");

  ctx.cy_px = 240.0F;
  ctx.width_px = 240.0F;
  ctx.height_px = 160.0F;
  ctx.bbox_area_ratio = 0.125F;
  assert(std::string(circle::strike_png::classifyTargetLossImageContext(
             ctx, 24.0F, 0.08F)) == "large_bbox");
}

void testStrikePngEntryHandoffSmoothlyBlendsToTargetCommand() {
  const circle::strike_png::EntryHandoffParams params{
      true, 1.0F, 0.40F};
  const circle::strike_png::EntryHandoffSnapshot snapshot{
      1000000000ULL, 0.20F, -0.10F, 0.50F};
  const circle::strike_png::EntryHandoffCommand target{
      1.00F, -0.80F, 0.80F};

  const auto start = circle::strike_png::applyEntryHandoff(
      params, snapshot, target, 1000000000ULL);
  assert(std::abs(start.roll_rate_rad_s - 0.20F) < 1.0e-6F);
  assert(std::abs(start.pitch_rate_rad_s + 0.10F) < 1.0e-6F);
  assert(std::abs(start.thrust_z - 0.50F) < 1.0e-6F);
  assert(start.active);

  const auto middle = circle::strike_png::applyEntryHandoff(
      params, snapshot, target, 1500000000ULL);
  assert(middle.roll_rate_rad_s > start.roll_rate_rad_s);
  assert(middle.roll_rate_rad_s < target.roll_rate_rad_s);
  assert(middle.pitch_rate_rad_s < start.pitch_rate_rad_s);
  assert(middle.pitch_rate_rad_s > target.pitch_rate_rad_s);
  assert(middle.thrust_z > start.thrust_z);
  assert(middle.thrust_z < target.thrust_z);
  assert(middle.active);

  const auto complete = circle::strike_png::applyEntryHandoff(
      params, snapshot, target, 2200000000ULL);
  assert(std::abs(complete.roll_rate_rad_s - target.roll_rate_rad_s) <
         1.0e-6F);
  assert(std::abs(complete.pitch_rate_rad_s - target.pitch_rate_rad_s) <
         1.0e-6F);
  assert(std::abs(complete.thrust_z - target.thrust_z) < 1.0e-6F);
  assert(!complete.active);
}

}  // namespace

int main() {
  testYoloPostprocess();
  testSotByteTrackSingleTarget();
  testSotByteTrackPredictionAndExpiry();
  testSotByteTrackRejectsOutOfOrderFrames();
  testMathUtils();
  testRateShaper();
  testEdgeProtection();
  testFinalApproachGate();
  testSpeedGovernor();
  testPreclimbModule();
  testThrustManager();
  testCommitModule();
  testCommitFutureGateBlocksProjectedMiss();
  testTerminalPredictor();
  testVisualPngGuidanceDerotatesBodyRates();
  testVisualPngGuidanceCommandsLosRateSuppression();
  testRhoRateWindowEstimatesClosingFromAreaGrowth();
  testControllerStateTransitions();
  testControllerPaperPngGuidanceDiffersFromLegacyPd();
  testApproachDriveFovGateSuppressesHighTargetPitchBias();
  testControllerTerminalPredictorTelemetry();
  testControllerUsesDkfEstimateWhenEnabled();
  testControllerAcceptsFreshReceiveStampWithOldCaptureStamp();
  testFinalApproachGateUsesCurrentImageRate();
  testCloseRangeFinalApproachKeepsRollLeveling();
  testTerminalEdgeProtectSurvivesFullRollLeveling();
  testRepeatedDetectionDoesNotKeepTerminalEdgeBoosting();
  testTargetLossCompletesFromFinalApproachFallback();
  testParameterClamping();
  testTelemetryOutput();
  testStrikePngControllerStartsInactiveWithoutTarget();
  testStrikePngControllerCommandsLosRateSuppression();
  testStrikePngGuidanceRequiresValidDerotationRate();
  testStrikePngControllerTiltEnvelopeHardCapLevels();
  testStrikePngControllerTiltEnvelopeSoftCapAttenuates();
  testStrikePngControllerHoldsLosRateBetweenRepeatedSetpointTicks();
  testStrikePngControllerPredictsDelayedVisualMeasurement();
  testStrikePngControllerUsesDkfLosEstimate();
  testStrikePngControllerDkfDisabledKeepsLegacyLosRate();
  testStrikePngControllerDkfRejectsOutOfOrderMeasurement();
  testStrikePngGuidanceGuardsTerminalFovWithoutLosRate();
  testStrikePngGuidanceOutputSignMirrorsAxes();
  testStrikePngGuidanceKeepsPursuingWhenLosRateIsSparse();
  testStrikePngGuidanceFadesFovTrimNearTerminalArea();
  testStrikePngGuidanceBoostsLateralTrimOnlyForStaleTerminalMeasurements();
  testStrikePngGuidanceAddsTerminalFutureMissCorrection();
  testStrikePngGuidanceAddsTerminalCrossingRateCorrection();
  testStrikePngGuidanceUsesVerticalAimBias();
  testStrikePngGuidanceUsesTerminalTiltAimBiasNearTarget();
  testStrikePngGuidanceDoesNotUseTerminalCrossingWhenFar();
  testStrikePngGuidanceDoesNotUseTerminalFutureMissWhenFar();
  testStrikePngGuidanceScalesPositivePitchAtHighTerminalSpeed();
  testStrikePngGuidanceDoesNotScaleNegativePitchAtHighTerminalSpeed();
  testStrikePngTargetLossHoldRequestsOnceAfterPostContactLoss();
  testStrikePngTargetLossClassifiesImageContext();
  testStrikePngEntryHandoffSmoothlyBlendsToTargetCommand();
  return 0;
}
