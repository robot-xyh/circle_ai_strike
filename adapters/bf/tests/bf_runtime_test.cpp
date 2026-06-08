// Unit + smoke tests for the shared BF runtime (circle_bf_runtime) and the two
// controller adapters. No hardware / MSP / camera: pure-function gating, the
// throttle-handover blend, and adapter input->output mapping smoke runs.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "circle/bf/runtime/bf_control_host.hpp"
#include "circle/bf/runtime/bf_gating.hpp"
#include "circle/bf/runtime/bf_strike_controller_iface.hpp"
#include "circle/debug_common/strike_png_param_tune.hpp"
#include "circle/ipc/strike_telemetry_shm.hpp"
#include "circle/strike_png/strike_png_node_params.hpp"
#include "circle/strike_png/strike_png_params_yaml.hpp"

#include <sys/mman.h>

// Adapter sources are compiled directly into this test target.
#include "png_controller_adapter.hpp"
#include "strike_controller_adapter.hpp"

namespace {

using circle::bf::runtime::BfControlContext;
using circle::bf::runtime::BfControlResult;
using circle::bf::runtime::BfPublishMode;

int g_failures = 0;

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) {                                                       \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
      ++g_failures;                                                      \
    }                                                                    \
  } while (0)

bool nearly(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }

void testDecideBfPublish() {
  using Cmd = BfControlResult::Command;
  // Base gate fails -> always PhysicalHold.
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, /*may=*/false, true,
                                             false, false, true, true) ==
        BfPublishMode::PhysicalHold);
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, true,
                                             /*override=*/false, false, false,
                                             true, true) ==
        BfPublishMode::PhysicalHold);
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, true, true,
                                             /*dry_run=*/true, false, true,
                                             true) == BfPublishMode::PhysicalHold);
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, true, true, false,
                                             /*wd_tripped=*/true, true, true) ==
        BfPublishMode::PhysicalHold);
  // Algorithm with fresh detection satisfied.
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, true, true, false,
                                             false, /*require_fresh=*/true,
                                             /*fresh_seen=*/true) ==
        BfPublishMode::Algorithm);
  // Algorithm but fresh required and not seen -> PhysicalHold.
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, true, true, false,
                                             false, true, false) ==
        BfPublishMode::PhysicalHold);
  // Algorithm when fresh gate disabled.
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::Algorithm, true, true, false,
                                             false, false, false) ==
        BfPublishMode::Algorithm);
  // LevelOnly bypasses the fresh gate.
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::LevelOnly, true, true, false,
                                             false, true, false) ==
        BfPublishMode::LevelOnly);
  // None -> PhysicalHold.
  CHECK(circle::bf::runtime::decideBfPublish(Cmd::None, true, true, false, false,
                                             false, true) ==
        BfPublishMode::PhysicalHold);
}

void testThrottleBlend() {
  CHECK(nearly(circle::bf::runtime::blendThrottleHandover(0.2F, 0.8F, 0.0F), 0.2F));
  CHECK(nearly(circle::bf::runtime::blendThrottleHandover(0.2F, 0.8F, 1.0F), 0.8F));
  CHECK(nearly(circle::bf::runtime::blendThrottleHandover(0.2F, 0.8F, 0.5F), 0.5F));
}

BfControlContext makeCenteredContext(uint64_t now_ns, bool mode_active) {
  BfControlContext ctx;
  ctx.now_ns = now_ns;
  ctx.mode_active = mode_active;
  ctx.override_active = mode_active;
  ctx.image_width = 640;
  ctx.image_height = 512;
  ctx.intrinsics.fx = 600.0F;
  ctx.intrinsics.fy = 600.0F;
  ctx.intrinsics.cx = 320.0F;
  ctx.intrinsics.cy = 256.0F;
  ctx.vehicle.valid = true;
  ctx.vehicle.armed = true;
  ctx.detection.valid = true;
  ctx.detection.capture_ns = now_ns;
  ctx.detection.image_width = 640;
  ctx.detection.image_height = 512;
  ctx.detection.detection.cx = 360.0F;  // 40px right of center
  ctx.detection.detection.cy = 256.0F;
  ctx.detection.detection.width = 40.0F;
  ctx.detection.detection.height = 40.0F;
  ctx.detection.detection.score = 0.9F;
  ctx.detection.detection.class_name = "UAV";
  return ctx;
}

void testPngAdapterMapping() {
  circle::strike_png::StrikePngNodeParams params;
  params.dry_run = false;
  params.require_armed_to_command = true;
  params.hover_thrust_z = 0.30F;
  params.strike_thrust_z = 0.55F;
  params.entry_handoff.enable = false;  // isolate steady-state mapping
  circle::bf::png::PngControllerAdapter adapter(params);

  // Not engaged -> command None regardless of detection.
  BfControlResult idle = adapter.update(makeCenteredContext(1'000'000ULL, false));
  CHECK(idle.command == BfControlResult::Command::None);

  // Engage rising edge then run a few ticks with a fresh centered detection.
  uint64_t t = 10'000'000ULL;
  BfControlContext ctx = makeCenteredContext(t, true);
  adapter.onEngageRisingEdge(ctx);
  BfControlResult r;
  for (int i = 0; i < 5; ++i) {
    t += 10'000'000ULL;  // 10 ms
    ctx = makeCenteredContext(t, true);
    r = adapter.update(ctx);
  }
  // Engaged + fresh target -> Algorithm, has_target true, image error +x.
  CHECK(r.command == BfControlResult::Command::Algorithm);
  CHECK(r.has_target);
  CHECK(r.image_ex > 0.0F);
  CHECK(nearly(r.rates.thrust_z, params.strike_thrust_z));
  CHECK(std::string(adapter.modeTag()) == "target_strike_png");

  // PNG telemetry mapping: adapter must populate the png_* decomposition fields.
  circle::ipc::StrikeTelemetrySample sample;
  adapter.fillTelemetry(sample, r);
  CHECK(nearly(sample.bbox_area_ratio, ctx.detection.detection.width *
                                           ctx.detection.detection.height /
                                           (640.0F * 512.0F)));
  CHECK(std::isfinite(sample.png_ff_roll_rad_s));
  CHECK(std::isfinite(sample.png_closure_scale) && sample.png_closure_scale > 0.0F);
  // entry handoff disabled in this test -> progress latched at complete (1.0).
  CHECK(nearly(sample.png_entry_handoff_progress, 1.0F));
  CHECK(std::isfinite(sample.png_measurement_age_s) &&
        sample.png_measurement_age_s >= 0.0F);
  CHECK(sample.png_loss_hold_latched == 0);
}

void testPngSeriesJson() {
  const std::string name = "/cps_bf_runtime_test_png";
  ::shm_unlink(name.c_str());
  circle::ipc::StrikeTelemetryWriter writer;
  CHECK(writer.open(name, 64));

  circle::ipc::StrikeTelemetrySample s;
  s.stamp_ns = 1'000'000'000LL;
  s.controller_kind = 1;  // target_strike_png
  s.roll_rate_sp = 0.3F;
  s.pitch_rate_sp = -0.2F;
  s.png_ff_roll_rad_s = 0.25F;
  s.png_ff_pitch_rad_s = -0.15F;
  s.png_closure_scale = 0.8F;
  s.png_entry_handoff_progress = 1.0F;
  writer.publish(s);

  circle::ipc::StrikeTelemetryReader reader;
  CHECK(reader.open(name));
  const std::string json = reader.seriesJson();
  // PNG schema: tune_mode + backend + component keys present.
  CHECK(json.find("\"tune_mode\":\"target_strike_png\"") != std::string::npos);
  CHECK(json.find("\"strike_backend\":\"bf\"") != std::string::npos);
  CHECK(json.find("png_ff_roll_rad_s") != std::string::npos);
  CHECK(json.find("png_entry_handoff_progress") != std::string::npos);
  // BF has no measured body rates -> emitted as null, never a finite number.
  const size_t pos = json.find("\"vehicle_roll_rate_rad_s\":[");
  CHECK(pos != std::string::npos);
  CHECK(json.compare(pos + 27, 4, "null") == 0);

  reader.close();
  writer.close();
  ::shm_unlink(name.c_str());
}

void testStrikeAdapterSmoke() {
  circle::strike::StrikeParams params;
  circle::bf::flight::StrikeControllerAdapter adapter(params);
  BfControlContext ctx = makeCenteredContext(10'000'000ULL, false);
  adapter.onEngageRisingEdge(ctx);
  // Should not crash and produce a valid command enum.
  BfControlResult r = adapter.update(ctx);
  CHECK(r.command == BfControlResult::Command::None ||
        r.command == BfControlResult::Command::Algorithm ||
        r.command == BfControlResult::Command::LevelOnly);
  CHECK(std::string(adapter.modeTag()) == "target_strike");
  CHECK(adapter.stateName(r) != nullptr);
}

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
void testPngTuneRoundTrip() {
  circle::strike_png::StrikePngNodeParams params;
  // Apply an online update through the JSON path used by bf_debugd/SHM.
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.nav_ratio_x","value":4.5})"));
  CHECK(nearly(params.controller.nav_ratio_x, 4.5F));
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"strike_png.hover_thrust_z","value":0.42})"));
  CHECK(nearly(params.hover_thrust_z, 0.42F));
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.terminal_intercept_enable","value":false})"));
  CHECK(params.controller.terminal_intercept_enable == false);
  // Output-axis sign is online-tunable (BF top-cam needs longitudinal=+1).
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.longitudinal_output_sign","value":1.0})"));
  CHECK(nearly(params.controller.longitudinal_output_sign, 1.0F));
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"strike_png.lateral_output_sign","value":-1.0})"));
  CHECK(nearly(params.controller.lateral_output_sign, -1.0F));
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.dkf_enable","value":true})"));
  CHECK(params.controller.dkf_enable);
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.dkf.process_accel_noise","value":42.0})"));
  CHECK(nearly(params.controller.dkf.process_accel_noise, 42.0F));
  CHECK(circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.dkf.predict_extra_delay_s","value":0.04})"));
  CHECK(nearly(params.controller.dkf.predict_extra_delay_s, 0.04F));
  // Unknown key rejected.
  CHECK(!circle::debug_common::applyStrikePngParamUpdate(
      params, R"({"name":"target_strike_png.does_not_exist","value":1})"));

  // Snapshot JSON must advertise the PNG tune mode and contain the edited key.
  const std::string json = circle::debug_common::strikePngParamsJson(params);
  CHECK(json.find("target_strike_png") != std::string::npos);
  CHECK(json.find("nav_ratio_x") != std::string::npos);
  CHECK(json.find("dkf.process_accel_noise") != std::string::npos);
}

#if defined(CIRCLE_PILOT_CONFIG_DIR)
void testSharedRuntimeConfig() {
  using circle::bf::runtime::BfRuntimeConfig;
  const std::string dir = CIRCLE_PILOT_CONFIG_DIR;

  // PNG flight config references the shared bf_flight_common.yaml.
  BfRuntimeConfig png;
  circle::bf::runtime::loadBfRuntimeConfigFromYaml(
      dir + "/strike_png_bf_flight.yaml", png);
  CHECK(png.msp.device == "/dev/ttyS1");           // from shared file
  CHECK(nearly(png.rc.max_roll_rate_rad_s, 3.491F)); // shared Profile C rate map
  CHECK(nearly(static_cast<float>(png.throttle_handover_s), 0.4F));
  CHECK(nearly(static_cast<float>(png.control_loop_hz), 200.0F)); // shared default
  CHECK(png.log_level == "info");                  // shared default (no override)
  CHECK(nearly(png.conf_threshold, 0.10F));
  CHECK(nearly(static_cast<float>(png.detection_coast_s), 0.0F));
  CHECK(png.byte_track.enabled);
  CHECK(nearly(png.byte_track.high_score_threshold, 0.25F));
  CHECK(nearly(png.byte_track.low_score_threshold, 0.10F));
  CHECK(nearly(png.byte_track.new_track_threshold, 0.25F));
  CHECK(nearly(png.byte_track.match_iou_threshold, 0.30F));
  CHECK(nearly(png.byte_track.second_match_iou_threshold, 0.20F));
  CHECK(png.byte_track.max_lost_frames == 5);
  CHECK(png.byte_track.emit_prediction_on_miss);
  CHECK(png.byte_track.emit_prediction_max_frames == 5);
  CHECK(nearly(png.byte_track.min_box_size_px, 4.0F));
  CHECK(nearly(png.byte_track.max_dt_s, 0.12F));
  circle::strike_png::StrikePngNodeParams png_params =
      circle::strike_png::loadStrikePngParamsFromYaml(
          dir + "/strike_png_bf_flight.yaml");
  CHECK(png_params.controller.dkf_enable);
  CHECK(png_params.controller.dkf.enable);
  CHECK(nearly(png_params.controller.dkf.process_accel_noise, 4.0F));
  CHECK(nearly(png_params.controller.dkf.meas_noise_px, 4.0F));
  CHECK(nearly(png_params.controller.dkf.predict_extra_delay_s, 0.03F));
  CHECK(nearly(png_params.controller.dkf.max_cov_trace, 0.25F));

  // Strike flight config references the same shared file but overrides log_level.
  BfRuntimeConfig strike;
  circle::bf::runtime::loadBfRuntimeConfigFromYaml(
      dir + "/strike_bf_flight.yaml", strike);
  CHECK(strike.msp.device == "/dev/ttyS1");        // shared base still applied
  CHECK(nearly(strike.rc.max_roll_rate_rad_s, 3.491F));
  CHECK(strike.log_level == "debug");              // inline override wins
}
#endif
#endif

}  // namespace

int main() {
  testDecideBfPublish();
  testThrottleBlend();
  testPngAdapterMapping();
  testPngSeriesJson();
  testStrikeAdapterSmoke();
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  testPngTuneRoundTrip();
#if defined(CIRCLE_PILOT_CONFIG_DIR)
  testSharedRuntimeConfig();
#endif
#endif
  if (g_failures == 0) {
    std::printf("bf_runtime_test: all checks passed\n");
    return 0;
  }
  std::printf("bf_runtime_test: %d check(s) failed\n", g_failures);
  return 1;
}
