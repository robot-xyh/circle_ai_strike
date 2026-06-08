#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "circle/bf/runtime/bf_strike_controller_iface.hpp"
#include "circle/strike_png/entry_handoff.hpp"
#include "circle/strike_png/strike_png_controller.hpp"
#include "circle/strike_png/strike_png_node_params.hpp"
#include "circle/strike_png/target_loss_hold.hpp"

namespace circle::bf::png {

/**
 * Adapts circle::strike_png::StrikePngController to the shared BF runtime host.
 * Mirrors the PX4 target_strike_png node 1:1: same input construction (ex/ey
 * from intrinsics, bbox_area_ratio, body rates), same thrust policy
 * (hover/strike), entry handoff blend and target-loss hold release. The host
 * owns OVERRIDE gating, throttle handover, watchdog, MSP wire and SHM.
 */
class PngControllerAdapter final : public circle::bf::runtime::IBfStrikeController {
 public:
  explicit PngControllerAdapter(circle::strike_png::StrikePngNodeParams params);

  void onEngageRisingEdge(const circle::bf::runtime::BfControlContext& ctx) override;
  circle::bf::runtime::BfControlResult update(
      const circle::bf::runtime::BfControlContext& ctx) override;
  bool applyParamUpdateJson(const std::string& json) override;
  std::string paramsSnapshotJson() const override;
  void fillTelemetry(circle::ipc::StrikeTelemetrySample& sample,
                     const circle::bf::runtime::BfControlResult& result) const override;
  void fillOverlay(circle::debug::PreviewOverlayContext& overlay,
                   const circle::bf::runtime::BfControlResult& result) const override;
  float hoverThrottleNorm() const override { return params_.hover_thrust_z; }
  const char* modeTag() const override { return "target_strike_png"; }
  const char* stateName(
      const circle::bf::runtime::BfControlResult& result) const override;

 private:
  struct BodyRateSample {
    uint64_t stamp_ns{0};
    float roll_rate_rad_s{0.0F};
    float pitch_rate_rad_s{0.0F};
    float yaw_rate_rad_s{0.0F};
    bool valid{false};
  };

  struct BodyRateLookup {
    BodyRateSample sample{};
    float interp_gap_ms{0.0F};
  };

  static constexpr size_t kBodyRateHistorySize = 256;

  void resetBodyRateHistory();
  void ingestBodyRateSample(const circle::types::FcState& vehicle);
  [[nodiscard]] bool lookupBodyRate(uint64_t lookup_ns,
                                    BodyRateLookup& out) const;
  [[nodiscard]] static float angleDeltaRad(float newer_rad, float older_rad);
  [[nodiscard]] static uint64_t subtractNs(uint64_t stamp_ns, int64_t offset_ns);

  circle::strike_png::StrikePngNodeParams params_;
  circle::strike_png::StrikePngController controller_;
  circle::strike_png::TargetLossHoldState loss_state_{};
  circle::strike_png::EntryHandoffSnapshot entry_snapshot_{};
  circle::strike_png::StrikePngInput last_input_{};
  circle::strike_png::StrikePngOutput last_output_{};
  std::array<BodyRateSample, kBodyRateHistorySize> body_rate_history_{};
  size_t body_rate_start_{0};
  size_t body_rate_count_{0};
  uint64_t prev_vehicle_stamp_ns_{0};
  float prev_vehicle_roll_rad_{0.0F};
  float prev_vehicle_pitch_rad_{0.0F};
  float prev_vehicle_yaw_rad_{0.0F};
  bool prev_vehicle_valid_{false};
  bool last_derotate_lookup_valid_{false};
  float last_derotate_lookup_age_ms_{0.0F};
  float last_derotate_interp_gap_ms_{0.0F};
  float last_derotate_roll_rate_rad_s_{0.0F};
  float last_derotate_pitch_rate_rad_s_{0.0F};
  float last_detection_score_{0.0F};
  float last_handoff_progress_{1.0F};
  bool last_loss_hold_latched_{false};
};

}  // namespace circle::bf::png
