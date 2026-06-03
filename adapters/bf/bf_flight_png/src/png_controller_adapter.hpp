#pragma once

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
  circle::strike_png::StrikePngNodeParams params_;
  circle::strike_png::StrikePngController controller_;
  circle::strike_png::TargetLossHoldState loss_state_{};
  circle::strike_png::EntryHandoffSnapshot entry_snapshot_{};
  circle::strike_png::StrikePngInput last_input_{};
  circle::strike_png::StrikePngOutput last_output_{};
  float last_detection_score_{0.0F};
  float last_handoff_progress_{1.0F};
  bool last_loss_hold_latched_{false};
};

}  // namespace circle::bf::png
