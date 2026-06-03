#pragma once

#include <string>

#include "circle/bf/runtime/bf_strike_controller_iface.hpp"
#include "circle/strike/strike_controller.hpp"
#include "circle/strike/strike_params.hpp"

namespace circle::bf::flight {

/**
 * Adapts the full circle::strike::StrikeController to the shared BF runtime
 * host. Mapping is byte-equivalent to the original inline bf_flight loop:
 *   - StrikeState -> Command via bfShouldCommandAlgorithm + ForceLevel,
 *   - telemetry / overlay fields carried verbatim from StrikeOutputs/params,
 *   - online tuning through the existing strike_param_tune helpers.
 */
class StrikeControllerAdapter final : public circle::bf::runtime::IBfStrikeController {
 public:
  explicit StrikeControllerAdapter(circle::strike::StrikeParams params);

  void onEngageRisingEdge(const circle::bf::runtime::BfControlContext& ctx) override;
  circle::bf::runtime::BfControlResult update(
      const circle::bf::runtime::BfControlContext& ctx) override;
  bool applyParamUpdateJson(const std::string& json) override;
  std::string paramsSnapshotJson() const override;
  void fillTelemetry(circle::ipc::StrikeTelemetrySample& sample,
                     const circle::bf::runtime::BfControlResult& result) const override;
  void fillOverlay(circle::debug::PreviewOverlayContext& overlay,
                   const circle::bf::runtime::BfControlResult& result) const override;
  float hoverThrottleNorm() const override { return params_.thrust.hover_scalar; }
  const char* modeTag() const override { return "target_strike"; }
  const char* stateName(
      const circle::bf::runtime::BfControlResult& result) const override;

 private:
  circle::strike::StrikeParams params_;
  circle::strike::StrikeController controller_;
  circle::strike::StrikeOutputs last_outputs_{};
};

}  // namespace circle::bf::flight
