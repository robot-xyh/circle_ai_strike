#include "strike_controller_adapter.hpp"

#include "circle/types/time.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include "circle/debug_common/strike_param_tune.hpp"
#endif

namespace circle::bf::flight {

namespace {

bool bfShouldCommandAlgorithm(const circle::strike::StrikeOutputs& outputs) {
  switch (outputs.state) {
    case circle::strike::StrikeState::Tracking:
    case circle::strike::StrikeState::CommitHold:
    case circle::strike::StrikeState::FaFallback:
      return outputs.has_valid_target;
    case circle::strike::StrikeState::WaitingTarget:
    case circle::strike::StrikeState::ForceLevel:
    case circle::strike::StrikeState::Complete:
      return false;
  }
  return false;
}

const char* strikeStateName(circle::strike::StrikeState state) {
  switch (state) {
    case circle::strike::StrikeState::WaitingTarget:
      return "WaitingTarget";
    case circle::strike::StrikeState::Tracking:
      return "Tracking";
    case circle::strike::StrikeState::ForceLevel:
      return "ForceLevel";
    case circle::strike::StrikeState::CommitHold:
      return "CommitHold";
    case circle::strike::StrikeState::FaFallback:
      return "FaFallback";
    case circle::strike::StrikeState::Complete:
      return "Complete";
  }
  return "Unknown";
}

}  // namespace

StrikeControllerAdapter::StrikeControllerAdapter(circle::strike::StrikeParams params)
    : params_(params), controller_(params) {}

void StrikeControllerAdapter::onEngageRisingEdge(
    const circle::bf::runtime::BfControlContext& /*ctx*/) {
  controller_.reset();
}

circle::bf::runtime::BfControlResult StrikeControllerAdapter::update(
    const circle::bf::runtime::BfControlContext& ctx) {
  circle::strike::StrikeInputs inputs;
  inputs.now_ns = ctx.now_ns;
  inputs.detection = ctx.detection;
  inputs.vehicle = ctx.vehicle;
  inputs.mode_active = ctx.mode_active;

  last_outputs_ = controller_.tick(inputs);

  circle::bf::runtime::BfControlResult result;
  result.rates = last_outputs_.rates;
  result.safety = last_outputs_.safety;
  result.has_target = last_outputs_.has_valid_target;
  result.image_ex = last_outputs_.image_ex;
  result.image_ey = last_outputs_.image_ey;
  result.state_code = static_cast<int>(last_outputs_.state);
  if (bfShouldCommandAlgorithm(last_outputs_)) {
    result.command = circle::bf::runtime::BfControlResult::Command::Algorithm;
  } else if (last_outputs_.state == circle::strike::StrikeState::ForceLevel) {
    result.command = circle::bf::runtime::BfControlResult::Command::LevelOnly;
  } else {
    result.command = circle::bf::runtime::BfControlResult::Command::None;
  }
  return result;
}

bool StrikeControllerAdapter::applyParamUpdateJson(const std::string& json) {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  if (circle::debug_common::applyStrikeParamUpdate(params_, json)) {
    controller_.setParams(params_);
    return true;
  }
  return false;
#else
  (void)json;
  return false;
#endif
}

std::string StrikeControllerAdapter::paramsSnapshotJson() const {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  return circle::debug_common::strikeCoreParamsJson(params_);
#else
  return R"({"ok":false,"error":"yaml support not built"})";
#endif
}

void StrikeControllerAdapter::fillTelemetry(
    circle::ipc::StrikeTelemetrySample& sample,
    const circle::bf::runtime::BfControlResult& /*result*/) const {
  const auto& outputs = last_outputs_;
  sample.ex_dot_filt = outputs.telemetry.ex_dot_filt;
  sample.ey_dot_filt = outputs.telemetry.ey_dot_filt;
  sample.e_rho = outputs.telemetry.e_rho;
  sample.e_rho_dot_filt = outputs.telemetry.e_rho_dot_filt;
  sample.rho_scale = outputs.telemetry.rho_scale;
  sample.roll_softcap_factor = outputs.telemetry.roll_softcap_factor;
  sample.pitch_softcap_factor = outputs.telemetry.pitch_softcap_factor;
  sample.roll_hard_headroom_rad = outputs.telemetry.roll_hard_headroom_rad;
  sample.pitch_hard_headroom_rad = outputs.telemetry.pitch_hard_headroom_rad;
  sample.final_approach_active = outputs.telemetry.final_approach_active ? 1 : 0;
  sample.aim_comp_x_px = outputs.telemetry.aim_comp_x_px;
  sample.aim_comp_y_px = outputs.telemetry.aim_comp_y_px;
  sample.tracking_thrust_scalar_smooth =
      outputs.telemetry.tracking_thrust_scalar_smooth;
  sample.tracking_thrust_scalar_target =
      outputs.telemetry.tracking_thrust_scalar_target;
  sample.deadband_eff_half_w_px = outputs.telemetry.deadband_eff_half_w_px;
  sample.deadband_eff_half_h_px = outputs.telemetry.deadband_eff_half_h_px;
  sample.fa_kp_scale = outputs.telemetry.fa_kp_scale;
  sample.fa_kd_scale = outputs.telemetry.fa_kd_scale;
  sample.bbox_area_ratio = outputs.telemetry.bbox_area_ratio;
  sample.detection_score = outputs.telemetry.detection_score;
  sample.tracker_fallback_active = outputs.telemetry.tracker_fallback_active ? 1 : 0;
  sample.strike_confident = outputs.telemetry.strike_confident ? 1 : 0;
  sample.preclimb_xy_gate_active =
      outputs.telemetry.preclimb_xy_gate_active ? 1 : 0;
  sample.speed_governor_blend = outputs.telemetry.speed_governor_blend;
  sample.speed_governor_scale = outputs.telemetry.speed_governor_scale;
  sample.fa_thrust_taper_scale = outputs.telemetry.fa_thrust_taper_scale;
}

void StrikeControllerAdapter::fillOverlay(
    circle::debug::PreviewOverlayContext& overlay,
    const circle::bf::runtime::BfControlResult& /*result*/) const {
  overlay.deadband_half_w_px = params_.x_deadband_px;
  overlay.deadband_half_h_px = params_.y_deadband_px;
  overlay.aim_offset_x_px = params_.aim_offset_x_px;
  overlay.aim_offset_y_px = params_.aim_offset_y_px;
}

const char* StrikeControllerAdapter::stateName(
    const circle::bf::runtime::BfControlResult& result) const {
  return strikeStateName(static_cast<circle::strike::StrikeState>(result.state_code));
}

}  // namespace circle::bf::flight
