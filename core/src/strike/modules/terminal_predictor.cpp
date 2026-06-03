#include "circle/strike/modules/terminal_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike {

TerminalPredictorOutput TerminalPredictor::compute(
    const TerminalPredictorParams& params,
    const TerminalPredictorInput& input) const {
  TerminalPredictorOutput out;
  out.predicted_ex = input.ex;
  out.predicted_ey = input.ey;
  out.closing_rate = std::max(0.0F, -input.e_rho_dot);

  if (!params.enable || !input.final_approach_active ||
      !input.fresh_detection || params.lead_s <= 0.0F ||
      params.blend <= 0.0F || input.fx <= 1.0e-6F || input.fy <= 1.0e-6F) {
    return out;
  }

  if (params.min_area_ratio > 0.0F &&
      input.bbox_area_ratio < params.min_area_ratio) {
    return out;
  }

  if (params.min_closing_rate > 0.0F &&
      out.closing_rate < params.min_closing_rate) {
    return out;
  }

  const float max_lead_x_norm =
      params.max_lead_px > 0.0F ? params.max_lead_px / input.fx : 0.0F;
  const float max_lead_y_norm =
      params.max_lead_px > 0.0F ? params.max_lead_px / input.fy : 0.0F;
  const float raw_lead_x_norm = input.ex_dot * params.lead_s;
  const float raw_lead_y_norm = input.ey_dot * params.lead_s;
  const float lead_x_norm = params.max_lead_px > 0.0F
                                ? std::clamp(raw_lead_x_norm,
                                             -max_lead_x_norm,
                                             max_lead_x_norm)
                                : raw_lead_x_norm;
  const float lead_y_norm = params.max_lead_px > 0.0F
                                ? std::clamp(raw_lead_y_norm,
                                             -max_lead_y_norm,
                                             max_lead_y_norm)
                                : raw_lead_y_norm;
  const float blend = std::clamp(params.blend, 0.0F, 1.0F);

  out.active = true;
  out.predicted_ex = input.ex + lead_x_norm * blend;
  out.predicted_ey = input.ey + lead_y_norm * blend;
  out.lead_x_px = lead_x_norm * input.fx * blend;
  out.lead_y_px = lead_y_norm * input.fy * blend;
  return out;
}

}  // namespace circle::strike
