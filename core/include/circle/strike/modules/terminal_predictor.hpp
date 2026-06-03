#pragma once

namespace circle::strike {

struct TerminalPredictorParams {
  bool enable{false};
  float min_area_ratio{0.0F};
  float min_closing_rate{0.0F};
  float lead_s{0.0F};
  float max_lead_px{0.0F};
  float blend{0.0F};
};

struct TerminalPredictorInput {
  bool final_approach_active{false};
  bool fresh_detection{false};
  float bbox_area_ratio{0.0F};
  float ex{0.0F};
  float ey{0.0F};
  float ex_dot{0.0F};
  float ey_dot{0.0F};
  float e_rho_dot{0.0F};
  float fx{0.0F};
  float fy{0.0F};
};

struct TerminalPredictorOutput {
  bool active{false};
  float predicted_ex{0.0F};
  float predicted_ey{0.0F};
  float lead_x_px{0.0F};
  float lead_y_px{0.0F};
  float closing_rate{0.0F};
};

class TerminalPredictor {
 public:
  [[nodiscard]] TerminalPredictorOutput compute(
      const TerminalPredictorParams& params,
      const TerminalPredictorInput& input) const;
};

}  // namespace circle::strike
