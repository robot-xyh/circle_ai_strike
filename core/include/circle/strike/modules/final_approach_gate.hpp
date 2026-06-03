#pragma once

#include <cstdint>
#include <optional>

#include "circle/strike/strike_params.hpp"
#include "circle/types/detection.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/time.hpp"

namespace circle::strike {

struct FAGateState {
  bool active{false};
  bool hold_active{false};
  double hold_age_s{0.0};
  std::optional<circle::types::TimestampNs> last_gate_time_ns;
  std::optional<circle::types::TimestampNs> stable_since_ns;
};

struct FAGateOutput {
  bool active{false};
  float bbox_area_ratio{0.0F};
  float e_rho{0.0F};
  float rho_scale{1.0F};
  float gate_tilt_rad{0.0F};
  float gate_error_x_px{0.0F};
  float gate_error_y_px{0.0F};
  float gate_rate_x_px_s{0.0F};
  float gate_rate_y_px_s{0.0F};
  bool area_quality_ok{true};
  bool preclimb_released{true};
  bool speed_governor_fa_speed_ok{true};
  const char* gate_reason{"none"};
};

class FinalApproachGate {
 public:
  void reset();

  FAGateOutput compute(
      FAGateState& state,
      const FAGateParams& gate_params,
      const SpeedGovernorParams& speed_governor_params,
      const PreclimbParams& preclimb_params,
      const EvaluationParams& evaluation_params,
      const RhoScaleParams& rho_scale_params,
      bool preclimb_xy_gate_released,
      bool fresh_detection, bool strike_confident,
      const circle::types::FrameDetection& detection,
      const circle::types::FcState& vehicle,
      float image_ex, float image_ey,
      float ex_dot_filt, float ey_dot_filt,
      circle::types::TimestampNs now_ns) const;
};

}  // namespace circle::strike
