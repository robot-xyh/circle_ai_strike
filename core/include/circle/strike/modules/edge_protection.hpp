#pragma once

#include <cstdint>

#include "circle/strike/strike_params.hpp"
#include "circle/types/detection.hpp"
#include "circle/types/fc_state.hpp"

namespace circle::strike {

struct EdgeProtectionOutput {
  bool active{false};
  float roll_boost{0.0F};
  float pitch_boost{0.0F};
  float thrust_scale{1.0F};
  float taper_score{0.0F};
  float bottom_pitch_guard_blend{0.0F};
  float bottom_pitch_guard_rate{0.0F};
  bool bottom_pitch_guard_active{false};
};

class EdgeProtection {
 public:
  EdgeProtectionOutput compute(
      bool final_approach_active, bool fresh_detection,
      const circle::types::FrameDetection& detection,
      const circle::types::FcState& vehicle,
      float image_ex, float image_ey,
      float ex_dot_filt, float ey_dot_filt,
      float lateral_output_sign, float longitudinal_output_sign,
      const FAEdgeProtectParams& edge_params,
      const FABottomPitchGuardParams& bottom_guard_params,
      float commit_min_margin_x_px, float commit_min_margin_y_px) const;
};

}  // namespace circle::strike
