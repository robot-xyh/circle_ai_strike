#pragma once

#include <limits>
#include <optional>

#include "circle/fc_iface/rate_sink.hpp"
#include "circle/fc_iface/vehicle_state_source.hpp"
#include "circle/fc_iface/log_sink.hpp"
#include "circle/strike/strike_params.hpp"
#include "circle/strike/modules/rate_shaper.hpp"
#include "circle/strike/modules/yaw_controller.hpp"
#include "circle/strike/modules/edge_protection.hpp"
#include "circle/strike/modules/final_approach_gate.hpp"
#include "circle/strike/modules/speed_governor.hpp"
#include "circle/strike/modules/preclimb_module.hpp"
#include "circle/strike/modules/thrust_manager.hpp"
#include "circle/strike/modules/commit_module.hpp"
#include "circle/strike/modules/tilt_guard.hpp"
#include "circle/strike/modules/directional_dive.hpp"
#include "circle/strike/modules/terminal_predictor.hpp"
#include "circle/strike/modules/visual_png_guidance.hpp"

namespace circle::strike {

class StrikeController {
 public:
  explicit StrikeController(StrikeParams params = {});

  void setParams(const StrikeParams& params);
  [[nodiscard]] const StrikeParams& params() const { return params_; }

  void reset();
  StrikeOutputs tick(const StrikeInputs& inputs);

  void bindRateSink(circle::fc_iface::IRateSink* sink) { rate_sink_ = sink; }
  void bindStateSource(circle::fc_iface::IVehicleStateSource* source) {
    state_source_ = source;
  }
  void bindLogSink(circle::fc_iface::ILogSink* sink) { log_sink_ = sink; }

 private:
  float computeWaitingThrustZ(bool entering_waiting,
                              const circle::types::FcState& vehicle,
                              float& out_ref_ned, float& out_err_ned,
                              float& out_corr);

  enum class ThrustRampPrevPublish {
    Waiting,
    ForceLevel,
    FaFallback,
    TrackingPd,
    CommitHold,
  };

  StrikeParams params_;
  StrikeState state_{StrikeState::WaitingTarget};
  DelayedPixelKalman dkf_;
  circle::types::TimestampNs last_tick_ns_{0};

  float prev_ex_{0.0F};
  float prev_ey_{0.0F};
  float ex_dot_filt_{0.0F};
  float ey_dot_filt_{0.0F};
  float e_rho_dot_filt_{0.0F};
  float last_image_ex_{0.0F};
  float last_image_ey_{0.0F};
  circle::types::TimestampNs last_detection_ns_{0};

  std::optional<float> last_ex_;
  std::optional<float> last_ey_;
  std::optional<float> last_e_rho_;
  float last_rho_scale_{1.0F};
  circle::types::TimestampNs last_pixel_dot_time_ns_{0};
  
  DelayedPixelKalman::Estimate tracker_fallback_est_{};
  bool tracker_fallback_active_{false};
  DelayedPixelKalman::Estimate confidence_est_{};
  bool confidence_est_valid_{false};
  bool strike_confident_{true};

  ThrustRampPrevPublish thrust_ramp_prev_publish_{ThrustRampPrevPublish::Waiting};

  bool final_approach_active_{false};
  std::optional<circle::types::TimestampNs> fa_last_active_time_ns_;

  std::optional<circle::types::TimestampNs> fa_fallback_start_time_ns_;

  std::optional<circle::types::TimestampNs> force_level_enter_time_ns_;
  std::optional<circle::types::TimestampNs> force_level_exit_candidate_time_ns_;

  std::optional<float> waiting_yaw_ref_;
  std::optional<float> waiting_altitude_ref_ned_;

  bool target_seen_this_activation_{false};
  bool target_loss_completed_{false};
  std::optional<circle::types::TimestampNs> last_target_seen_time_ns_;
  std::optional<circle::types::TimestampNs> first_fresh_target_time_ns_;

  // Modules
  RateShaper rate_shaper_;
  YawController yaw_controller_;
  EdgeProtection edge_protection_;
  FinalApproachGate final_approach_gate_;
  SpeedGovernor speed_governor_;
  PreclimbModule preclimb_module_;
  ThrustManager thrust_manager_;
  CommitModule commit_module_;
  TiltGuard tilt_guard_;
  DirectionalDive directional_dive_;
  TerminalPredictor terminal_predictor_;
  VisualPngGuidance visual_png_guidance_;
  RhoRateWindowEstimator rho_rate_window_;
  FAGateState fa_gate_state_;
  PreclimbState preclimb_state_;
  ThrustManagerState thrust_state_;
  CommitState commit_state_;
  DirectionalDiveState dive_state_;

  circle::fc_iface::IRateSink* rate_sink_{nullptr};
  circle::fc_iface::IVehicleStateSource* state_source_{nullptr};
  circle::fc_iface::ILogSink* log_sink_{nullptr};
};

}  // namespace circle::strike
