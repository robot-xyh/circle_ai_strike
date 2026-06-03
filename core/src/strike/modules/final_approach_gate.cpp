#include "circle/strike/modules/final_approach_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

void FinalApproachGate::reset() {
}

FAGateOutput FinalApproachGate::compute(
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
    circle::types::TimestampNs now_ns) const {
  FAGateOutput out;

  const float bbox_area = detection.detection.width * detection.detection.height;
  const float frame_area = static_cast<float>(
      detection.image_width * detection.image_height);
  out.bbox_area_ratio = (frame_area > 1.0F) ? (bbox_area / frame_area) : 0.0F;

  if (rho_scale_params.desired_bbox_area_px > 0.0) {
    out.e_rho = static_cast<float>(
        0.5 * std::log(rho_scale_params.desired_bbox_area_px / bbox_area));
  }

  const float e_rho_near = -0.5F * std::log(
      std::max(1.0001F, rho_scale_params.near_ratio));
  const float e_rho_far = -0.5F * std::log(
      std::clamp(rho_scale_params.far_ratio, 1.0e-4F, 0.9999F));
  out.rho_scale = (rho_scale_params.desired_bbox_area_px > 0.0)
      ? lerp(out.e_rho, e_rho_near, e_rho_far,
             rho_scale_params.scale_near, rho_scale_params.scale_far)
      : 1.0F;

  const bool was_active = state.active;
  const bool fa_gate_configured =
      gate_params.boost_enable && frame_area > 1.0F;
  out.gate_tilt_rad = std::hypot(vehicle.roll_rad, vehicle.pitch_rad);
  out.gate_error_x_px = std::abs(image_ex * detection.intrinsics.fx);
  out.gate_error_y_px = std::abs(image_ey * detection.intrinsics.fy);
  out.gate_rate_x_px_s = std::abs(ex_dot_filt * detection.intrinsics.fx);
  out.gate_rate_y_px_s = std::abs(ey_dot_filt * detection.intrinsics.fy);

  out.speed_governor_fa_speed_ok =
      !speed_governor_params.fa_gate_enable ||
      !vehicle.velocity_xy_valid ||
      speed_governor_params.fa_max_vxy_m_s <= 0.0F ||
      vehicle.velocity_xy_m_s <= speed_governor_params.fa_max_vxy_m_s;

  out.area_quality_ok =
      !gate_params.area_quality_gate_enable ||
      (fresh_detection &&
       out.gate_error_x_px <= gate_params.area_quality_error_x_px &&
       out.gate_error_y_px <= gate_params.area_quality_error_y_px &&
       out.gate_rate_x_px_s <= gate_params.area_quality_rate_x_px_s &&
       out.gate_rate_y_px_s <= gate_params.area_quality_rate_y_px_s &&
       out.gate_tilt_rad <= gate_params.area_quality_max_tilt_rad);

  out.preclimb_released =
      !preclimb_params.xy_gate_enable || preclimb_xy_gate_released;

  const bool altitude_gate_ok =
      !gate_params.altitude_gate_enable ||
      (evaluation_params.target_altitude_enable &&
       vehicle.valid && vehicle.position_z_valid &&
       std::fabs(evaluation_params.target_altitude_m + vehicle.position_ned_z) <=
           gate_params.altitude_gate_max_gap_m);

  const bool fa_area_entry_candidate =
      fa_gate_configured &&
      out.preclimb_released &&
      altitude_gate_ok &&
      out.speed_governor_fa_speed_ok &&
      out.area_quality_ok &&
      out.bbox_area_ratio >= gate_params.area_ratio_enter;

  const bool fa_area_hold_candidate =
      fa_gate_configured &&
      out.bbox_area_ratio >= gate_params.area_ratio_exit &&
      (was_active ||
       (altitude_gate_ok && out.speed_governor_fa_speed_ok && out.area_quality_ok));

  const bool fa_tilt_gate_base_candidate =
      fa_gate_configured &&
      gate_params.tilt_gate_enable &&
      fresh_detection &&
      out.preclimb_released &&
      altitude_gate_ok &&
      out.speed_governor_fa_speed_ok &&
      out.bbox_area_ratio >= gate_params.tilt_gate_min_area_ratio;

  const bool fa_tilt_gate_entry_candidate =
      fa_tilt_gate_base_candidate &&
      out.gate_tilt_rad >= gate_params.tilt_gate_start_rad &&
      out.gate_error_x_px <= gate_params.tilt_gate_error_x_px &&
      out.gate_error_y_px <= gate_params.tilt_gate_error_y_px &&
      out.gate_rate_x_px_s <= gate_params.tilt_gate_rate_x_px_s &&
      out.gate_rate_y_px_s <= gate_params.tilt_gate_rate_y_px_s;

  const bool fa_stable_gate_base_candidate =
      fa_gate_configured &&
      gate_params.stable_gate_enable &&
      fresh_detection &&
      out.preclimb_released &&
      altitude_gate_ok &&
      out.speed_governor_fa_speed_ok &&
      out.bbox_area_ratio >= gate_params.stable_gate_min_area_ratio &&
      (gate_params.stable_gate_max_area_ratio <= 0.0F ||
       out.bbox_area_ratio <= gate_params.stable_gate_max_area_ratio) &&
      out.gate_tilt_rad <= gate_params.stable_gate_max_tilt_rad &&
      out.gate_error_x_px <= gate_params.stable_gate_error_x_px &&
      out.gate_error_y_px <= gate_params.stable_gate_error_y_px &&
      out.gate_rate_x_px_s <= gate_params.stable_gate_rate_x_px_s &&
      out.gate_rate_y_px_s <= gate_params.stable_gate_rate_y_px_s;

  if (fa_stable_gate_base_candidate && strike_confident) {
    if (!state.stable_since_ns.has_value()) {
      state.stable_since_ns = now_ns;
    }
  } else {
    state.stable_since_ns.reset();
  }

  const double fa_stable_hold_s =
      state.stable_since_ns.has_value()
          ? circle::types::secondsBetween(*state.stable_since_ns, now_ns)
          : 0.0;

  const bool fa_stable_gate_entry_candidate =
      fa_stable_gate_base_candidate &&
      strike_confident &&
      fa_stable_hold_s >= static_cast<double>(gate_params.stable_gate_hold_s);

  const bool fa_tilt_gate_hold_candidate =
      fa_tilt_gate_base_candidate && strike_confident;
  const bool fa_stable_gate_hold_candidate =
      fa_stable_gate_base_candidate && strike_confident;

  bool fa_gate_active = false;
  state.hold_active = false;
  state.hold_age_s = std::numeric_limits<double>::quiet_NaN();

  if (fa_gate_configured) {
    if (was_active) {
      const bool direct_hold_candidate =
          fa_area_hold_candidate ||
          fa_tilt_gate_hold_candidate ||
          fa_stable_gate_hold_candidate;
      if (direct_hold_candidate) {
        state.last_gate_time_ns = now_ns;
      }
      if (!direct_hold_candidate &&
          gate_params.hold_s > 0.0F &&
          state.last_gate_time_ns.has_value()) {
        state.hold_age_s = circle::types::secondsBetween(
            *state.last_gate_time_ns, now_ns);
        state.hold_active =
            state.hold_age_s >= 0.0 &&
            state.hold_age_s <= static_cast<double>(gate_params.hold_s);
      }
      fa_gate_active = direct_hold_candidate || state.hold_active;
      
      if (fa_area_hold_candidate) {
        out.gate_reason = "area_hold";
      } else if (fa_tilt_gate_hold_candidate) {
        out.gate_reason = "tilt_hold";
      } else if (fa_stable_gate_hold_candidate) {
        out.gate_reason = "stable_hold";
      } else if (state.hold_active) {
        out.gate_reason = "time_hold";
      } else {
        out.gate_reason = "hold_expired";
      }
    } else {
      const bool fa_entry_candidate =
          fa_area_entry_candidate ||
          fa_tilt_gate_entry_candidate ||
          fa_stable_gate_entry_candidate;
      fa_gate_active = fa_entry_candidate && strike_confident;
      if (fa_gate_active) {
        state.last_gate_time_ns = now_ns;
        if (fa_area_entry_candidate) {
          out.gate_reason = "area";
        } else if (fa_tilt_gate_entry_candidate) {
          out.gate_reason = "tilt_gate";
        } else {
          out.gate_reason = "stable_gate";
        }
      } else if (fa_area_entry_candidate) {
        out.gate_reason = "area_confidence_block";
      } else if (fa_tilt_gate_entry_candidate) {
        out.gate_reason = "tilt_gate_confidence_block";
      } else if (fa_stable_gate_base_candidate) {
        out.gate_reason = "stable_gate_hold";
      } else {
        out.gate_reason = "no_candidate";
      }
    }
  }

  state.active = fa_gate_active;
  out.active = fa_gate_active;

  if (!out.active) {
    state.last_gate_time_ns.reset();
  }

  return out;
}

}  // namespace circle::strike
