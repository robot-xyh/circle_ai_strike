#include "circle/strike_png/strike_png_controller.hpp"

#include <algorithm>
#include <cmath>

namespace circle::strike_png {

namespace {

constexpr float kNsToS = 1.0e-9F;
constexpr float kDegToRad = 0.017453292519943295F;

float lpfAlpha(float tau_s, float dt_s) {
  if (tau_s <= 0.0F) {
    return 1.0F;
  }
  return std::clamp(dt_s / (tau_s + dt_s), 0.0F, 1.0F);
}

}  // namespace

void StrikePngController::reset() {
  last_valid_ = false;
  last_stamp_ns_ = 0;
  last_measurement_ns_ = 0;
  last_ex_ = 0.0F;
  last_ey_ = 0.0F;
  ex_dot_filt_ = 0.0F;
  ey_dot_filt_ = 0.0F;
  dkf_.reset();
  tilt_envelope_.reset();
}

StrikePngOutput StrikePngController::tick(const StrikePngParams& params,
                                          const StrikePngInput& input) {
  StrikePngOutput out;
  out.has_target = params.enable && input.detection_valid;
  out.thrust_z = 0.0F;
  if (!out.has_target || input.now_ns == 0) {
    reset();
    return out;
  }

  const float ctrl_dt_s =
      (last_valid_ && input.now_ns > last_stamp_ns_)
          ? static_cast<float>(input.now_ns - last_stamp_ns_) * kNsToS
          : 0.0F;

  const uint64_t measurement_ns =
      input.measurement_ns > 0 ? input.measurement_ns : input.now_ns;
  const float measurement_age_s =
      input.now_ns > measurement_ns
          ? static_cast<float>(input.now_ns - measurement_ns) * kNsToS
          : 0.0F;
  const bool first_measurement = !last_valid_;
  const bool new_measurement =
      last_valid_ && measurement_ns > last_measurement_ns_;
  const bool dkf_requested = params.dkf_enable && params.dkf.enable;
  circle::strike::DelayedPixelKalman::Estimate dkf_est;
  bool dkf_valid = false;

  if (dkf_requested && input.fx > 1.0F && input.fy > 1.0F) {
    if (first_measurement || new_measurement) {
      circle::strike::DelayedPixelKalman::Measurement meas;
      meas.image_stamp_ns = measurement_ns;
      meas.receive_stamp_ns = input.now_ns;
      meas.ex = input.ex;
      meas.ey = input.ey;
      meas.bbox_area_px = input.bbox_area_px;
      meas.score = input.detection_score;
      circle::types::CameraIntrinsics intr;
      intr.fx = input.fx;
      intr.fy = input.fy;
      dkf_.addMeasurement(meas, intr, params.dkf);
    }
    dkf_est = dkf_.predict(input.now_ns, params.dkf);
    dkf_valid =
        dkf_est.valid && dkf_est.cov_trace <= std::max(1.0e-6F, params.dkf.max_cov_trace);
  }

  if (dkf_valid) {
    ex_dot_filt_ = dkf_est.ex_dot;
    ey_dot_filt_ = dkf_est.ey_dot;
  } else if (new_measurement) {
    const float dt_s = std::max(
        1.0e-3F,
        static_cast<float>(measurement_ns - last_measurement_ns_) * kNsToS);
    const float ex_dot_raw = (input.ex - last_ex_) / dt_s;
    const float ey_dot_raw = (input.ey - last_ey_) / dt_s;
    const float a = lpfAlpha(params.pixel_dot_lpf_tau_s, dt_s);
    ex_dot_filt_ += a * (ex_dot_raw - ex_dot_filt_);
    ey_dot_filt_ += a * (ey_dot_raw - ey_dot_filt_);
  } else if (first_measurement) {
    ex_dot_filt_ = 0.0F;
    ey_dot_filt_ = 0.0F;
  } else if (input.now_ns > last_stamp_ns_) {
    const float dt_s = std::max(
        1.0e-3F, static_cast<float>(input.now_ns - last_stamp_ns_) * kNsToS);
    const float tau_s = std::max(0.0F, params.los_rate_hold_tau_s);
    if (tau_s > 0.0F) {
      const float decay = std::exp(-dt_s / tau_s);
      ex_dot_filt_ *= decay;
      ey_dot_filt_ *= decay;
    } else {
      ex_dot_filt_ = 0.0F;
      ey_dot_filt_ = 0.0F;
    }
  }
  last_valid_ = true;
  last_stamp_ns_ = input.now_ns;
  if (measurement_ns >= last_measurement_ns_) {
    last_measurement_ns_ = measurement_ns;
    last_ex_ = input.ex;
    last_ey_ = input.ey;
  }

  float control_ex = dkf_valid ? dkf_est.ex : input.ex;
  float control_ey = dkf_valid ? dkf_est.ey : input.ey;
  const float prediction_max_age_s =
      std::max(0.0F, params.visual_prediction_max_age_s);
  const float prediction_horizon_s =
      params.visual_prediction_enable && !dkf_valid
          ? std::clamp(measurement_age_s, 0.0F, prediction_max_age_s)
          : 0.0F;
  if (prediction_horizon_s > 0.0F) {
    const float max_offset =
        std::max(0.0F, params.visual_prediction_max_offset_norm);
    const float pred_dx =
        std::clamp(ex_dot_filt_ * prediction_horizon_s,
                   -max_offset,
                   max_offset);
    const float pred_dy =
        std::clamp(ey_dot_filt_ * prediction_horizon_s,
                   -max_offset,
                   max_offset);
    control_ex += pred_dx;
    control_ey += pred_dy;
    out.visual_prediction_active =
        std::abs(pred_dx) > 1.0e-6F || std::abs(pred_dy) > 1.0e-6F;
  }
  out.measurement_age_s = measurement_age_s;
  out.control_ex = control_ex;
  out.control_ey = control_ey;
  out.derotate_rate_valid = input.derotate_rate_valid;

  const auto png = guidance_.compute(
      VisualPngGuidanceParams{
          params.nav_ratio_x,
          params.nav_ratio_y,
          params.derotate_body_rates,
          params.derotate_pitch_to_x_gain,
          params.derotate_roll_to_y_gain,
          params.residual_rate_limit_rad_s,
          params.closure_base_scale,
          params.closure_area_gain,
          params.max_feedforward_rad_s,
          params.fov_trim_kp_rate,
          params.fov_trim_fade_area_ratio_start,
          params.fov_trim_fade_area_ratio_full,
          params.vertical_aim_ey,
          params.terminal_tilt_aim_area_ratio_start,
          params.terminal_tilt_aim_area_ratio_full,
          params.terminal_tilt_aim_roll_gain,
          params.terminal_tilt_aim_pitch_gain,
          params.terminal_tilt_aim_max_offset_norm,
          params.edge_guard_enable,
          params.edge_guard_start_norm,
          params.edge_guard_full_norm,
          params.edge_guard_kp_rate,
          params.edge_guard_min_rate_rad_s,
          params.edge_guard_max_rate_rad_s,
          params.edge_guard_pitch_scale,
          params.pursuit_fallback_enable,
          params.pursuit_fallback_kp_rate,
          params.pursuit_fallback_start_norm,
          params.pursuit_fallback_full_norm,
          params.pursuit_fallback_min_rate_rad_s,
          params.pursuit_fallback_max_rate_rad_s,
          params.pursuit_fallback_png_weak_rate_rad_s,
          params.terminal_stale_lateral_trim_enable,
          params.terminal_stale_lateral_trim_area_ratio_start,
          params.terminal_stale_lateral_trim_area_ratio_full,
          params.terminal_stale_lateral_trim_stale_s_start,
          params.terminal_stale_lateral_trim_stale_s_full,
          params.terminal_stale_lateral_trim_kp_rate,
          params.terminal_stale_lateral_trim_max_rate_rad_s,
          params.terminal_intercept_enable,
          params.terminal_intercept_area_ratio_start,
          params.terminal_intercept_area_ratio_full,
          params.terminal_intercept_lead_s,
          params.terminal_intercept_kp_rate,
          params.terminal_intercept_max_rate_rad_s,
          params.terminal_crossing_enable,
          params.terminal_crossing_area_ratio_start,
          params.terminal_crossing_area_ratio_full,
          params.terminal_crossing_rate_start_norm_s,
          params.terminal_crossing_rate_full_norm_s,
          params.terminal_crossing_kd_rate,
          params.terminal_crossing_max_rate_rad_s,
          params.terminal_forward_speed_guard_enable,
          params.terminal_forward_speed_guard_area_ratio_start,
          params.terminal_forward_speed_guard_area_ratio_full,
          params.terminal_forward_speed_guard_start_m_s,
          params.terminal_forward_speed_guard_full_m_s,
          params.terminal_forward_speed_guard_min_positive_pitch_scale,
          params.lateral_output_sign,
          params.longitudinal_output_sign},
      VisualPngGuidanceInput{
          control_ex,
          control_ey,
          ex_dot_filt_,
          ey_dot_filt_,
          input.bbox_area_ratio,
          input.roll_rate_rad_s,
          input.pitch_rate_rad_s,
          input.derotate_rate_valid,
          input.attitude_valid,
          input.vehicle_roll_rad,
          input.vehicle_pitch_rad,
          params.max_roll_rate_rad_s,
          params.max_pitch_rate_rad_s,
          measurement_age_s,
          input.ownship_forward_speed_valid,
          input.ownship_forward_speed_m_s});

  out.png_active = png.active;
  out.roll_rate_rad_s = png.roll_rate_rad_s;
  out.pitch_rate_rad_s = png.pitch_rate_rad_s;
  out.ex_dot_filt = ex_dot_filt_;
  out.ey_dot_filt = ey_dot_filt_;
  out.png_closure_scale = png.closure_scale;
  out.png_ex_dot_inertial = png.ex_dot_inertial;
  out.png_ey_dot_inertial = png.ey_dot_inertial;
  out.measurement_age_s = measurement_age_s;
  out.roll_png_ff_rad_s = png.roll_png_ff_rad_s;
  out.pitch_png_ff_rad_s = png.pitch_png_ff_rad_s;
  out.roll_fov_trim_rad_s = png.roll_fov_trim_rad_s;
  out.pitch_fov_trim_rad_s = png.pitch_fov_trim_rad_s;
  out.roll_edge_guard_rad_s = png.roll_edge_guard_rad_s;
  out.pitch_edge_guard_rad_s = png.pitch_edge_guard_rad_s;
  out.roll_pursuit_fallback_rad_s = png.roll_pursuit_fallback_rad_s;
  out.pitch_pursuit_fallback_rad_s = png.pitch_pursuit_fallback_rad_s;
  out.roll_terminal_stale_trim_rad_s = png.roll_terminal_stale_trim_rad_s;
  out.terminal_intercept_active = png.terminal_intercept_active;
  out.terminal_intercept_lead_s = png.terminal_intercept_lead_s;
  out.terminal_future_ex = png.terminal_future_ex;
  out.terminal_future_ey = png.terminal_future_ey;
  out.roll_terminal_intercept_rad_s = png.roll_terminal_intercept_rad_s;
  out.pitch_terminal_intercept_rad_s = png.pitch_terminal_intercept_rad_s;
  out.terminal_aim_ex = png.terminal_aim_ex;
  out.terminal_aim_ey = png.terminal_aim_ey;
  out.terminal_crossing_active = png.terminal_crossing_active;
  out.terminal_crossing_weight = png.terminal_crossing_weight;
  out.roll_terminal_crossing_rad_s = png.roll_terminal_crossing_rad_s;
  out.pitch_terminal_crossing_rad_s = png.pitch_terminal_crossing_rad_s;
  out.terminal_forward_speed_guard_active =
      png.terminal_forward_speed_guard_active;
  out.terminal_forward_speed_guard_weight =
      png.terminal_forward_speed_guard_weight;
  out.terminal_forward_speed_guard_scale =
      png.terminal_forward_speed_guard_scale;

  if (params.tilt_cap.enable && input.attitude_valid) {
    const auto& tc = params.tilt_cap;
    circle::strike::TiltEnvelopeParams ep;
    ep.enable = true;
    ep.max_roll_angle_rad = tc.max_roll_angle_deg * kDegToRad;
    ep.max_pitch_angle_rad = tc.max_pitch_angle_deg * kDegToRad;
    ep.softcap_band_rad = std::max(0.0F, tc.softcap_band_deg) * kDegToRad;
    ep.hardcap_margin_rad = std::max(0.0F, tc.hardcap_margin_deg) * kDegToRad;
    ep.hardcap_level_kp = tc.hardcap_level_kp;
    ep.max_level_rate_rad_s =
        std::max(0.0F, tc.hardcap_max_level_rate_deg_s) * kDegToRad;
    ep.out_lpf_tau_s = std::max(0.0F, tc.out_lpf_tau_s);
    ep.out_max_jerk_rad_s2 = std::max(0.0F, tc.out_max_jerk_deg_s2) * kDegToRad;

    const auto env = tilt_envelope_.compute(
        ep, out.roll_rate_rad_s, out.pitch_rate_rad_s, input.vehicle_roll_rad,
        input.vehicle_pitch_rad, ctrl_dt_s);
    out.roll_rate_rad_s = env.roll_rate_rad_s;
    out.pitch_rate_rad_s = env.pitch_rate_rad_s;
    out.roll_tilt_softcap_factor = env.roll_softcap_factor;
    out.pitch_tilt_softcap_factor = env.pitch_softcap_factor;
    out.tilt_hardcap_active = env.hardcap_active;
  } else {
    tilt_envelope_.reset();
  }
  return out;
}

}  // namespace circle::strike_png
