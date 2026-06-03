#include "circle/strike/strike_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "circle/strike/math_utils.hpp"
#include "circle/vision/detection_filter.hpp"

namespace circle::strike {

namespace {

circle::types::TimestampNs effectiveDetectionStampNs(
    const circle::types::FrameDetection& detection,
    circle::types::TimestampNs now_ns) {
  if (detection.capture_ns == 0) {
    return detection.receive_ns;
  }
  if (detection.receive_ns == 0) {
    return detection.capture_ns;
  }

  const double capture_age_s =
      circle::types::secondsBetween(detection.capture_ns, now_ns);
  const double receive_age_s =
      circle::types::secondsBetween(detection.receive_ns, now_ns);
  return std::fabs(receive_age_s) < std::fabs(capture_age_s)
             ? detection.receive_ns
             : detection.capture_ns;
}

float approachDriveFovScale(const ApproachDriveParams& params,
                            float image_ey,
                            float fy) {
  if (!params.fov_gate_enable || fy <= 1.0e-6F) {
    return 1.0F;
  }
  const float high_error_px = std::max(0.0F, -image_ey * fy);
  const float span_px = std::max(
      1.0F, params.fov_gate_high_error_px -
                params.fov_gate_release_error_px);
  const float t = (high_error_px - params.fov_gate_release_error_px) /
                  span_px;
  const float suppress = smoothstep01(t);
  return 1.0F - suppress * (1.0F - params.fov_gate_min_scale);
}

}  // namespace

StrikeController::StrikeController(StrikeParams params) : params_(std::move(params)) {
  params_.filter.target_class_name = params_.filter.target_class_name.empty()
                                          ? "UAV"
                                          : params_.filter.target_class_name;
}

void StrikeController::setParams(const StrikeParams& params) { params_ = params; }

void StrikeController::reset() {
  state_ = StrikeState::WaitingTarget;
  dkf_.reset();
  last_tick_ns_ = 0;
  prev_ex_ = 0.0F;
  prev_ey_ = 0.0F;
  ex_dot_filt_ = 0.0F;
  ey_dot_filt_ = 0.0F;
  e_rho_dot_filt_ = 0.0F;
  rho_rate_window_.reset();
  last_image_ex_ = 0.0F;
  last_image_ey_ = 0.0F;
  last_detection_ns_ = 0;
  last_e_rho_.reset();
  last_rho_scale_ = 1.0F;
  // Reset modules
  rate_shaper_.reset();
  fa_gate_state_ = FAGateState{};
  preclimb_state_ = PreclimbState{};
  thrust_state_ = ThrustManagerState{};
  commit_state_ = CommitState{};
  dive_state_ = DirectionalDiveState{};
  thrust_ramp_prev_publish_ = ThrustRampPrevPublish::Waiting;
  final_approach_active_ = false;
  fa_last_active_time_ns_.reset();
  fa_fallback_start_time_ns_.reset();
  force_level_enter_time_ns_.reset();
  force_level_exit_candidate_time_ns_.reset();
  waiting_yaw_ref_.reset();
  waiting_altitude_ref_ned_.reset();
  target_seen_this_activation_ = false;
  target_loss_completed_ = false;
  last_target_seen_time_ns_.reset();
  first_fresh_target_time_ns_.reset();
  last_pixel_dot_time_ns_ = 0;
  last_ex_.reset();
  last_ey_.reset();
  
  if (rate_sink_) {
    rate_sink_->resetActivationLatch();
  }
}

float StrikeController::computeWaitingThrustZ(
    bool entering_waiting, const circle::types::FcState& vehicle,
    float& out_ref_ned, float& out_err_ned, float& out_corr) {
  out_ref_ned = std::numeric_limits<float>::quiet_NaN();
  out_err_ned = std::numeric_limits<float>::quiet_NaN();
  out_corr = std::numeric_limits<float>::quiet_NaN();
  float thrust_scalar = params_.thrust.hover_scalar;

  if (vehicle.valid && vehicle.position_z_valid) {
    const float z_now = vehicle.position_ned_z;
    if (entering_waiting || !waiting_altitude_ref_ned_.has_value()) {
      waiting_altitude_ref_ned_ = z_now;
    }
    if (waiting_altitude_ref_ned_.has_value()) {
      out_ref_ned = *waiting_altitude_ref_ned_;
      const float z_error_ned = z_now - *waiting_altitude_ref_ned_;
      out_err_ned = z_error_ned;
      const float vz_ned = vehicle.velocity_ned_z;
      const float altitude_correction = std::clamp(
          params_.waiting.altitude_kp * z_error_ned +
              params_.waiting.altitude_kd * vz_ned,
          -params_.waiting.altitude_max_correction,
          params_.waiting.altitude_max_correction);
      out_corr = altitude_correction;
      thrust_scalar = clampThrustScalar(
          params_.thrust.hover_scalar + altitude_correction,
          params_.thrust.scalar_min, params_.thrust.scalar_max);
    }
  } else {
    waiting_altitude_ref_ned_.reset();
  }

  const float tilt_denom =
      params_.thrust.enable_tilt_compensation
          ? std::max(std::cos(vehicle.pitch_rad) * std::cos(vehicle.roll_rad),
                     params_.thrust.tilt_cos_floor * params_.thrust.tilt_cos_floor)
          : 1.0F;
  return clampThrustScalar(thrust_scalar / tilt_denom, params_.thrust.scalar_min,
                           params_.thrust.scalar_max);
}

StrikeOutputs StrikeController::tick(const StrikeInputs& inputs) {
  StrikeOutputs out;
  out.safety.dry_run = params_.dry_run;
  out.safety.require_armed_to_command = params_.require_armed_to_command;
  out.safety.armed = inputs.vehicle.valid && inputs.vehicle.armed;
  out.state = state_;

  const float dt_s = (last_tick_ns_ > 0)
                          ? static_cast<float>(
                                circle::types::secondsBetween(last_tick_ns_, inputs.now_ns))
                          : 0.0F;
  const float safe_dt = std::max(dt_s, 1.0e-4F);
  last_tick_ns_ = inputs.now_ns;

  circle::types::FcState vehicle = inputs.vehicle;
  if (state_source_) {
    vehicle = state_source_->snapshot();
    out.safety.armed = vehicle.valid && vehicle.armed;
  }

  const float roll_hard_headroom_rad =
      (params_.tilt_cap.max_roll_angle_rad + params_.tilt_cap.hardcap_margin_rad) -
      std::fabs(vehicle.roll_rad);
  const float pitch_hard_headroom_rad =
      (params_.tilt_cap.max_pitch_angle_rad + params_.tilt_cap.hardcap_margin_rad) -
      std::fabs(vehicle.pitch_rad);

  const bool attitude_sane =
      vehicle.valid && std::fabs(vehicle.roll_rad) < 1.2F &&
      std::fabs(vehicle.pitch_rad) < 1.2F;
  const bool tilt_over_hard =
      attitude_sane &&
      (std::fabs(vehicle.roll_rad) >
           params_.tilt_cap.max_roll_angle_rad + params_.tilt_cap.hardcap_margin_rad ||
       std::fabs(vehicle.pitch_rad) >
           params_.tilt_cap.max_pitch_angle_rad + params_.tilt_cap.hardcap_margin_rad);

  if (tilt_over_hard) {
    if (state_ != StrikeState::ForceLevel) {
      force_level_enter_time_ns_ = inputs.now_ns;
    }
    state_ = StrikeState::ForceLevel;
    force_level_exit_candidate_time_ns_.reset();
  } else if (state_ == StrikeState::ForceLevel) {
    const bool min_hold_elapsed =
        force_level_enter_time_ns_.has_value() &&
        circle::types::secondsBetween(*force_level_enter_time_ns_, inputs.now_ns) *
                1000.0 >=
            static_cast<double>(params_.force_level.min_hold_ms);
    const bool well_inside =
        std::fabs(vehicle.roll_rad) <
            (params_.tilt_cap.max_roll_angle_rad - params_.tilt_cap.softcap_band_rad) &&
        std::fabs(vehicle.pitch_rad) <
            (params_.tilt_cap.max_pitch_angle_rad - params_.tilt_cap.softcap_band_rad);
    if (well_inside && min_hold_elapsed) {
      if (!force_level_exit_candidate_time_ns_.has_value()) {
        force_level_exit_candidate_time_ns_ = inputs.now_ns;
      }
      if (circle::types::secondsBetween(
              *force_level_exit_candidate_time_ns_, inputs.now_ns) >= 0.10) {
        force_level_enter_time_ns_.reset();
        force_level_exit_candidate_time_ns_.reset();
        state_ = StrikeState::WaitingTarget;
      }
    } else {
      force_level_exit_candidate_time_ns_.reset();
    }
  }

  bool fresh_detection = false;
  float detection_score = 0.0F;
  double detection_age_s = std::numeric_limits<double>::infinity();
  circle::types::TimestampNs detection_stamp_ns = 0;
  if (inputs.detection.valid) {
    detection_stamp_ns =
        effectiveDetectionStampNs(inputs.detection, inputs.now_ns);
    detection_age_s =
        circle::types::secondsBetween(detection_stamp_ns, inputs.now_ns);
    fresh_detection = detection_age_s <= params_.detection_stale_s;
    detection_score = inputs.detection.detection.score;
  }

  float tilt_aim_comp_x_px = 0.0F;
  float tilt_aim_comp_y_px = 0.0F;
  if (inputs.detection.valid) {
    const float bbox_area = inputs.detection.detection.width *
                            inputs.detection.detection.height;
    const float frame_area = static_cast<float>(
        inputs.detection.image_width * inputs.detection.image_height);
    const float bbox_area_ratio = frame_area > 1.0F ? bbox_area / frame_area : 0.0F;
    const auto& tac = params_.final_approach.tilt_aim_comp;
    if (tac.enable && tac.gain > 0.0F && tac.max_px > 0.0F &&
        tac.end_ratio > tac.start_ratio && frame_area > 1.0F) {
      const float t_raw = (bbox_area_ratio - tac.start_ratio) /
                          (tac.end_ratio - tac.start_ratio);
      const float blend = smoothstep01(t_raw);
      const float raw_x_px = tac.roll_sign * tac.gain *
          std::tan(vehicle.roll_rad) * inputs.detection.intrinsics.fx;
      const float raw_y_px = tac.pitch_sign * tac.gain *
          std::tan(vehicle.pitch_rad) * inputs.detection.intrinsics.fy;
      tilt_aim_comp_x_px = blend * std::clamp(raw_x_px, -tac.max_px, tac.max_px);
      tilt_aim_comp_y_px = blend * std::clamp(raw_y_px, -tac.max_px, tac.max_px);
    }
  }

  // 2.2: Tracker fallback (DKF prediction when bbox is stale)
  tracker_fallback_active_ = false;
  const bool tracker_fallback_requested =
      params_.tracker_fallback.enable && params_.dkf_enable &&
      inputs.detection.valid && std::isfinite(detection_age_s) &&
      detection_age_s > static_cast<double>(params_.tracker_fallback.after_s);
  if (tracker_fallback_requested &&
      detection_age_s <= static_cast<double>(params_.tracker_fallback.max_s) &&
      detection_score >= params_.tracker_fallback.min_score) {
    DelayedPixelKalman::Params fallback_params = params_.dkf;
    fallback_params.max_cov_trace = params_.tracker_fallback.max_cov_trace;
    tracker_fallback_est_ = dkf_.predict(inputs.now_ns, fallback_params);
    tracker_fallback_active_ =
        tracker_fallback_est_.valid &&
        tracker_fallback_est_.cov_trace <= params_.tracker_fallback.max_cov_trace;
  }

  if (fresh_detection && state_ != StrikeState::ForceLevel) {
    const std::vector<circle::types::Detection> dets{inputs.detection.detection};
    const auto filtered = vision::filterDetections(dets, params_.filter);
    if (filtered.best_index >= 0) {
      const auto& det = dets[static_cast<size_t>(filtered.best_index)];
      const float fx = std::max(1.0F, inputs.detection.intrinsics.fx);
      const float fy = std::max(1.0F, inputs.detection.intrinsics.fy);
      const float cx = inputs.detection.intrinsics.cx;
      const float cy = inputs.detection.intrinsics.cy;

      const float ex_px =
          (det.cx - cx) - params_.aim_offset_x_px - tilt_aim_comp_x_px;
      const float ey_px =
          (det.cy - cy) - params_.aim_offset_y_px - tilt_aim_comp_y_px;
      const float ex = ex_px / fx;
      const float ey = ey_px / fy;

      DelayedPixelKalman::Estimate image_est;
      if (params_.dkf_enable) {
        DelayedPixelKalman::Measurement meas;
        meas.image_stamp_ns = detection_stamp_ns;
        meas.receive_stamp_ns = inputs.detection.receive_ns;
        meas.ex = ex;
        meas.ey = ey;
        meas.bbox_area_px = det.width * det.height;
        meas.score = det.score;
        dkf_.addMeasurement(meas, inputs.detection.intrinsics, params_.dkf);
        image_est = dkf_.predict(inputs.now_ns, params_.dkf);
      }

      // 2.3: Strike confidence gate
      confidence_est_valid_ = false;
      if (params_.dkf_enable) {
        confidence_est_ = image_est;
        confidence_est_valid_ = confidence_est_.valid;
      } else {
        confidence_est_.valid = true;
        confidence_est_.ex = ex;
        confidence_est_.ey = ey;
        confidence_est_.ex_dot = ex_dot_filt_;
        confidence_est_.ey_dot = ey_dot_filt_;
        confidence_est_.cov_trace = 0.0F;
        confidence_est_valid_ = true;
      }
      
      const bool strike_gate_disabled = !params_.confidence.gate_enable;
      const bool strike_score_ok =
          det.score >= params_.confidence.min_score;
      const bool strike_cov_ok =
          !params_.dkf_enable ||
          (confidence_est_valid_ &&
           confidence_est_.cov_trace <= params_.confidence.max_cov_trace);
      const bool strike_no_tracker_fallback = !tracker_fallback_active_;
      strike_confident_ = strike_gate_disabled ||
          (strike_no_tracker_fallback && strike_score_ok && strike_cov_ok);

      const bool use_dkf_image_state =
          params_.dkf_enable && image_est.valid &&
          image_est.cov_trace <= params_.dkf.max_cov_trace;
      const float control_ex = use_dkf_image_state ? image_est.ex : ex;
      const float control_ey = use_dkf_image_state ? image_est.ey : ey;

      out.has_valid_target = true;
      out.image_ex = control_ex;
      out.image_ey = control_ey;
      last_image_ex_ = control_ex;
      last_image_ey_ = control_ey;
      last_detection_ns_ = detection_stamp_ns;
      detection_score = det.score;
      state_ = StrikeState::Tracking;
      target_seen_this_activation_ = true;
      if (!first_fresh_target_time_ns_.has_value()) {
        first_fresh_target_time_ns_ = inputs.now_ns;
      }
      last_target_seen_time_ns_ = inputs.now_ns;
      target_loss_completed_ = false;
    }
  } else if (last_detection_ns_ > 0) {
    const double lost_s =
        circle::types::secondsBetween(last_detection_ns_, inputs.now_ns);
    if (lost_s > params_.lost_timeout_s) {
      state_ = StrikeState::WaitingTarget;
      out.has_valid_target = false;
    } else if (tracker_fallback_active_) {
      // Use DKF prediction from tracker fallback
      out.has_valid_target = true;
      out.image_ex = tracker_fallback_est_.ex;
      out.image_ey = tracker_fallback_est_.ey;
      last_image_ex_ = tracker_fallback_est_.ex;
      last_image_ey_ = tracker_fallback_est_.ey;
    } else {
      out.has_valid_target = false;
    }
  } else {
    state_ = StrikeState::WaitingTarget;
  }

  out.state = state_;

  if (state_ == StrikeState::ForceLevel && inputs.mode_active && vehicle.valid) {
    commit_state_.active = false;
    commit_state_.terminal_ready = false;
    commit_state_.snapshot = FinalApproachCommitSnapshot{};
    commit_state_.recent_centered_snapshot = FinalApproachCommitSnapshot{};
    commit_state_.latch_start_time_ns.reset();
    commit_state_.align_since_ns.reset();
    fa_fallback_start_time_ns_.reset();

    const float roll_rate_des =
        -params_.force_level.hard_level_kp * vehicle.roll_rad;
    const float pitch_rate_des =
        -params_.force_level.hard_level_kp * vehicle.pitch_rad;
    const float yaw_rate_des = 0.0F;

    auto rate_output = rate_shaper_.compute(
        roll_rate_des, pitch_rate_des, yaw_rate_des,
        vehicle.roll_rad, vehicle.pitch_rad,
        params_.tilt_cap.max_roll_angle_rad, params_.tilt_cap.max_pitch_angle_rad,
        params_.tilt_cap.softcap_band_rad,
        params_.rate_lpf_tau_s, params_.max_jerk_rad_s2,
        params_.max_roll_rate_rad_s, params_.max_pitch_rate_rad_s,
        params_.yaw.rate_lpf_tau_s, params_.yaw.lock_enabled,
        1.0F, 1.0F, 1.0F, safe_dt, safe_dt, false);

    out.rates.roll_rate_rad_s = rate_output.roll_rate_rad_s;
    out.rates.pitch_rate_rad_s = rate_output.pitch_rate_rad_s;
    out.rates.yaw_rate_rad_s = rate_output.yaw_rate_rad_s;

    float ref_ned = 0.0F, err_ned = 0.0F, corr = 0.0F;
    out.rates.thrust_z = computeWaitingThrustZ(false, vehicle, ref_ned, err_ned, corr);

    out.telemetry.valid = true;
    out.telemetry.coasting = true;
    out.telemetry.final_approach_active = false;
    out.telemetry.roll_rate_sp_rad_s = out.rates.roll_rate_rad_s;
    out.telemetry.pitch_rate_sp_rad_s = out.rates.pitch_rate_rad_s;
    out.telemetry.yaw_rate_sp_rad_s = out.rates.yaw_rate_rad_s;
    out.telemetry.thrust_z = out.rates.thrust_z;
    out.telemetry.constant_thrust = params_.thrust.hover_scalar;
    out.telemetry.state = static_cast<int>(StrikeState::ForceLevel);
    out.telemetry.waiting_altitude_ref_ned = ref_ned;
    out.telemetry.waiting_altitude_error_ned = err_ned;
    out.telemetry.waiting_altitude_correction = corr;
    out.telemetry.roll_hard_headroom_rad = roll_hard_headroom_rad;
    out.telemetry.pitch_hard_headroom_rad = pitch_hard_headroom_rad;
    thrust_ramp_prev_publish_ = ThrustRampPrevPublish::ForceLevel;

  } else if (state_ == StrikeState::Tracking && out.has_valid_target) {
    const float dt_ctrl = dt_s > 1.0e-5F ? dt_s : 0.005F;
    
    // 2.1: Compute e_rho and rho_scale (distance-adaptive gain)
    // Note: These will be updated by FA gate module below
    const float bbox_area = std::max(1.0F,
        inputs.detection.detection.width * inputs.detection.detection.height);
    const float frame_area = static_cast<float>(
        inputs.detection.image_width * inputs.detection.image_height);
    float bbox_area_ratio = (frame_area > 1.0F) 
        ? (bbox_area / frame_area) : 0.0F;
    
    // Max edge ratio: more robust for elongated targets (e.g., 20x200px)
    const float max_edge_px = std::max(inputs.detection.detection.width, 
                                        inputs.detection.detection.height);
    const float max_edge_ratio = max_edge_px / std::max(
        static_cast<float>(inputs.detection.image_width),
        static_cast<float>(inputs.detection.image_height));
    
    float e_rho = 0.0F;
    if (params_.rho_scale.desired_bbox_area_px > 0.0) {
      e_rho = static_cast<float>(
          0.5 * std::log(params_.rho_scale.desired_bbox_area_px / bbox_area));
    }
    
    const float e_rho_near = -0.5F * std::log(
        std::max(1.0001F, params_.rho_scale.near_ratio));
    const float e_rho_far = -0.5F * std::log(
        std::clamp(params_.rho_scale.far_ratio, 1.0e-4F, 0.9999F));
    float rho_scale = (params_.rho_scale.desired_bbox_area_px > 0.0)
        ? lerp(e_rho, e_rho_near, e_rho_far,
               params_.rho_scale.scale_near, params_.rho_scale.scale_far)
        : 1.0F;
    last_rho_scale_ = rho_scale;
    
    // 2.7: Pixel velocity LPF (use detection->stamp for accurate dt).
    // This must run before FA gate so entry/hold decisions see the current
    // image-rate sample instead of the previous control tick.
    const bool new_measurement = fresh_detection &&
        (last_pixel_dot_time_ns_ == 0 ||
         detection_stamp_ns > last_pixel_dot_time_ns_);
    
    if (new_measurement) {
      if (last_ex_.has_value() && last_ey_.has_value() &&
          last_pixel_dot_time_ns_ > 0) {
        const float dt_meas = std::max(1.0e-3F, static_cast<float>(
            circle::types::secondsBetween(last_pixel_dot_time_ns_,
                                          detection_stamp_ns)));
        const float ex_dot_raw = (out.image_ex - *last_ex_) / dt_meas;
        const float ey_dot_raw = (out.image_ey - *last_ey_) / dt_meas;
        const float e_rho_dot_raw = last_e_rho_.has_value()
            ? ((e_rho - *last_e_rho_) / dt_meas) : 0.0F;
        const float pixel_dot_lpf_tau_eff = final_approach_active_
            ? params_.pixel_dot_lpf_tau_s * params_.final_approach.scaling.pixel_dot_lpf_scale
            : params_.pixel_dot_lpf_tau_s;
        const float a = lpfAlpha(pixel_dot_lpf_tau_eff, dt_meas);
        ex_dot_filt_ += a * (ex_dot_raw - ex_dot_filt_);
        ey_dot_filt_ += a * (ey_dot_raw - ey_dot_filt_);
        e_rho_dot_filt_ += a * (e_rho_dot_raw - e_rho_dot_filt_);
        if (params_.rho_rate_window.enable) {
          e_rho_dot_filt_ = rho_rate_window_.addSample(
              params_.rho_rate_window, e_rho, detection_stamp_ns);
        }
      } else {
        ex_dot_filt_ = 0.0F;
        ey_dot_filt_ = 0.0F;
        e_rho_dot_filt_ = 0.0F;
        rho_rate_window_.reset();
      }
      last_ex_ = out.image_ex;
      last_ey_ = out.image_ey;
      last_e_rho_ = e_rho;
      last_pixel_dot_time_ns_ = detection_stamp_ns;
    } else if (tracker_fallback_active_) {
      ex_dot_filt_ = tracker_fallback_est_.ex_dot;
      ey_dot_filt_ = tracker_fallback_est_.ey_dot;
      e_rho_dot_filt_ = 0.0F;
      rho_rate_window_.reset();
      last_ex_ = out.image_ex;
      last_ey_ = out.image_ey;
      last_e_rho_ = e_rho;
    } else if (inputs.detection.valid && !fresh_detection) {
      ex_dot_filt_ = 0.0F;
      ey_dot_filt_ = 0.0F;
      e_rho_dot_filt_ = 0.0F;
      rho_rate_window_.reset();
      last_ex_ = out.image_ex;
      last_ey_ = out.image_ey;
      last_e_rho_ = e_rho;
    }
    
    // 3.1: FA gate logic (using module)
    auto fa_gate_output = final_approach_gate_.compute(
        fa_gate_state_,
        params_.final_approach.gate,
        params_.speed_governor,
        params_.preclimb,
        params_.evaluation,
        params_.rho_scale,
        preclimb_state_.xy_gate_released,
        fresh_detection, strike_confident_,
        inputs.detection, vehicle,
        out.image_ex, out.image_ey,
        ex_dot_filt_, ey_dot_filt_,
        inputs.now_ns);
    
    final_approach_active_ = fa_gate_output.active;
    if (final_approach_active_) {
      fa_last_active_time_ns_ = inputs.now_ns;
      preclimb_state_.xy_gate_released = true;
      if (!preclimb_state_.xy_released_since_ns.has_value()) {
        preclimb_state_.xy_released_since_ns = inputs.now_ns;
      }
    }
    if (!final_approach_active_) {
      if (!commit_state_.active) {
        commit_state_.terminal_ready = false;
        commit_state_.recent_centered_snapshot = FinalApproachCommitSnapshot{};
      }
    }
    
    // Use module outputs
    bbox_area_ratio = fa_gate_output.bbox_area_ratio;
    e_rho = fa_gate_output.e_rho;
    rho_scale = fa_gate_output.rho_scale;
    
    // 2.4: Image lead (forward prediction)
    float ex_ctrl = out.image_ex;
    float ey_ctrl = out.image_ey;
    float image_lead_x_px = 0.0F;
    float image_lead_y_px = 0.0F;
    bool image_lead_active = false;
    const auto& il = params_.image_lead;
    float start_kp_scale = 1.0F;
    float start_kd_scale = 1.0F;
    float start_lead_scale = 1.0F;
    const auto& ss = params_.tracking_start_smoothing;
    if (ss.enable && !final_approach_active_ && ss.smoothing_s > 1.0e-3F &&
        first_fresh_target_time_ns_.has_value()) {
      const float age_s = static_cast<float>(
          circle::types::secondsBetween(*first_fresh_target_time_ns_, inputs.now_ns));
      const float blend = smoothstep01(age_s / ss.smoothing_s);
      start_kp_scale = ss.kp_scale_initial +
                       (1.0F - ss.kp_scale_initial) * blend;
      start_kd_scale = ss.kd_scale_initial +
                       (1.0F - ss.kd_scale_initial) * blend;
      start_lead_scale = ss.lead_scale_initial +
                         (1.0F - ss.lead_scale_initial) * blend;
    }
    const bool terminal_predictor_configured =
        final_approach_active_ && params_.final_approach.terminal_predictor.enable;
    if (!terminal_predictor_configured &&
        il.enable && il.time_s > 0.0F && il.max_px > 0.0F &&
        (fresh_detection || tracker_fallback_active_)) {
      const float fx = std::max(1.0F, inputs.detection.intrinsics.fx);
      const float fy = std::max(1.0F, inputs.detection.intrinsics.fy);
      const float lead_x_norm = std::clamp(
          ex_dot_filt_ * il.time_s, -il.max_px / fx, il.max_px / fx);
      const float lead_y_norm = std::clamp(
          ey_dot_filt_ * il.time_s, -il.max_px / fy, il.max_px / fy);
      ex_ctrl += lead_x_norm * start_lead_scale;
      ey_ctrl += lead_y_norm * start_lead_scale;
      image_lead_x_px = lead_x_norm * fx * start_lead_scale;
      image_lead_y_px = lead_y_norm * fy * start_lead_scale;
      image_lead_active = std::abs(image_lead_x_px) > 1.0F ||
                          std::abs(image_lead_y_px) > 1.0F;
    }

    const auto terminal_predictor_output = terminal_predictor_.compute(
        params_.final_approach.terminal_predictor,
        TerminalPredictorInput{
            final_approach_active_,
            fresh_detection,
            bbox_area_ratio,
            out.image_ex,
            out.image_ey,
            ex_dot_filt_,
            ey_dot_filt_,
            e_rho_dot_filt_,
            inputs.detection.intrinsics.fx,
            inputs.detection.intrinsics.fy});
    if (terminal_predictor_output.active) {
      ex_ctrl = terminal_predictor_output.predicted_ex;
      ey_ctrl = terminal_predictor_output.predicted_ey;
    }
    
    // 2.8: Distance-adaptive deadband (tapers from full at far range to
    // final_approach_deadband_scale at near range; snaps to FA scale when active)
    const float range_db_scale = (params_.rho_scale.desired_bbox_area_px > 0.0)
        ? lerp(e_rho, e_rho_near, e_rho_far,
               params_.final_approach.scaling.deadband_scale, 1.0F)
        : 1.0F;
    const float fa_db_scale = final_approach_active_
        ? params_.final_approach.scaling.deadband_scale
        : range_db_scale;
    const float x_deadband_norm =
        params_.x_deadband_px > 0.0F && inputs.detection.intrinsics.fx > 1.0e-6F
            ? (params_.x_deadband_px * fa_db_scale) / inputs.detection.intrinsics.fx
            : params_.x_deadband;
    const float y_deadband_norm =
        params_.y_deadband_px > 0.0F && inputs.detection.intrinsics.fy > 1.0e-6F
            ? (params_.y_deadband_px * fa_db_scale) / inputs.detection.intrinsics.fy
            : params_.y_deadband;
    
    const float ex_db = applyDeadband(ex_ctrl, x_deadband_norm);
    const float ey_db = applyDeadband(ey_ctrl, y_deadband_norm);
    
    // 2.5: Proximity error normalization
    const float prox_norm = (final_approach_active_ &&
        params_.final_approach.scaling.proximity_error_norm && e_rho < 0.0F)
        ? std::exp(e_rho) : 1.0F;
    const float ex_pd = ex_db * prox_norm;
    const float ey_pd = ey_db * prox_norm;
    const float ex_dot_pd = ex_dot_filt_ * prox_norm;
    const float ey_dot_pd = ey_dot_filt_ * prox_norm;
    
    // 2.9: FA PD gain scaling + proximity gain
    float final_kp_scale = 1.0F;
    float final_kd_scale = 1.0F;
    if (final_approach_active_) {
      const float proximity = std::max(0.0F, -e_rho);
      final_kp_scale = params_.final_approach.scaling.kp_scale *
          (1.0F + params_.final_approach.scaling.kp_proximity_gain * proximity);
      final_kd_scale = params_.final_approach.scaling.kd_scale *
          (1.0F + params_.final_approach.scaling.kd_proximity_gain * proximity);
    }
    final_kp_scale *= start_kp_scale;
    final_kd_scale *= start_kd_scale;
    
    // Image PD controller
    float roll_cmd = params_.lateral_output_sign *
        (params_.lateral_kp_rate * final_kp_scale * ex_pd +
         params_.lateral_kd_rate * final_kd_scale * ex_dot_pd);
    float pitch_cmd = params_.longitudinal_output_sign *
        (params_.longitudinal_kp_rate * final_kp_scale * ey_pd +
         params_.longitudinal_kd_rate * final_kd_scale * ey_dot_pd);

    auto png_output = visual_png_guidance_.compute(
        params_.visual_png,
        VisualPngGuidanceInput{
            params_.guidance_mode == GuidanceMode::PaperPng,
            ex_ctrl,
            ey_ctrl,
            ex_dot_filt_,
            ey_dot_filt_,
            e_rho_dot_filt_,
            bbox_area_ratio,
            vehicle.roll_rate_rad_s,
            vehicle.pitch_rate_rad_s,
            params_.lateral_output_sign,
            params_.longitudinal_output_sign,
            params_.max_roll_rate_rad_s,
            params_.max_pitch_rate_rad_s});
    if (png_output.active) {
      roll_cmd = png_output.roll_rate_rad_s;
      pitch_cmd = png_output.pitch_rate_rad_s;
    }

    // 2.6: Approach pitch bias (far-range e_rho drive)
    float approach_pitch_bias = 0.0F;
    const auto& ad = params_.approach_drive;
    if (ad.enable && !final_approach_active_ &&
        e_rho > ad.e_rho_deadband && ad.pitch_rate_max_rad_s > 0.0F) {
      const float e_rho_drive = e_rho - ad.e_rho_deadband;
      const float fov_scale = approachDriveFovScale(
          ad, out.image_ey, inputs.detection.intrinsics.fy);
      const float bias_mag = fov_scale * std::clamp(
          ad.pitch_rate_gain * e_rho_drive, 0.0F, ad.pitch_rate_max_rad_s);
      approach_pitch_bias = ad.pitch_output_sign * bias_mag;
      pitch_cmd += approach_pitch_bias;
    }
    
    // 6.1: Speed governor (scale down rates at high vehicle speed)
    auto gov_output = speed_governor_.compute(
        params_.speed_governor,
        final_approach_active_,
        vehicle.velocity_xy_valid,
        vehicle.velocity_xy_m_s,
        vehicle.roll_rad,
        vehicle.pitch_rad,
        roll_cmd,
        pitch_cmd);
    roll_cmd = gov_output.roll_rate_rad_s;
    pitch_cmd = gov_output.pitch_rate_rad_s;
    
    // 5.1: Edge protection (using module)
    auto edge_output = edge_protection_.compute(
        final_approach_active_, new_measurement,
        inputs.detection, vehicle,
        out.image_ex, out.image_ey,
        ex_dot_filt_, ey_dot_filt_,
        params_.lateral_output_sign, params_.longitudinal_output_sign,
        params_.final_approach.edge_protect,
        params_.final_approach.bottom_pitch_guard,
        params_.final_approach.commit.min_margin_x_px,
        params_.final_approach.commit.min_margin_y_px);
    
    const bool edge_protect_active = edge_output.active;
    const float edge_taper_score = edge_output.taper_score;
    const float bottom_pitch_guard_blend = edge_output.bottom_pitch_guard_blend;
    
    // Apply edge protection boost to rate commands
    roll_cmd += edge_output.roll_boost;
    pitch_cmd += edge_output.pitch_boost;
    
    // Apply bottom pitch guard (blend toward level rate when target is low)
    if (edge_output.bottom_pitch_guard_active && bottom_pitch_guard_blend > 0.0F) {
      pitch_cmd = pitch_cmd * (1.0F - bottom_pitch_guard_blend) +
                  edge_output.bottom_pitch_guard_rate * bottom_pitch_guard_blend;
    }
    
    // 3.5: e_rho_dot pitch feedforward
    if (final_approach_active_ && e_rho_dot_filt_ < 0.0F) {
      pitch_cmd += params_.final_approach.scaling.pitch_ff_erho_gain * e_rho_dot_filt_;
    }
    
    // Compute bbox margins for preclimb
    const float bbox_cx = inputs.detection.detection.cx;
    const float bbox_cy = inputs.detection.detection.cy;
    const float bbox_w = inputs.detection.detection.width;
    const float bbox_h = inputs.detection.detection.height;
    const float bbox_margin_x_px = std::min(
        bbox_cx - bbox_w * 0.5F,
        static_cast<float>(inputs.detection.image_width) - (bbox_cx + bbox_w * 0.5F));
    const float bbox_margin_y_px = std::min(
        bbox_cy - bbox_h * 0.5F,
        static_cast<float>(inputs.detection.image_height) - (bbox_cy + bbox_h * 0.5F));
    
    // 7.1: Preclimb (safe hold + level assist when gate not released)
    auto preclimb_output = preclimb_module_.compute(
        preclimb_state_,
        params_.preclimb,
        final_approach_active_,
        fresh_detection,
        strike_confident_,
        out.image_ex, out.image_ey,
        ex_dot_filt_, ey_dot_filt_,
        inputs.detection.intrinsics.fx,
        inputs.detection.intrinsics.fy,
        bbox_margin_x_px, bbox_margin_y_px,
        vehicle.roll_rad, vehicle.pitch_rad,
        roll_cmd, pitch_cmd,
        inputs.now_ns);
    roll_cmd = preclimb_output.roll_rate_rad_s;
    pitch_cmd = preclimb_output.pitch_rate_rad_s;
    
    // 3.4: FA roll/pitch leveling
    float fa_roll_level_blend = 0.0F;
    float fa_roll_level_rate = 0.0F;
    const auto& rl = params_.final_approach.leveling;
    const bool disable_fa_pitch_leveling_close_range = max_edge_ratio > 0.15F;
    if (final_approach_active_ &&
        rl.roll_level_kp > 0.0F &&
        rl.roll_level_max_rad_s > 0.0F &&
        rl.roll_level_end_ratio > rl.roll_level_start_ratio) {
      const float level_t_raw =
          (bbox_area_ratio - rl.roll_level_start_ratio) /
          (rl.roll_level_end_ratio - rl.roll_level_start_ratio);
      const float level_t = std::clamp(level_t_raw, 0.0F, 1.0F);
      fa_roll_level_blend = smoothstep01(level_t);
      fa_roll_level_rate = std::clamp(
          -rl.roll_level_kp * vehicle.roll_rad,
          -rl.roll_level_max_rad_s,
          rl.roll_level_max_rad_s);
      roll_cmd = roll_cmd * (1.0F - fa_roll_level_blend) +
                 fa_roll_level_rate * fa_roll_level_blend;
    }
    
    float fa_pitch_level_blend = 0.0F;
    float fa_pitch_level_rate = 0.0F;
    float fa_pitch_level_y_guard = 1.0F;
    if (final_approach_active_ &&
        !disable_fa_pitch_leveling_close_range &&
        rl.pitch_level_kp > 0.0F &&
        rl.pitch_level_max_rad_s > 0.0F &&
        rl.pitch_level_end_ratio > rl.pitch_level_start_ratio) {
      const float level_t_raw =
          (bbox_area_ratio - rl.pitch_level_start_ratio) /
          (rl.pitch_level_end_ratio - rl.pitch_level_start_ratio);
      const float level_t = std::clamp(level_t_raw, 0.0F, 1.0F);
      fa_pitch_level_blend = smoothstep01(level_t);
      
      if (fresh_detection && inputs.detection.intrinsics.fy > 1.0e-6F) {
        const float y_error_full_px = std::max(
            params_.final_approach.commit.align_max_error_y_px * 1.5F,
            y_deadband_norm * inputs.detection.intrinsics.fy * 1.5F);
        const float y_error_zero_px = std::max(
            y_error_full_px + 1.0F,
            params_.final_approach.commit.align_max_error_y_px * 4.0F);
        const float fa_align_error_y_px =
            std::abs(out.image_ey * inputs.detection.intrinsics.fy);
        const float y_error_t = std::clamp(
            (fa_align_error_y_px - y_error_full_px) /
                (y_error_zero_px - y_error_full_px),
            0.0F, 1.0F);
        const float y_error_guard = 1.0F - smoothstep01(y_error_t);
        
        const float y_rate_px_s =
            std::abs(ey_dot_filt_ * inputs.detection.intrinsics.fy);
        const float y_rate_full_px_s = std::max(
            30.0F, params_.final_approach.thrust.vertical_drift_start_px_s * 0.45F);
        const float y_rate_zero_px_s = std::max(
            y_rate_full_px_s + 1.0F,
            params_.final_approach.thrust.vertical_drift_start_px_s * 1.15F);
        const float y_rate_t = std::clamp(
            (y_rate_px_s - y_rate_full_px_s) /
                (y_rate_zero_px_s - y_rate_full_px_s),
            0.0F, 1.0F);
        const float y_rate_guard = 1.0F - smoothstep01(y_rate_t);
        fa_pitch_level_y_guard = std::min(y_error_guard, y_rate_guard);
        fa_pitch_level_blend *= fa_pitch_level_y_guard;
      }
      
      fa_pitch_level_rate = std::clamp(
          -rl.pitch_level_kp * vehicle.pitch_rad,
          -rl.pitch_level_max_rad_s,
          rl.pitch_level_max_rad_s);
      pitch_cmd = pitch_cmd * (1.0F - fa_pitch_level_blend) +
                  fa_pitch_level_rate * fa_pitch_level_blend;
    }

    if (edge_protect_active) {
      roll_cmd += edge_output.roll_boost * fa_roll_level_blend;
      pitch_cmd += edge_output.pitch_boost * fa_pitch_level_blend;
    }
    
    // 4.1: Tilt softcap is applied inside RateShaper before jerk/LPF shaping.
    
    // 4.4.5: Pitch chatter guard (after softcap, before rate shaping)
    const auto& pcg = params_.final_approach.pitch_chatter_guard;
    const float prev_pitch_slew = rate_shaper_.prevPitchRateSlew();
    if (final_approach_active_ && pcg.enable && fresh_detection &&
        !edge_protect_active &&
        pcg.max_area_ratio > 0.0F && bbox_area_ratio <= pcg.max_area_ratio &&
        pcg.max_error_y_px > 0.0F && pcg.max_rate_y_px_s > 0.0F &&
        std::abs(out.image_ey * inputs.detection.intrinsics.fy) <= pcg.max_error_y_px &&
        std::abs(ey_dot_filt_ * inputs.detection.intrinsics.fy) <= pcg.max_rate_y_px_s &&
        std::abs(prev_pitch_slew) >= pcg.prev_min_rate_rad_s &&
        pitch_cmd * prev_pitch_slew < 0.0F) {
      pitch_cmd = std::clamp(pitch_cmd,
                             -pcg.max_reversal_rate_rad_s,
                             pcg.max_reversal_rate_rad_s);
    }

    // 4.5: FA-scaled shaping (jerk/rate scaling during final approach)
    const float fa_jerk_scale = final_approach_active_
         ? params_.final_approach.scaling.jerk_scale : 1.0F;
    const float fa_rate_scale = final_approach_active_
        ? params_.final_approach.scaling.roll_rate_scale : 1.0F;
    
    // 4.4: Unified rate axis shaping (using module)
    auto yaw_output = yaw_controller_.compute(
        out.image_ex, rho_scale,
        params_.yaw.bearing_kp,
        params_.yaw.track_deadband_rad,
        params_.yaw.rate_min_rad_s,
        params_.yaw.rate_max_rad_s);
    
    const float rate_lpf_tau_eff = final_approach_active_
        ? params_.rate_lpf_tau_s * params_.final_approach.scaling.rate_lpf_scale
        : params_.rate_lpf_tau_s;

    auto rate_output = rate_shaper_.compute(
        roll_cmd, pitch_cmd, yaw_output.yaw_rate_target,
        vehicle.roll_rad, vehicle.pitch_rad,
        params_.tilt_cap.max_roll_angle_rad, params_.tilt_cap.max_pitch_angle_rad,
        params_.tilt_cap.softcap_band_rad,
        rate_lpf_tau_eff, params_.max_jerk_rad_s2,
        params_.max_roll_rate_rad_s, params_.max_pitch_rate_rad_s,
        params_.yaw.rate_lpf_tau_s, params_.yaw.lock_enabled,
        rho_scale, fa_jerk_scale, fa_rate_scale,
        dt_ctrl, safe_dt, params_.rate_shaper_diag_log);
    
    out.rates.roll_rate_rad_s = rate_output.roll_rate_rad_s;
    out.rates.pitch_rate_rad_s = rate_output.pitch_rate_rad_s;
    out.rates.yaw_rate_rad_s = rate_output.yaw_rate_rad_s;
    const float roll_softcap_factor = rate_output.roll_softcap_factor;
    const float pitch_softcap_factor = rate_output.pitch_softcap_factor;
    
    // 7.5: Tilt Guard
    auto tilt_guard_output = tilt_guard_.compute(
        params_.tilt_guard,
        vehicle,
        out.rates.roll_rate_rad_s,
        out.rates.pitch_rate_rad_s,
        max_edge_ratio);
    
    if (tilt_guard_output.active) {
      if (std::abs(tilt_guard_output.pitch_rate_correction) > 0.01F) {
        out.rates.pitch_rate_rad_s = tilt_guard_output.pitch_rate_correction;
      }
      if (std::abs(tilt_guard_output.roll_rate_correction) > 0.01F) {
        out.rates.roll_rate_rad_s = tilt_guard_output.roll_rate_correction;
      }
    }
    
    // Directional Dive
    auto dive_output = directional_dive_.compute(
        dive_state_,
        params_.directional_dive,
        params_.tilt_guard,
        final_approach_active_,
        max_edge_ratio,
        tilt_guard_output.tilt_angle,
        out.image_ex,
        out.image_ey,
        ex_dot_filt_,
        ey_dot_filt_,
        vehicle.pitch_rad,
        vehicle.roll_rad,
        gov_output.scale,
        out.rates.thrust_z,
        out.rates.roll_rate_rad_s,
        out.rates.pitch_rate_rad_s,
        detection_stamp_ns,
        inputs.now_ns,
        dt_ctrl);
    
    if (dive_output.active) {
      out.rates.roll_rate_rad_s = dive_output.roll_rate_rad_s;
      out.rates.pitch_rate_rad_s = dive_output.pitch_rate_rad_s;
      out.rates.thrust_z = dive_output.thrust_z;
    }
    
    // Tilt thrust compensation: cos(tilt) reduces effective vertical thrust
    const float tilt_cos = std::cos(std::hypot(vehicle.pitch_rad, vehicle.roll_rad));
    if (tilt_cos < params_.directional_dive.tilt_comp_threshold) {
      out.rates.thrust_z = std::min(0.99F,
          out.rates.thrust_z + (params_.directional_dive.tilt_comp_threshold - tilt_cos) * params_.directional_dive.tilt_comp_gain);
    }
    
    // 8.0: Thrust management (using module)
    auto thrust_output = thrust_manager_.compute(
        thrust_state_,
        params_,
        final_approach_active_,
        edge_protect_active,
        edge_output.thrust_scale,
        edge_output.taper_score,
        preclimb_output.xy_gate_active,
        preclimb_state_.xy_gate_released,
        preclimb_state_.xy_released_since_ns,
        fresh_detection,
        bbox_area_ratio,
        out.image_ex, out.image_ey,
        ex_dot_filt_, ey_dot_filt_,
        inputs.detection.intrinsics.fx,
        inputs.detection.intrinsics.fy,
        vehicle.roll_rad, vehicle.pitch_rad,
        x_deadband_norm * inputs.detection.intrinsics.fx,
        y_deadband_norm * inputs.detection.intrinsics.fy,
        std::abs(out.image_ex * inputs.detection.intrinsics.fx),
        std::abs(out.image_ey * inputs.detection.intrinsics.fy),
        commit_state_.terminal_ready,
        inputs.now_ns,
        dt_ctrl);
    
    out.rates.thrust_z = thrust_output.thrust_z;

    const float align_error_x_px = std::abs(out.image_ex * inputs.detection.intrinsics.fx);
    const float align_error_y_px = std::abs(out.image_ey * inputs.detection.intrinsics.fy);
    const float align_rate_x_px_s = std::abs(ex_dot_filt_ * inputs.detection.intrinsics.fx);
    const float align_rate_y_px_s = std::abs(ey_dot_filt_ * inputs.detection.intrinsics.fy);
    const auto& cp = params_.final_approach.commit;
    const float future_lead_s = std::max(0.0F, cp.future_lead_s);
    const float future_error_x_px =
        std::abs((out.image_ex + ex_dot_filt_ * future_lead_s) *
                 inputs.detection.intrinsics.fx);
    const float future_error_y_px =
        std::abs((out.image_ey + ey_dot_filt_ * future_lead_s) *
                 inputs.detection.intrinsics.fy);
    
    // Close-range commit bypass: when target is very close (max edge > 20% of frame),
    // commit regardless of confidence. At close range, the target is physically there
    // and commit is the correct behavior even if detection stalls.
    // Using max_edge_ratio instead of bbox_area_ratio for robustness with elongated targets.
    const bool close_range_commit = final_approach_active_ && max_edge_ratio > 0.20F;
    const bool measure_reliable = (fresh_detection && strike_confident_) || close_range_commit;
    
    const bool stable_centered = fa_gate_state_.stable_since_ns.has_value();
    const float snapshot_area_ratio =
        cp.stable_bypass_area_enable && stable_centered
            ? std::max(bbox_area_ratio, cp.min_area_ratio)
            : bbox_area_ratio;
    const bool create_snapshot = commit_module_.shouldCreateSnapshot(
        commit_state_, cp, final_approach_active_, measure_reliable,
        strike_confident_, snapshot_area_ratio, bbox_margin_x_px, bbox_margin_y_px,
        align_error_x_px, align_error_y_px, align_rate_x_px_s, align_rate_y_px_s,
        future_error_x_px, future_error_y_px,
        vehicle.roll_rad, vehicle.pitch_rad, edge_protect_active, inputs.now_ns);
    if (create_snapshot) {
      commit_module_.updateSnapshot(
          commit_state_, false, out.rates.roll_rate_rad_s,
          out.rates.pitch_rate_rad_s, out.rates.yaw_rate_rad_s,
          out.rates.thrust_z, bbox_area_ratio, bbox_margin_x_px, bbox_margin_y_px,
          align_error_x_px, align_error_y_px, out.image_ex, out.image_ey,
          ex_dot_filt_, ey_dot_filt_, inputs.now_ns, detection_stamp_ns);
      commit_state_.recent_centered_snapshot = commit_state_.snapshot;
    }

    const bool live_centered_anchor_ok =
        cp.enable &&
        final_approach_active_ &&
        measure_reliable &&
        (cp.terminal_max_error_x_px <= 0.0F ||
         align_error_x_px <= cp.terminal_max_error_x_px) &&
        (cp.terminal_max_error_y_px <= 0.0F ||
         align_error_y_px <= cp.terminal_max_error_y_px) &&
        bbox_margin_x_px >= cp.min_margin_x_px &&
        bbox_margin_y_px >= cp.min_margin_y_px &&
        !edge_protect_active &&
        (cp.snapshot_max_area_ratio <= 0.0F ||
         bbox_area_ratio <= cp.snapshot_max_area_ratio) &&
        !commit_state_.active &&
        !commit_state_.terminal_ready;
    if (live_centered_anchor_ok) {
      FinalApproachCommitSnapshot live_anchor;
      live_anchor.valid = true;
      live_anchor.blind_terminal = false;
      live_anchor.recent_centered_terminal = false;
      live_anchor.command_stamp_ns = inputs.now_ns;
      live_anchor.detection_stamp_ns = detection_stamp_ns;
      live_anchor.roll_rate_sp_rad_s = out.rates.roll_rate_rad_s;
      live_anchor.pitch_rate_sp_rad_s = out.rates.pitch_rate_rad_s;
      live_anchor.yaw_rate_sp_rad_s = params_.yaw.lock_enabled
          ? 0.0F : out.rates.yaw_rate_rad_s;
      live_anchor.thrust_z = out.rates.thrust_z;
      live_anchor.bbox_area_ratio = bbox_area_ratio;
      live_anchor.margin_x_px = bbox_margin_x_px;
      live_anchor.margin_y_px = bbox_margin_y_px;
      live_anchor.align_error_x_px = align_error_x_px;
      live_anchor.align_error_y_px = align_error_y_px;
      live_anchor.ex = out.image_ex;
      live_anchor.ey = out.image_ey;
      live_anchor.ex_dot = ex_dot_filt_;
      live_anchor.ey_dot = ey_dot_filt_;
      commit_state_.recent_centered_snapshot = live_anchor;
    }

    commit_state_.terminal_ready = commit_module_.checkTerminalReady(
        commit_state_, cp, final_approach_active_, bbox_area_ratio,
        align_error_x_px, align_error_y_px, inputs.now_ns);

    if (!commit_state_.active && commit_state_.terminal_ready &&
        cp.start_on_terminal_ready_enable) {
      commit_state_.active = true;
      commit_state_.latch_start_time_ns = inputs.now_ns;
    }
    if (!commit_state_.active && cp.recent_centered_handoff_enable &&
        commit_state_.recent_centered_snapshot.valid) {
      const double recent_age_s = circle::types::secondsBetween(
          commit_state_.recent_centered_snapshot.command_stamp_ns, inputs.now_ns);
      const bool recent_ok = recent_age_s >= 0.0 &&
          recent_age_s <= static_cast<double>(cp.recent_centered_handoff_max_age_s) &&
          bbox_area_ratio >= cp.recent_centered_handoff_min_area_ratio &&
          bbox_area_ratio + 1.0e-5F >=
              commit_state_.recent_centered_snapshot.bbox_area_ratio;
      const bool live_escaping =
          align_error_x_px >= cp.recent_centered_handoff_trigger_error_x_px ||
          align_error_y_px >= cp.recent_centered_handoff_trigger_error_y_px;
      if (recent_ok && live_escaping) {
        const float live_blend = cp.recent_centered_handoff_live_blend;
        auto snap = commit_state_.recent_centered_snapshot;
        snap.recent_centered_terminal = true;
        snap.blind_terminal = true;
        snap.command_stamp_ns = inputs.now_ns;
        snap.detection_stamp_ns = detection_stamp_ns;
        snap.roll_rate_sp_rad_s = snap.roll_rate_sp_rad_s * (1.0F - live_blend) +
                                  out.rates.roll_rate_rad_s * live_blend;
        snap.pitch_rate_sp_rad_s = snap.pitch_rate_sp_rad_s * (1.0F - live_blend) +
                                   out.rates.pitch_rate_rad_s * live_blend;
        snap.yaw_rate_sp_rad_s = snap.yaw_rate_sp_rad_s * (1.0F - live_blend) +
                                 out.rates.yaw_rate_rad_s * live_blend;
        snap.thrust_z = snap.thrust_z * (1.0F - live_blend) +
                        out.rates.thrust_z * live_blend;
        commit_state_.snapshot = snap;
        commit_state_.terminal_ready = true;
        commit_state_.active = true;
        commit_state_.latch_start_time_ns = inputs.now_ns;
      }
    }

    if (!commit_state_.active && !commit_state_.terminal_ready &&
        cp.enable && cp.start_on_terminal_ready_enable &&
        final_approach_active_ && measure_reliable &&
        commit_state_.snapshot.valid &&
        !commit_state_.snapshot.blind_terminal) {
      const double snap_age_s = circle::types::secondsBetween(
          commit_state_.snapshot.command_stamp_ns, inputs.now_ns);
      const bool snap_fresh = snap_age_s >= 0.0 &&
          snap_age_s <= static_cast<double>(cp.detection_stale_s);
      const bool snap_x_ok = cp.terminal_max_error_x_px <= 0.0F ||
          commit_state_.snapshot.align_error_x_px <= cp.terminal_max_error_x_px;
      const bool snap_y_ok = cp.terminal_max_error_y_px <= 0.0F ||
          commit_state_.snapshot.align_error_y_px <= cp.terminal_max_error_y_px;
      const bool snap_progress_ok =
          bbox_area_ratio + 1.0e-4F >= commit_state_.snapshot.bbox_area_ratio;
      const bool terminal_area_ok = cp.terminal_min_area_ratio <= 0.0F ||
          bbox_area_ratio >= cp.terminal_min_area_ratio;
      const bool terminal_align_lost =
          (cp.terminal_max_error_x_px > 0.0F &&
           align_error_x_px > cp.terminal_max_error_x_px) ||
          (cp.terminal_max_error_y_px > 0.0F &&
           align_error_y_px > cp.terminal_max_error_y_px);
      if (snap_fresh && snap_x_ok && snap_y_ok && snap_progress_ok &&
          terminal_area_ok && terminal_align_lost) {
        commit_state_.terminal_ready = true;
        commit_state_.active = true;
        commit_state_.snapshot.blind_terminal = true;
        commit_state_.snapshot.recent_centered_terminal = false;
        commit_state_.snapshot.command_stamp_ns = inputs.now_ns;
        commit_state_.snapshot.detection_stamp_ns = detection_stamp_ns;
        commit_state_.snapshot.thrust_z = out.rates.thrust_z;
        commit_state_.latch_start_time_ns = inputs.now_ns;
      }
    }

    if (!commit_state_.active && commit_state_.terminal_ready &&
        (!measure_reliable || (cp.blind_commit_enable &&
         commit_module_.shouldCreateBlindSnapshot(
             commit_state_, cp, final_approach_active_, true, bbox_area_ratio,
             bbox_margin_x_px, bbox_margin_y_px, align_error_x_px,
             align_error_y_px, align_rate_x_px_s, align_rate_y_px_s,
             ex_dot_filt_, ey_dot_filt_, detection_score, inputs.now_ns)))) {
      commit_state_.active = true;
      commit_state_.snapshot.blind_terminal = true;
      commit_state_.snapshot.recent_centered_terminal = false;
      commit_state_.snapshot.command_stamp_ns = inputs.now_ns;
      commit_state_.snapshot.detection_stamp_ns = detection_stamp_ns;
      if (cp.blind_commit_trend_lead_s > 0.0F &&
          cp.blind_commit_trend_max_rate_rad_s > 0.0F) {
        const float trend_roll_rate =
            params_.lateral_output_sign * params_.lateral_kp_rate * ex_dot_filt_ *
            cp.blind_commit_trend_lead_s;
        const float trend_pitch_rate =
            params_.longitudinal_output_sign * params_.longitudinal_kp_rate * ey_dot_filt_ *
            cp.blind_commit_trend_lead_s;
        commit_state_.snapshot.roll_rate_sp_rad_s = std::clamp(
            commit_state_.snapshot.roll_rate_sp_rad_s + trend_roll_rate,
            -cp.blind_commit_trend_max_rate_rad_s,
            cp.blind_commit_trend_max_rate_rad_s);
        commit_state_.snapshot.pitch_rate_sp_rad_s = std::clamp(
            commit_state_.snapshot.pitch_rate_sp_rad_s + trend_pitch_rate,
            -cp.blind_commit_trend_max_rate_rad_s,
            cp.blind_commit_trend_max_rate_rad_s);
      }
      commit_state_.latch_start_time_ns = inputs.now_ns;
    }

    if (commit_state_.active && !commit_state_.snapshot.blind_terminal &&
        measure_reliable) {
      const double latch_age_s =
          commit_state_.latch_start_time_ns.has_value()
              ? circle::types::secondsBetween(
                    *commit_state_.latch_start_time_ns, inputs.now_ns)
              : std::numeric_limits<double>::infinity();
      const bool min_latch_active =
          cp.min_latch_s > 0.0F &&
          latch_age_s < static_cast<double>(cp.min_latch_s);
      const bool recent_centered_hold_through_active =
          commit_state_.snapshot.recent_centered_terminal &&
          cp.recent_centered_hold_through_s > 0.0F &&
          latch_age_s < static_cast<double>(cp.recent_centered_hold_through_s);
      if (!min_latch_active && !recent_centered_hold_through_active) {
        if (log_sink_) {
          log_sink_->logThrottled(75,
              "[STRIKE][commit] exit reason=live_terminal_alignment_lost "
              "latch_age_s=%.3f bbox_ratio=%.3f "
              "align_err=(x=%.1f,y=%.1f) limit=(x=%.1f,y=%.1f)",
              latch_age_s, static_cast<double>(bbox_area_ratio),
              static_cast<double>(align_error_x_px),
              static_cast<double>(align_error_y_px),
              static_cast<double>(cp.terminal_max_error_x_px),
              static_cast<double>(cp.terminal_max_error_y_px));
        }
        commit_state_.active = false;
        commit_state_.terminal_ready = false;
        commit_state_.latch_start_time_ns.reset();
        commit_state_.align_since_ns.reset();
      } else if (min_latch_active && log_sink_) {
        log_sink_->logThrottled(75,
            "[STRIKE][commit] continue reason=min_latch "
            "latch_age_s=%.3f latch_limit_s=%.3f bbox_ratio=%.3f",
            latch_age_s, static_cast<double>(cp.min_latch_s),
            static_cast<double>(bbox_area_ratio));
      }
    }

    if (commit_state_.active && commit_state_.snapshot.blind_terminal &&
        measure_reliable) {
      const double latch_age_s =
          commit_state_.latch_start_time_ns.has_value()
              ? circle::types::secondsBetween(
                    *commit_state_.latch_start_time_ns, inputs.now_ns)
              : std::numeric_limits<double>::infinity();
      const bool min_latch_active =
          cp.min_latch_s > 0.0F &&
          latch_age_s < static_cast<double>(cp.min_latch_s);
      const bool recent_centered_hold_through_active =
          commit_state_.snapshot.recent_centered_terminal &&
          cp.recent_centered_hold_through_s > 0.0F &&
          latch_age_s < static_cast<double>(cp.recent_centered_hold_through_s);

      const bool current_area_ok =
          cp.terminal_min_area_ratio <= 0.0F ||
          bbox_area_ratio >= cp.terminal_min_area_ratio;
      const bool soft_x_ok = cp.terminal_max_error_x_px <= 0.0F ||
          align_error_x_px <= cp.terminal_max_error_x_px * 1.35F;
      const bool soft_y_ok = cp.terminal_max_error_y_px <= 0.0F ||
          align_error_y_px <= cp.terminal_max_error_y_px * 1.35F;

      const float hard_edge_margin_y_px =
          cp.blind_commit_edge_margin_y_px > 1.0F
              ? std::min(12.0F, cp.blind_commit_edge_margin_y_px * 0.15F)
              : 0.0F;
      const float hard_edge_margin_x_px =
          cp.blind_commit_edge_margin_x_px > 1.0F
              ? std::min(12.0F, cp.blind_commit_edge_margin_x_px * 0.15F)
              : 0.0F;
      const bool large_hard_edge =
          bbox_area_ratio >= std::max(0.020F, cp.terminal_min_area_ratio * 12.0F) &&
          (cp.snapshot_max_area_ratio <= 0.0F ||
           bbox_area_ratio <= cp.snapshot_max_area_ratio) &&
          ((cp.blind_commit_edge_margin_x_px > 1.0F &&
            bbox_margin_x_px <= hard_edge_margin_x_px) ||
           (cp.blind_commit_edge_margin_y_px > 1.0F &&
            bbox_margin_y_px <= hard_edge_margin_y_px));

      const bool alignment_lost =
          !current_area_ok &&
          (!soft_x_ok || !soft_y_ok) &&
          !large_hard_edge;

      if (alignment_lost && !min_latch_active &&
          !recent_centered_hold_through_active) {
        if (log_sink_) {
          log_sink_->logThrottled(75,
              "[STRIKE][commit] exit reason=blind_alignment_lost "
              "latch_age_s=%.3f bbox_ratio=%.3f "
              "align_err=(x=%.1f,y=%.1f) soft_ok=(x=%d,y=%d) area_ok=%d "
              "hard_edge=%d",
              latch_age_s, static_cast<double>(bbox_area_ratio),
              static_cast<double>(align_error_x_px),
              static_cast<double>(align_error_y_px),
              static_cast<int>(soft_x_ok), static_cast<int>(soft_y_ok),
              static_cast<int>(current_area_ok),
              static_cast<int>(large_hard_edge));
        }
        commit_state_.active = false;
        commit_state_.terminal_ready = false;
        commit_state_.snapshot = FinalApproachCommitSnapshot{};
        commit_state_.recent_centered_snapshot = FinalApproachCommitSnapshot{};
        commit_state_.latch_start_time_ns.reset();
        commit_state_.align_since_ns.reset();
      } else if (min_latch_active && log_sink_) {
        log_sink_->logThrottled(75,
            "[STRIKE][commit] continue reason=min_latch(blind) "
            "latch_age_s=%.3f latch_limit_s=%.3f bbox_ratio=%.3f",
            latch_age_s, static_cast<double>(cp.min_latch_s),
            static_cast<double>(bbox_area_ratio));
      }
    }

    // Dive and commit are mutually exclusive: dive when detection is fresh,
    // commit hold as fallback when detection is stale.
    if (commit_state_.active && !dive_output.active) {
      auto commit_output = commit_module_.computeHold(
          commit_state_, cp, params_.thrust, params_.thrust.hover_scalar,
          params_.lateral_output_sign, params_.longitudinal_output_sign,
          vehicle.roll_rad, vehicle.pitch_rad,
          params_.tilt_cap.max_roll_angle_rad,
          params_.tilt_cap.max_pitch_angle_rad,
          params_.tilt_cap.softcap_band_rad,
          params_.yaw.lock_enabled, detection_age_s, inputs.now_ns);
      if (commit_output.should_hold) {
        out.rates.roll_rate_rad_s = commit_output.roll_rate_rad_s;
        out.rates.pitch_rate_rad_s = commit_output.pitch_rate_rad_s;
        out.rates.yaw_rate_rad_s = commit_output.yaw_rate_rad_s;
        out.rates.thrust_z = commit_output.thrust_z;
        // Stay in Tracking state - commit hold runs within the Tracking branch.
        // Setting state_ to CommitHold would cause the next tick to fall into
        // the WaitingTarget else-branch, clearing all commit state immediately.
      }
    }
    
    // 11.3: Cycle log
    if (log_sink_) {
      log_sink_->logThrottled(
          final_approach_active_ ? 75 : 200,
          "[STRIKE][cycle] ex=%.3f ey=%.3f ex_dot=%.3f ey_dot=%.3f "
          "roll=%.1fdeg pitch=%.1fdeg "
          "roll_rate_sp=%.2f pitch_rate_sp=%.2f yaw_rate_sp=%.2f "
          "softcap=(r=%.2f,p=%.2f) fa_level=(r=%.2f,p=%.2f) "
          "e_rho=%.3f e_rho_dot=%.3f rho_scale=%.2f bbox_ratio=%.5f "
          "bbox=(%.1f,%.1f %.1fx%.1f) desired_area=%.1f "
          "png=(active=%d,ff=%.2f/%.2f,trim=%.2f/%.2f,closure=%.2f,losdot=%.2f/%.2f) "
          "fa=%d commit=%d fallback=%d conf=%d score=%.2f "
          "speed_gov=(active=%d,blend=%.2f,scale=%.2f) "
          "preclimb=(active=%d,released=%d,scale=%.2f) "
          "thrust_z=%.3f thrust_s_tgt=%.3f thrust_s_smooth=%.3f "
          "max_edge_ratio=%.3f dive_blend=%.3f "
          "gates=%s",
          static_cast<double>(out.image_ex), static_cast<double>(out.image_ey),
          static_cast<double>(ex_dot_filt_), static_cast<double>(ey_dot_filt_),
          static_cast<double>(vehicle.roll_rad) * 180.0 / M_PI,
          static_cast<double>(vehicle.pitch_rad) * 180.0 / M_PI,
          static_cast<double>(out.rates.roll_rate_rad_s),
          static_cast<double>(out.rates.pitch_rate_rad_s),
          static_cast<double>(out.rates.yaw_rate_rad_s),
          static_cast<double>(roll_softcap_factor),
          static_cast<double>(pitch_softcap_factor),
          static_cast<double>(fa_roll_level_blend),
          static_cast<double>(fa_pitch_level_blend),
          static_cast<double>(e_rho), static_cast<double>(e_rho_dot_filt_),
          static_cast<double>(rho_scale),
          static_cast<double>(bbox_area_ratio),
          static_cast<double>(inputs.detection.detection.cx),
          static_cast<double>(inputs.detection.detection.cy),
          static_cast<double>(inputs.detection.detection.width),
          static_cast<double>(inputs.detection.detection.height),
          static_cast<double>(params_.rho_scale.desired_bbox_area_px),
          static_cast<int>(png_output.active),
          static_cast<double>(png_output.roll_png_ff_rad_s),
          static_cast<double>(png_output.pitch_png_ff_rad_s),
          static_cast<double>(png_output.roll_trim_rad_s),
          static_cast<double>(png_output.pitch_trim_rad_s),
          static_cast<double>(png_output.closure_scale),
          static_cast<double>(png_output.ex_dot_inertial),
          static_cast<double>(png_output.ey_dot_inertial),
          static_cast<int>(final_approach_active_),
          static_cast<int>(commit_state_.active),
          static_cast<int>(tracker_fallback_active_),
          static_cast<int>(strike_confident_),
          static_cast<double>(detection_score),
          static_cast<int>(gov_output.active),
          static_cast<double>(gov_output.blend),
          static_cast<double>(gov_output.scale),
          static_cast<int>(preclimb_output.xy_gate_active),
          static_cast<int>(preclimb_state_.xy_gate_released),
          static_cast<double>(preclimb_output.thrust_scale),
          static_cast<double>(out.rates.thrust_z),
          static_cast<double>(thrust_output.base_target),
          static_cast<double>(thrust_output.thrust_scalar_smooth),
          static_cast<double>(max_edge_ratio),
          static_cast<double>(dive_output.dive_blend),
          fa_gate_output.gate_reason);
    }
    
    out.telemetry.valid = true;
    out.telemetry.coasting = false;
    out.telemetry.final_approach_active = final_approach_active_;
    out.telemetry.roll_rate_sp_rad_s = out.rates.roll_rate_rad_s;
    out.telemetry.pitch_rate_sp_rad_s = out.rates.pitch_rate_rad_s;
    out.telemetry.yaw_rate_sp_rad_s = out.rates.yaw_rate_rad_s;
    out.telemetry.thrust_z = out.rates.thrust_z;
    out.telemetry.ex = out.image_ex;
    out.telemetry.ey = out.image_ey;
    out.telemetry.ex_dot_filt = ex_dot_filt_;
    out.telemetry.ey_dot_filt = ey_dot_filt_;
    out.telemetry.e_rho = e_rho;
    out.telemetry.e_rho_dot_filt = e_rho_dot_filt_;
    out.telemetry.rho_scale = rho_scale;
    out.telemetry.aim_comp_x_px = tilt_aim_comp_x_px;
    out.telemetry.aim_comp_y_px = tilt_aim_comp_y_px;
    out.telemetry.image_lead_x_px = image_lead_x_px;
    out.telemetry.image_lead_y_px = image_lead_y_px;
    out.telemetry.image_lead_active = image_lead_active;
    out.telemetry.terminal_predictor_active = terminal_predictor_output.active;
    out.telemetry.terminal_predictor_lead_x_px =
        terminal_predictor_output.lead_x_px;
    out.telemetry.terminal_predictor_lead_y_px =
        terminal_predictor_output.lead_y_px;
    out.telemetry.terminal_predictor_predicted_ex =
        terminal_predictor_output.predicted_ex;
    out.telemetry.terminal_predictor_predicted_ey =
        terminal_predictor_output.predicted_ey;
    out.telemetry.terminal_predictor_closing_rate =
        terminal_predictor_output.closing_rate;
    out.telemetry.png_active = png_output.active;
    out.telemetry.png_roll_ff_rad_s = png_output.roll_png_ff_rad_s;
    out.telemetry.png_pitch_ff_rad_s = png_output.pitch_png_ff_rad_s;
    out.telemetry.png_roll_trim_rad_s = png_output.roll_trim_rad_s;
    out.telemetry.png_pitch_trim_rad_s = png_output.pitch_trim_rad_s;
    out.telemetry.png_closure_scale = png_output.closure_scale;
    out.telemetry.png_ex_dot_inertial = png_output.ex_dot_inertial;
    out.telemetry.png_ey_dot_inertial = png_output.ey_dot_inertial;
    out.telemetry.fa_kp_scale = final_kp_scale;
    out.telemetry.fa_kd_scale = final_kd_scale;
    out.telemetry.fa_prox_norm = prox_norm;
    out.telemetry.fa_roll_level_blend = fa_roll_level_blend;
    out.telemetry.fa_pitch_level_blend = fa_pitch_level_blend;
    out.telemetry.detection_score = detection_score;
    out.telemetry.bbox_area_ratio = bbox_area_ratio;
    out.telemetry.deadband_eff_half_w_px = x_deadband_norm * inputs.detection.intrinsics.fx;
    out.telemetry.deadband_eff_half_h_px = y_deadband_norm * inputs.detection.intrinsics.fy;
    out.telemetry.tracker_fallback_active = tracker_fallback_active_;
    out.telemetry.strike_confident = strike_confident_;
    out.telemetry.state = static_cast<int>(StrikeState::Tracking);
    out.telemetry.guidance_mode = static_cast<int>(params_.guidance_mode);
    out.telemetry.roll_hard_headroom_rad = roll_hard_headroom_rad;
    out.telemetry.pitch_hard_headroom_rad = pitch_hard_headroom_rad;
    out.telemetry.roll_softcap_factor = roll_softcap_factor;
    out.telemetry.pitch_softcap_factor = pitch_softcap_factor;
    out.telemetry.edge_protect_active = edge_protect_active;
    out.telemetry.edge_taper_score = edge_taper_score;
    out.telemetry.bottom_pitch_guard_blend = bottom_pitch_guard_blend;
    out.telemetry.tracking_speed_governor_active = gov_output.active;
    out.telemetry.speed_governor_blend = gov_output.blend;
    out.telemetry.speed_governor_scale = gov_output.scale;
    out.telemetry.preclimb_xy_gate_active = preclimb_output.xy_gate_active;
    out.telemetry.preclimb_safe_hold_blend = preclimb_output.safe_hold_blend;
    out.telemetry.preclimb_xy_thrust_scale = preclimb_output.thrust_scale;
    out.telemetry.preclimb_release_slowdown_active = thrust_output.preclimb_release_slowdown_active;
    out.telemetry.preclimb_release_slowdown_scale = thrust_output.preclimb_release_slowdown_scale;
    out.telemetry.tracking_deadband_priority_active = thrust_output.tracking_deadband_priority_active;
    out.telemetry.tracking_deadband_priority_scale = thrust_output.tracking_deadband_priority_scale;
    out.telemetry.fa_vertical_drift_slowdown_active = thrust_output.fa_vertical_drift_slowdown_active;
    out.telemetry.fa_ascent_budget_active = thrust_output.fa_ascent_budget_active;
    out.telemetry.fa_ascent_budget_tilt_score = thrust_output.fa_ascent_budget_tilt_score;
    out.telemetry.fa_ascent_budget_y_rate_score = thrust_output.fa_ascent_budget_y_rate_score;
    out.telemetry.fa_ascent_budget_y_error_score = thrust_output.fa_ascent_budget_y_error_score;
    out.telemetry.fa_thrust_taper_scale = thrust_output.fa_thrust_taper_scale;
    out.telemetry.fa_unaligned_slowdown_scale = thrust_output.fa_unaligned_slowdown_scale;
    out.telemetry.fa_tilt_slowdown_scale = thrust_output.fa_tilt_slowdown_scale;
    out.telemetry.fa_vertical_drift_slowdown_scale = thrust_output.fa_vertical_drift_slowdown_scale;
    out.telemetry.ascent_image_velocity_damping_active = thrust_output.ascent_image_velocity_damping_active;
    out.telemetry.constant_thrust = thrust_output.base_target;
    out.telemetry.tracking_thrust_scalar_smooth = thrust_output.thrust_scalar_smooth;
    out.telemetry.tracking_thrust_scalar_target = thrust_output.base_target;
    
    // 11.5: LastVisualContactMetric (updated on fresh detection)
    if (fresh_detection) {
      auto& metric = out.telemetry.last_visual_contact;
      metric.valid = true;
      metric.stamp_ns = inputs.now_ns;
      metric.bbox_area_ratio = bbox_area_ratio;
      metric.detection_score = detection_score;
      metric.align_error_x_px = std::abs(out.image_ex * inputs.detection.intrinsics.fx);
      metric.align_error_y_px = std::abs(out.image_ey * inputs.detection.intrinsics.fy);
      metric.final_approach_active = final_approach_active_;
      if (vehicle.valid && vehicle.position_z_valid) {
        metric.vehicle_position_valid = true;
        metric.vehicle_altitude_m = -vehicle.position_ned_z;
      }
      if (vehicle.velocity_xy_valid) {
        metric.vehicle_velocity_valid = true;
        metric.vehicle_vxy_m_s = vehicle.velocity_xy_m_s;
      }
      if (params_.evaluation.target_altitude_enable && metric.vehicle_position_valid) {
        metric.target_altitude_valid = true;
        metric.target_altitude_m = params_.evaluation.target_altitude_m;
        metric.altitude_gap_to_target_m =
            params_.evaluation.target_altitude_m - metric.vehicle_altitude_m;
        metric.altitude_goal_met =
            std::fabs(metric.altitude_gap_to_target_m) <=
            params_.evaluation.success_altitude_gap_m;
      }
    }
    
    thrust_ramp_prev_publish_ = ThrustRampPrevPublish::TrackingPd;
    
    // 10.2: Check if we should activate FA fallback
    // Activate when FA is active but target is lost (simplified commit expiration)
    if (final_approach_active_ && !out.has_valid_target &&
        !fa_fallback_start_time_ns_.has_value()) {
      fa_fallback_start_time_ns_ = inputs.now_ns;
    }

  } else if (inputs.mode_active && vehicle.valid &&
             (commit_state_.active || commit_state_.terminal_ready ||
              commit_state_.snapshot.valid)) {
    if (!commit_state_.active && commit_state_.terminal_ready) {
      commit_state_.active = true;
      commit_state_.latch_start_time_ns = inputs.now_ns;
    }
    auto commit_output = commit_module_.computeHold(
        commit_state_, params_.final_approach.commit, params_.thrust,
        params_.thrust.hover_scalar, params_.lateral_output_sign,
        params_.longitudinal_output_sign, vehicle.roll_rad, vehicle.pitch_rad,
        params_.tilt_cap.max_roll_angle_rad,
        params_.tilt_cap.max_pitch_angle_rad,
        params_.tilt_cap.softcap_band_rad, params_.yaw.lock_enabled,
        detection_age_s, inputs.now_ns);
    if (commit_output.should_hold) {
      state_ = StrikeState::CommitHold;
      out.rates.roll_rate_rad_s = commit_output.roll_rate_rad_s;
      out.rates.pitch_rate_rad_s = commit_output.pitch_rate_rad_s;
      out.rates.yaw_rate_rad_s = commit_output.yaw_rate_rad_s;
      out.rates.thrust_z = commit_output.thrust_z;
      out.telemetry.valid = true;
      out.telemetry.coasting = true;
      out.telemetry.final_approach_commit_active = true;
      out.telemetry.roll_rate_sp_rad_s = out.rates.roll_rate_rad_s;
      out.telemetry.pitch_rate_sp_rad_s = out.rates.pitch_rate_rad_s;
      out.telemetry.yaw_rate_sp_rad_s = out.rates.yaw_rate_rad_s;
      out.telemetry.thrust_z = out.rates.thrust_z;
      out.telemetry.constant_thrust = params_.final_approach.commit.thrust_scalar;
      out.telemetry.state = static_cast<int>(StrikeState::CommitHold);
      thrust_ramp_prev_publish_ = ThrustRampPrevPublish::CommitHold;
    } else {
      commit_state_.active = false;
      if (final_approach_active_ && !fa_fallback_start_time_ns_.has_value()) {
        fa_fallback_start_time_ns_ = inputs.now_ns;
      }
      state_ = StrikeState::WaitingTarget;
    }

  } else if ((fa_fallback_start_time_ns_.has_value() ||
              final_approach_active_ ||
              (fa_gate_state_.last_gate_time_ns.has_value() &&
               circle::types::secondsBetween(
                   *fa_gate_state_.last_gate_time_ns, inputs.now_ns) <=
                   static_cast<double>(params_.final_approach.fallback.max_s))) &&
             inputs.mode_active && vehicle.valid) {
    if (!fa_fallback_start_time_ns_.has_value()) {
      fa_fallback_start_time_ns_ = inputs.now_ns;
    }
    const double elapsed_s = circle::types::secondsBetween(
        *fa_fallback_start_time_ns_, inputs.now_ns);
    
    if (elapsed_s > static_cast<double>(params_.final_approach.fallback.max_s)) {
      fa_fallback_start_time_ns_.reset();
      state_ = StrikeState::WaitingTarget;
    } else {
      state_ = StrikeState::FaFallback;
      
      const float decay_tau = std::max(
          params_.final_approach.fallback.decay_tau_s,
          params_.rate_lpf_tau_s);
      
      const float roll_err = applyDeadband(
          vehicle.roll_rad, params_.waiting.level_deadband_rad);
      const float pitch_err = applyDeadband(
          vehicle.pitch_rad - params_.final_approach.fallback.pitch_bias_rad,
          params_.waiting.level_deadband_rad);
      const float roll_rate_des = -params_.waiting.level_kp * roll_err;
      const float pitch_rate_des = -params_.waiting.level_kp * pitch_err;
      
      if (!waiting_yaw_ref_.has_value()) {
        waiting_yaw_ref_ = vehicle.yaw_rad;
      }
      const float heading_err_raw =
          wrapAngle(waiting_yaw_ref_.value_or(vehicle.yaw_rad) - vehicle.yaw_rad);
      float heading_err = heading_err_raw;
      if (std::fabs(heading_err) < params_.yaw.hold_deadband_rad) {
        heading_err = 0.0F;
      }
      const float yaw_rate_target = std::clamp(
          params_.yaw.hold_gain * heading_err,
          params_.yaw.rate_min_rad_s, params_.yaw.rate_max_rad_s);
      
      auto rate_output = rate_shaper_.compute(
          roll_rate_des, pitch_rate_des, yaw_rate_target,
          vehicle.roll_rad, vehicle.pitch_rad,
          params_.tilt_cap.max_roll_angle_rad, params_.tilt_cap.max_pitch_angle_rad,
          params_.tilt_cap.softcap_band_rad,
          decay_tau, params_.max_jerk_rad_s2,
          params_.max_roll_rate_rad_s, params_.max_pitch_rate_rad_s,
          params_.yaw.rate_lpf_tau_s, true,
          1.0F, 1.0F, 1.0F, safe_dt, safe_dt, false);
      
      out.rates.roll_rate_rad_s = rate_output.roll_rate_rad_s;
      out.rates.pitch_rate_rad_s = rate_output.pitch_rate_rad_s;
      out.rates.yaw_rate_rad_s = rate_output.yaw_rate_rad_s;
      
      const float tilt_denom_fb = params_.thrust.enable_tilt_compensation
          ? std::max(std::cos(vehicle.pitch_rad) * std::cos(vehicle.roll_rad),
                     params_.thrust.tilt_cos_floor)
          : 1.0F;
      const float fallback_thrust_s = std::min(
          params_.final_approach.commit.thrust_scalar / tilt_denom_fb,
          params_.final_approach.commit.thrust_scalar);
      out.rates.thrust_z = clampThrustScalar(
          fallback_thrust_s, params_.thrust.scalar_min, params_.thrust.scalar_max);
      
      out.telemetry.valid = true;
      out.telemetry.coasting = true;
      out.telemetry.final_approach_active = false;
      out.telemetry.roll_rate_sp_rad_s = out.rates.roll_rate_rad_s;
      out.telemetry.pitch_rate_sp_rad_s = out.rates.pitch_rate_rad_s;
      out.telemetry.yaw_rate_sp_rad_s = out.rates.yaw_rate_rad_s;
      out.telemetry.thrust_z = out.rates.thrust_z;
      out.telemetry.constant_thrust = params_.final_approach.commit.thrust_scalar;
      out.telemetry.rho_scale = 1.0F;
      out.telemetry.state = static_cast<int>(StrikeState::FaFallback);
      out.telemetry.roll_hard_headroom_rad = roll_hard_headroom_rad;
      out.telemetry.pitch_hard_headroom_rad = pitch_hard_headroom_rad;
      thrust_ramp_prev_publish_ = ThrustRampPrevPublish::FaFallback;
    }

  } else {
    const bool entering_waiting =
        (state_ != StrikeState::WaitingTarget &&
         state_ != StrikeState::ForceLevel);

    if (entering_waiting) {
      commit_state_.active = false;
      commit_state_.terminal_ready = false;
      commit_state_.snapshot = FinalApproachCommitSnapshot{};
      commit_state_.recent_centered_snapshot = FinalApproachCommitSnapshot{};
      commit_state_.latch_start_time_ns.reset();
      commit_state_.align_since_ns.reset();
      fa_fallback_start_time_ns_.reset();
    }

    if (params_.waiting.level_hold_enabled && inputs.mode_active && vehicle.valid) {
      const float roll_err =
          applyDeadband(vehicle.roll_rad, params_.waiting.level_deadband_rad);
      const float pitch_err =
          applyDeadband(vehicle.pitch_rad, params_.waiting.level_deadband_rad);
      const float roll_rate_des = -params_.waiting.level_kp * roll_err;
      const float pitch_rate_des = -params_.waiting.level_kp * pitch_err;

      if (entering_waiting || !waiting_yaw_ref_.has_value()) {
        waiting_yaw_ref_ = vehicle.yaw_rad;
      }
      const float heading_err_raw =
          wrapAngle(waiting_yaw_ref_.value_or(vehicle.yaw_rad) - vehicle.yaw_rad);
      float heading_err = heading_err_raw;
      if (std::fabs(heading_err) < params_.yaw.hold_deadband_rad) {
        heading_err = 0.0F;
      }
      const float yaw_rate_target = std::clamp(
          params_.yaw.hold_gain * heading_err,
          params_.yaw.rate_min_rad_s, params_.yaw.rate_max_rad_s);

      const float decay_tau =
          std::max(params_.rate_lpf_tau_s,
                   params_.target_loss.lost_target_rate_decay_tau_s);

      auto rate_output = rate_shaper_.compute(
          roll_rate_des, pitch_rate_des, yaw_rate_target,
          vehicle.roll_rad, vehicle.pitch_rad,
          params_.tilt_cap.max_roll_angle_rad, params_.tilt_cap.max_pitch_angle_rad,
          params_.tilt_cap.softcap_band_rad,
          decay_tau, params_.max_jerk_rad_s2,
          params_.max_roll_rate_rad_s, params_.max_pitch_rate_rad_s,
          params_.yaw.rate_lpf_tau_s, false,
          1.0F, 1.0F, 1.0F, safe_dt, safe_dt, false);

      out.rates.roll_rate_rad_s = rate_output.roll_rate_rad_s;
      out.rates.pitch_rate_rad_s = rate_output.pitch_rate_rad_s;
      out.rates.yaw_rate_rad_s = rate_output.yaw_rate_rad_s;
    }

    float ref_ned = 0.0F, err_ned = 0.0F, corr = 0.0F;
    out.rates.thrust_z = computeWaitingThrustZ(
        entering_waiting, vehicle, ref_ned, err_ned, corr);

    state_ = StrikeState::WaitingTarget;

    out.telemetry.valid = true;
    out.telemetry.coasting = true;
    out.telemetry.final_approach_active = false;
    out.telemetry.roll_rate_sp_rad_s = out.rates.roll_rate_rad_s;
    out.telemetry.pitch_rate_sp_rad_s = out.rates.pitch_rate_rad_s;
    out.telemetry.yaw_rate_sp_rad_s = out.rates.yaw_rate_rad_s;
    out.telemetry.thrust_z = out.rates.thrust_z;
    out.telemetry.constant_thrust = params_.thrust.hover_scalar;
    out.telemetry.rho_scale = 1.0F;
    out.telemetry.state = static_cast<int>(StrikeState::WaitingTarget);
    out.telemetry.waiting_altitude_ref_ned = ref_ned;
    out.telemetry.waiting_altitude_error_ned = err_ned;
    out.telemetry.waiting_altitude_correction = corr;
    out.telemetry.roll_hard_headroom_rad = roll_hard_headroom_rad;
    out.telemetry.pitch_hard_headroom_rad = pitch_hard_headroom_rad;
    thrust_ramp_prev_publish_ = ThrustRampPrevPublish::Waiting;

  }

  out.state = state_;

  if (params_.target_loss.complete_on_loss_enable &&
      target_seen_this_activation_ && !target_loss_completed_) {
    const double target_lost_s =
        last_target_seen_time_ns_.has_value()
            ? circle::types::secondsBetween(
                  *last_target_seen_time_ns_, inputs.now_ns)
            : std::numeric_limits<double>::infinity();
    bool altitude_gate_ok = true;
    if (params_.target_loss.complete_altitude_gate_enable) {
      altitude_gate_ok = false;
      if (vehicle.valid && vehicle.position_z_valid &&
          params_.evaluation.target_altitude_enable) {
        const float vehicle_altitude_m = -vehicle.position_ned_z;
        const float altitude_gap_m =
            std::fabs(params_.evaluation.target_altitude_m - vehicle_altitude_m);
        altitude_gate_ok =
            altitude_gap_m <= params_.target_loss.complete_max_altitude_gap_m;
      }
    }
    if (target_lost_s >= static_cast<double>(params_.target_loss.loss_complete_s) &&
        altitude_gate_ok) {
      target_loss_completed_ = true;
      state_ = StrikeState::Complete;
      out.state = StrikeState::Complete;
      out.telemetry.state = static_cast<int>(StrikeState::Complete);
    }
  }

  const bool may_command =
      inputs.mode_active &&
      (!out.safety.require_armed_to_command || out.safety.armed);

  if (!may_command && params_.dry_run && state_ != StrikeState::Tracking) {
    out.rates.thrust_z = 0.0F;
  } else if (!may_command && !params_.dry_run) {
    out.rates.thrust_z = 0.0F;
  }

  if (rate_sink_ && may_command && !params_.dry_run) {
    rate_sink_->publishRates(out.rates, out.safety);
  }

  return out;
}

}  // namespace circle::strike
