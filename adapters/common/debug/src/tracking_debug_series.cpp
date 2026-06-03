#include "circle/debug/tracking_debug_schema.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace circle::debug {

namespace {
int queryGetInt(const std::string& q, const char* key, int fallback) {
  const std::string pat = std::string(key) + "=";
  size_t p = q.find(pat);
  if (p == std::string::npos) return fallback;
  p += pat.size();
  int v = 0;
  bool any = false;
  while (p < q.size() && std::isdigit(static_cast<unsigned char>(q[p]))) {
    any = true;
    v = v * 10 + (q[p] - '0');
    ++p;
  }
  return any ? v : fallback;
}

double queryGetDouble(const std::string& q, const char* key, double fallback) {
  const std::string pat = std::string(key) + "=";
  size_t p = q.find(pat);
  if (p == std::string::npos) return fallback;
  p += pat.size();
  char* end = nullptr;
  double v = std::strtod(q.c_str() + p, &end);
  if (end == q.c_str() + p) return fallback;
  return v;
}

std::string queryGetString(const std::string& q, const char* key,
                           const std::string& fallback) {
  const std::string pat = std::string(key) + "=";
  size_t p = q.find(pat);
  if (p == std::string::npos) return fallback;
  p += pat.size();
  const size_t amp = q.find('&', p);
  if (amp == std::string::npos) {
    return q.substr(p);
  }
  return q.substr(p, amp - p);
}

}  // namespace

std::string makeVisionSeriesJson(const std::string& query,
                                 const std::vector<VisionPlotSample>& samples,
                                 const std::string& tune_target_node,
                                 int default_max_points,
                                 double default_window_s,
                                 std::optional<float> live_px4_thrust_z) {
  const int max_pts =
      std::max(32, queryGetInt(query, "max_points", default_max_points));
  const double window_s =
      std::max(1.0, queryGetDouble(query, "window_s", default_window_s));
  const int digits = std::clamp(queryGetInt(query, "digits", 5), 3, 8);

  std::string live_px4_thrust_json;
  if (live_px4_thrust_z.has_value() && std::isfinite(*live_px4_thrust_z)) {
    std::ostringstream a;
    a << std::setprecision(digits) << *live_px4_thrust_z;
    live_px4_thrust_json = ",\"live_px4_thrust_z\":" + a.str();
  }

  const TuneMode tune_mode = detectTuneMode(tune_target_node);
  const bool tune_top_cam = (tune_mode == TuneMode::Vertical ||
                             tune_mode == TuneMode::RatesCtrl ||
                             tune_mode == TuneMode::TargetStrike ||
                             tune_mode == TuneMode::TargetStrikePng);
  const bool is_png = (tune_mode == TuneMode::TargetStrikePng);
  const bool emit_backend =
      (tune_mode == TuneMode::TargetStrike || is_png);

  std::vector<VisionPlotSample> snap;
  if (samples.empty()) {
    return std::string("{\"ok\":true,\"tune_mode\":\"") +
           tuneModeKey(tune_mode) + "\",\"n\":0" +
           (emit_backend ? std::string(",\"strike_backend\":\"px4\"")
                         : std::string()) +
           live_px4_thrust_json + ",\"t\":[]}\n";
  }
  const double t_end = samples.back().t_sec;
  const double t_min = t_end - window_s;
  for (const auto& s : samples) {
    if (s.t_sec >= t_min) snap.push_back(s);
  }
  if (snap.size() > static_cast<size_t>(max_pts)) {
    snap.erase(snap.begin(), snap.end() - max_pts);
  }
  const size_t n = snap.size();
  auto fmt_vec = [&snap, digits](float VisionPlotSample::*member_ptr) {
    std::ostringstream a;
    a << '[';
    for (size_t i = 0; i < snap.size(); ++i) {
      if (i) a << ',';
      float v = snap[i].*member_ptr;
      if (std::isfinite(v)) {
        a << std::setprecision(digits) << v;
      } else {
        a << "null";
      }
    }
    a << ']';
    return a.str();
  };
  auto fmt_scalar = [digits](float v) {
    std::ostringstream a;
    if (std::isfinite(v)) {
      a << std::setprecision(digits) << v;
    } else {
      a << "null";
    }
    return a.str();
  };
  const double t_base = snap.front().t_sec;
  std::string groups = queryGetString(query, "groups", "");
  for (char& ch : groups) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }

  std::ostringstream oss;
  oss << std::setprecision(digits);
  oss << "{\"ok\":true,\"tune_mode\":\"" << tuneModeKey(tune_mode)
      << "\",\"n\":" << n;
  if (emit_backend) {
    oss << ",\"strike_backend\":\"px4\"";
  }
  oss << ",\"t\":[";
  for (size_t i = 0; i < n; ++i) {
    if (i) oss << ',';
    oss << (snap[i].t_sec - t_base);
  }
  oss << ']';

  auto append_coasting = [&]() {
    oss << ",\"vision_coasting\":[";
    for (size_t i = 0; i < n; ++i) {
      if (i) oss << ',';
      oss << (snap[i].coasting ? 1 : 0);
    }
    oss << ']';
  };

  auto append_thrust_block = [&]() {
    oss << ",\"vision_area_valid\":[";
    for (size_t i = 0; i < n; ++i) {
      if (i) oss << ',';
      oss << (snap[i].area_thrust_valid ? 1 : 0);
    }
    oss << "],\"vision_tracking_thrust\":"
        << fmt_vec(&VisionPlotSample::tracking_thrust)
        << ",\"vision_thrust_target\":"
        << fmt_vec(&VisionPlotSample::thrust_target)
        << ",\"vision_thrust_floor\":"
        << fmt_vec(&VisionPlotSample::thrust_floor)
        << ",\"vision_filtered_rho_dot\":"
        << fmt_vec(&VisionPlotSample::filtered_rho_dot)
        << ",\"vision_rho_dot\":"
        << fmt_vec(&VisionPlotSample::rho_dot)
        << ",\"vision_filtered_ey_thrust\":"
        << fmt_vec(&VisionPlotSample::filtered_ey_thrust)
        << ",\"vision_high_thrust_s\":"
        << fmt_vec(&VisionPlotSample::high_thrust_duration_s)
        << ",\"vision_alt_vz_command_ned\":"
        << fmt_vec(&VisionPlotSample::alt_vz_cmd)
        << ",\"vision_alt_vz_error_ned\":"
        << fmt_vec(&VisionPlotSample::alt_vz_err)
        << ",\"vision_alt_d_term\":"
        << fmt_vec(&VisionPlotSample::alt_d)
        << ",\"vision_tilt_hard_headroom_roll_rad\":"
        << fmt_vec(&VisionPlotSample::tilt_hard_headroom_roll_rad)
        << ",\"vision_tilt_hard_headroom_pitch_rad\":"
        << fmt_vec(&VisionPlotSample::tilt_hard_headroom_pitch_rad);
  };

  auto append_bool_vec = [&](const char* name, bool VisionPlotSample::*m) {
    oss << ",\"" << name << "\":[";
    for (size_t i = 0; i < n; ++i) {
      if (i) oss << ',';
      oss << (snap[i].*m ? 1 : 0);
    }
    oss << ']';
  };

  // Unified target_strike_png series schema: same png_* keys emitted by the BF
  // SHM path (strike_telemetry_shm seriesJson). PX4 fills actual vehicle rates;
  // BF emits them as null. plots.html consumes one target_strike_png layout for
  // both backends, hiding null/missing traces automatically.
  auto append_png_block = [&]() {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp)
        << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z)
        << ",\"bbox_area_ratio\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"detection_score\":" << fmt_vec(&VisionPlotSample::detection_score)
        << ",\"png_closure_scale\":" << fmt_vec(&VisionPlotSample::png_closure_scale)
        << ",\"png_ex_dot_inertial\":"
        << fmt_vec(&VisionPlotSample::png_ex_dot_inertial)
        << ",\"png_ey_dot_inertial\":"
        << fmt_vec(&VisionPlotSample::png_ey_dot_inertial)
        << ",\"png_measurement_age_s\":"
        << fmt_vec(&VisionPlotSample::png_measurement_age_s)
        << ",\"png_ff_roll_rad_s\":" << fmt_vec(&VisionPlotSample::png_ff_roll_rad_s)
        << ",\"png_ff_pitch_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_ff_pitch_rad_s)
        << ",\"png_fov_trim_roll_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_fov_trim_roll_rad_s)
        << ",\"png_fov_trim_pitch_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_fov_trim_pitch_rad_s)
        << ",\"png_edge_guard_roll_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_edge_guard_roll_rad_s)
        << ",\"png_edge_guard_pitch_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_edge_guard_pitch_rad_s)
        << ",\"png_pursuit_roll_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_pursuit_roll_rad_s)
        << ",\"png_pursuit_pitch_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_pursuit_pitch_rad_s)
        << ",\"png_stale_trim_roll_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_stale_trim_roll_rad_s)
        << ",\"png_intercept_roll_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_intercept_roll_rad_s)
        << ",\"png_intercept_pitch_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_intercept_pitch_rad_s)
        << ",\"png_crossing_pitch_rad_s\":"
        << fmt_vec(&VisionPlotSample::png_crossing_pitch_rad_s)
        << ",\"png_future_ex\":" << fmt_vec(&VisionPlotSample::png_future_ex)
        << ",\"png_future_ey\":" << fmt_vec(&VisionPlotSample::png_future_ey)
        << ",\"png_intercept_lead_s\":"
        << fmt_vec(&VisionPlotSample::png_intercept_lead_s)
        << ",\"png_crossing_weight\":"
        << fmt_vec(&VisionPlotSample::png_crossing_weight)
        << ",\"png_fwd_guard_scale\":"
        << fmt_vec(&VisionPlotSample::png_fwd_guard_scale)
        << ",\"png_entry_handoff_progress\":"
        << fmt_vec(&VisionPlotSample::png_entry_handoff_progress)
        << ",\"png_tilt_softcap_roll\":"
        << fmt_vec(&VisionPlotSample::png_tilt_softcap_roll)
        << ",\"png_tilt_softcap_pitch\":"
        << fmt_vec(&VisionPlotSample::png_tilt_softcap_pitch);
    append_bool_vec("png_intercept_active",
                    &VisionPlotSample::png_intercept_active);
    append_bool_vec("png_crossing_active",
                    &VisionPlotSample::png_crossing_active);
    append_bool_vec("png_fwd_guard_active",
                    &VisionPlotSample::png_fwd_guard_active);
    append_bool_vec("png_loss_hold_latched",
                    &VisionPlotSample::png_loss_hold_latched);
    append_bool_vec("png_tilt_hardcap_active",
                    &VisionPlotSample::png_tilt_hardcap_active);
    append_bool_vec("detection_valid", &VisionPlotSample::detection_valid);
  };

  auto append_vehicle_attitude = [&]() {
    oss << ",\"vehicle_roll_rad\":"
        << fmt_vec(&VisionPlotSample::vehicle_roll_rad)
        << ",\"vehicle_pitch_rad\":"
        << fmt_vec(&VisionPlotSample::vehicle_pitch_rad)
        << ",\"vehicle_yaw_rad\":"
        << fmt_vec(&VisionPlotSample::vehicle_yaw_rad)
        << ",\"vehicle_roll_rate_rad_s\":"
        << fmt_vec(&VisionPlotSample::vehicle_roll_rate_rad_s)
        << ",\"vehicle_pitch_rate_rad_s\":"
        << fmt_vec(&VisionPlotSample::vehicle_pitch_rate_rad_s)
        << ",\"vehicle_yaw_rate_rad_s\":"
        << fmt_vec(&VisionPlotSample::vehicle_yaw_rate_rad_s);
  };

  auto append_plot_page_top_cam = [&]() {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_ex_raw\":" << fmt_vec(&VisionPlotSample::ex_raw)
        << ",\"vision_ey_raw\":" << fmt_vec(&VisionPlotSample::ey_raw)
        << ",\"vision_filtered_rho_dot\":"
        << fmt_vec(&VisionPlotSample::filtered_rho_dot)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp)
        << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z)
        << ",\"vision_tracking_thrust\":"
        << fmt_vec(&VisionPlotSample::tracking_thrust)
        << ",\"vision_thrust_target\":"
        << fmt_vec(&VisionPlotSample::thrust_target)
        << ",\"vision_alt_d_term\":"
        << fmt_vec(&VisionPlotSample::alt_d)
        << ",\"vision_alt_vz_command_ned\":"
        << fmt_vec(&VisionPlotSample::alt_vz_cmd)
        << ",\"vision_alt_vz_error_ned\":"
        << fmt_vec(&VisionPlotSample::alt_vz_err)
        << ",\"vision_tilt_hard_headroom_roll_rad\":"
        << fmt_vec(&VisionPlotSample::tilt_hard_headroom_roll_rad)
        << ",\"vision_tilt_hard_headroom_pitch_rad\":"
        << fmt_vec(&VisionPlotSample::tilt_hard_headroom_pitch_rad);
    append_vehicle_attitude();
  };

  if (is_png) {
    // PNG uses a single unified layout regardless of the requested group; the
    // component decomposition keys are always emitted and plots.html toggles
    // visibility per chart.
    append_png_block();
    append_coasting();
    append_vehicle_attitude();
  } else if (groups == "plot_page" && tune_top_cam) {
    append_plot_page_top_cam();
  } else if (groups == "plot_page") {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp)
        << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp);
    append_vehicle_attitude();
  } else if (groups.empty() || groups == "all" || groups == "full") {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_ex_raw\":" << fmt_vec(&VisionPlotSample::ex_raw)
        << ",\"vision_ey_raw\":" << fmt_vec(&VisionPlotSample::ey_raw)
        << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp)
        << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd)
        << ",\"vision_right_mps\":" << fmt_vec(&VisionPlotSample::right)
        << ",\"vision_down_mps\":" << fmt_vec(&VisionPlotSample::down)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp)
        << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z);
    append_coasting();
    append_thrust_block();
    append_vehicle_attitude();
  } else if (groups == "overview") {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_ex_raw\":" << fmt_vec(&VisionPlotSample::ex_raw)
        << ",\"vision_ey_raw\":" << fmt_vec(&VisionPlotSample::ey_raw)
        << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"vision_filtered_rho_dot\":"
        << fmt_vec(&VisionPlotSample::filtered_rho_dot)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp)
        << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd)
        << ",\"vision_right_mps\":" << fmt_vec(&VisionPlotSample::right)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp)
        << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z);
    append_coasting();
  } else if (groups == "yaw") {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp);
  } else if (groups == "range") {
    oss << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd);
  } else if (groups == "vertical") {
    oss << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_down_mps\":" << fmt_vec(&VisionPlotSample::down);
  } else if (groups == "attitude") {
    oss << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp);
  } else if (groups == "alt" || groups == "thrust") {
    oss << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z);
    append_thrust_block();
  } else if (groups == "forward_attitude") {
    if (tune_top_cam) {
      oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
          << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
          << ",\"vision_ex_raw\":" << fmt_vec(&VisionPlotSample::ex_raw)
          << ",\"vision_ey_raw\":" << fmt_vec(&VisionPlotSample::ey_raw)
          << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
          << ",\"vision_yaw_rate_sp_rad_s\":"
          << fmt_vec(&VisionPlotSample::yaw_rate_sp)
          << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
          << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp)
          << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z);
      append_thrust_block();
      append_vehicle_attitude();
    } else {
      oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
          << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
          << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
          << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd)
          << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
          << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp);
      append_vehicle_attitude();
    }
  } else {
    oss << ",\"vision_ex\":" << fmt_vec(&VisionPlotSample::ex)
        << ",\"vision_ey\":" << fmt_vec(&VisionPlotSample::ey)
        << ",\"vision_e_rho\":" << fmt_vec(&VisionPlotSample::e_rho)
        << ",\"vision_yaw_rate_sp_rad_s\":"
        << fmt_vec(&VisionPlotSample::yaw_rate_sp)
        << ",\"vision_fwd_mps\":" << fmt_vec(&VisionPlotSample::fwd)
        << ",\"vision_right_mps\":" << fmt_vec(&VisionPlotSample::right)
        << ",\"vision_down_mps\":" << fmt_vec(&VisionPlotSample::down)
        << ",\"vision_roll_sp_rad\":" << fmt_vec(&VisionPlotSample::roll_sp)
        << ",\"vision_pitch_sp_rad\":" << fmt_vec(&VisionPlotSample::pitch_sp)
        << ",\"vision_thrust_z\":" << fmt_vec(&VisionPlotSample::thrust_z);
    append_coasting();
    append_thrust_block();
  }

  if (n > 0) {
    const VisionPlotSample& latest = snap.back();
    size_t phase_start_idx = n - 1;
    while (phase_start_idx > 0 &&
           snap[phase_start_idx - 1].track_phase == latest.track_phase) {
      --phase_start_idx;
    }
    const double latest_phase_duration_s =
        latest.t_sec - snap[phase_start_idx].t_sec;
    const bool latest_phase_duration_lower_bound =
        (phase_start_idx == 0 && n > 1 &&
         snap.front().track_phase == latest.track_phase);
    oss << ",\"latest_tracking_thrust\":"
        << fmt_scalar(latest.tracking_thrust)
        << ",\"latest_thrust_target\":"
        << fmt_scalar(latest.thrust_target)
        << ",\"latest_thrust_z\":"
        << fmt_scalar(latest.thrust_z)
        << ",\"latest_actual_thrust\":"
        << fmt_scalar(latest.alt_d)
        << ",\"latest_waiting_altitude_ref_ned\":"
        << fmt_scalar(latest.alt_setpoint)
        << ",\"latest_waiting_altitude_error_ned\":"
        << fmt_scalar(latest.alt_pos_err)
        << ",\"latest_waiting_altitude_correction\":"
        << fmt_scalar(latest.alt_integ)
        << ",\"latest_phase\":"
        << static_cast<int>(latest.track_phase)
        << ",\"latest_phase_duration_s\":"
        << std::setprecision(digits) << latest_phase_duration_s
        << ",\"latest_phase_duration_lower_bound\":"
        << (latest_phase_duration_lower_bound ? "true" : "false");
    oss << ",\"track_phase\":[";
    for (size_t i = 0; i < n; ++i) {
      if (i) {
        oss << ',';
      }
      oss << static_cast<int>(snap[i].track_phase);
    }
    oss << ']';
  }

  oss << live_px4_thrust_json;
  oss << "}\n";
  return oss.str();
}

}  // namespace circle::debug
