#include "circle/debug/preview_overlay.hpp"

#include "circle/types/time.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace circle::debug {
namespace {

constexpr double kRad2Deg = 57.29577951308232;

cv::Scalar phaseColor(const PreviewOverlayContext& ctx) {
  if (!ctx.armed) {
    return cv::Scalar(136, 122, 109);
  }
  if (ctx.state == "Tracking") {
    return cv::Scalar(230, 124, 47);
  }
  if (ctx.state == "ForceLevel") {
    return cv::Scalar(69, 69, 230);
  }
  return cv::Scalar(102, 93, 90);
}

void drawImageCenterGuides(cv::Mat& img, const cv::Scalar& color, int thick,
                           float center_x, float center_y) {
  if (img.empty()) {
    return;
  }
  const int mx = static_cast<int>(std::lround(std::clamp(
      center_x, 0.0F, static_cast<float>(std::max(0, img.cols - 1)))));
  const int my = static_cast<int>(std::lround(std::clamp(
      center_y, 0.0F, static_cast<float>(std::max(0, img.rows - 1)))));
  const int arm = std::max(15, std::min(img.cols, img.rows) * 3 / 100);
  cv::line(img, cv::Point(mx - arm, my), cv::Point(mx + arm, my), color, thick,
           cv::LINE_AA);
  cv::line(img, cv::Point(mx, my - arm), cv::Point(mx, my + arm), color, thick,
           cv::LINE_AA);
}

void drawDeadbandBox(cv::Mat& img, float half_w_px, float half_h_px,
                     cv::Point2f center) {
  if (img.empty() || half_w_px <= 0.0F || half_h_px <= 0.0F) {
    return;
  }
  const int mx = static_cast<int>(std::lround(center.x));
  const int my = static_cast<int>(std::lround(center.y));
  const int hw = static_cast<int>(std::lround(half_w_px));
  const int hh = static_cast<int>(std::lround(half_h_px));
  const int x1 = std::clamp(mx - hw, 0, std::max(0, img.cols - 1));
  const int y1 = std::clamp(my - hh, 0, std::max(0, img.rows - 1));
  const int x2 = std::clamp(mx + hw, 0, std::max(0, img.cols - 1));
  const int y2 = std::clamp(my + hh, 0, std::max(0, img.rows - 1));
  cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2),
                cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
}

cv::Scalar colorForError(float mag_px, float deadband_w_px, float deadband_h_px) {
  const float deadband = std::max(1.0F, std::max(deadband_w_px, deadband_h_px));
  if (mag_px <= deadband) {
    return cv::Scalar(80, 220, 80);
  }
  if (mag_px <= deadband * 3.0F) {
    return cv::Scalar(0, 180, 255);
  }
  return cv::Scalar(0, 80, 255);
}

void drawTargetErrorArrow(cv::Mat& img, cv::Point2f from, cv::Point2f to,
                          float deadband_w_px, float deadband_h_px) {
  const cv::Point2f delta = to - from;
  const float mag = std::sqrt(delta.x * delta.x + delta.y * delta.y);
  const cv::Scalar color = colorForError(mag, deadband_w_px, deadband_h_px);
  cv::arrowedLine(img, from, to, color, 2, cv::LINE_AA, 0, 0.18);

  std::ostringstream ss;
  ss << "e=(" << static_cast<int>(std::lround(delta.x)) << ","
     << static_cast<int>(std::lround(delta.y)) << ")";
  const std::string label = ss.str();
  constexpr double kFontScale = 0.45;
  constexpr int kTextThick = 1;
  constexpr int kPad = 3;
  int baseline = 0;
  const cv::Size sz = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                      kFontScale, kTextThick, &baseline);
  cv::Point org(static_cast<int>(std::lround((from.x + to.x) * 0.5F)) + 4,
                static_cast<int>(std::lround((from.y + to.y) * 0.5F)) - 4);
  org.x = std::clamp(org.x, kPad + 1,
                     std::max(kPad + 1, img.cols - sz.width - kPad - 1));
  org.y = std::clamp(org.y, sz.height + kPad + 1,
                     std::max(sz.height + kPad + 1,
                              img.rows - baseline - kPad - 1));
  cv::Rect bg(org.x - kPad, org.y - sz.height - kPad,
              sz.width + kPad * 2, sz.height + baseline + kPad * 2);
  bg &= cv::Rect(0, 0, img.cols, img.rows);
  if (bg.width > 0 && bg.height > 0) {
    cv::rectangle(img, bg, color, cv::FILLED);
  }
  cv::putText(img, label, org, cv::FONT_HERSHEY_SIMPLEX, kFontScale,
              cv::Scalar(0, 0, 0), kTextThick, cv::LINE_AA);
}

void drawThrustBar(cv::Mat& img, float thrust_cmd, float thrust_ref) {
  if (img.empty()) {
    return;
  }
  const int bar_h = std::clamp(img.rows / 4, 70, 170);
  const int bar_w = 12;
  const int margin = 16;
  const int x = img.cols - margin - bar_w;
  const int y = img.rows - margin - bar_h;
  if (x < 0 || y < 0) {
    return;
  }
  cv::rectangle(img, cv::Rect(x, y, bar_w, bar_h), cv::Scalar(30, 30, 30), -1);
  cv::rectangle(img, cv::Rect(x, y, bar_w, bar_h), cv::Scalar(180, 180, 180), 1);
  const float cmd = std::clamp(thrust_cmd, 0.0F, 1.0F);
  const int fill_h = static_cast<int>(std::lround(cmd * static_cast<float>(bar_h - 2)));
  cv::rectangle(img, cv::Rect(x + 1, y + bar_h - 1 - fill_h, bar_w - 2, fill_h),
                cv::Scalar(70, 220, 120), -1);
  const int ref_y = y + bar_h -
      static_cast<int>(std::lround(std::clamp(thrust_ref, 0.0F, 1.0F) *
                                   static_cast<float>(bar_h)));
  cv::line(img, cv::Point(x - 5, ref_y), cv::Point(x + bar_w + 5, ref_y),
           cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
}

void drawInfoPanel(cv::Mat& img,
                   const std::vector<std::pair<std::string, cv::Scalar>>& lines,
                   double font_scale, int text_thick, int pad, int line_gap) {
  if (lines.empty()) {
    return;
  }
  int panel_w = 0;
  int text_stack_h = 0;
  int last_baseline = 0;
  for (const auto& line : lines) {
    int bl = 0;
    const cv::Size sz = cv::getTextSize(line.first, cv::FONT_HERSHEY_SIMPLEX,
                                        font_scale, text_thick, &bl);
    panel_w = std::max(panel_w, sz.width);
    text_stack_h += sz.height + line_gap;
    last_baseline = bl;
  }
  text_stack_h -= line_gap;
  panel_w += pad * 2;
  const int panel_h = pad + text_stack_h + last_baseline + pad;
  cv::Rect panel(0, 0, std::min(panel_w, img.cols),
                 std::min(panel_h, img.rows));
  if (panel.width <= 0 || panel.height <= 0) {
    return;
  }

  cv::Mat roi = img(panel);
  cv::Mat overlay = roi.clone();
  cv::rectangle(overlay, cv::Rect(0, 0, panel.width, panel.height),
                cv::Scalar(8, 8, 8), cv::FILLED);
  cv::addWeighted(overlay, 0.70, roi, 0.30, 0.0, roi);

  int y = panel.y + pad;
  for (const auto& line : lines) {
    int bl = 0;
    const cv::Size sz = cv::getTextSize(line.first, cv::FONT_HERSHEY_SIMPLEX,
                                        font_scale, text_thick, &bl);
    y += sz.height;
    cv::putText(img, line.first, cv::Point(panel.x + pad, y),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, line.second, text_thick,
                cv::LINE_AA);
    y += line_gap;
  }
}

std::string formatWallTimeMs(std::chrono::system_clock::time_point tp) {
  using namespace std::chrono;
  const auto ms_part =
      duration_cast<milliseconds>(tp.time_since_epoch()) % 1000;
  const std::time_t sec = system_clock::to_time_t(tp);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &sec);
#else
  localtime_r(&sec, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%H:%M:%S") << '.'
      << std::setfill('0') << std::setw(3) << ms_part.count();
  return oss.str();
}

std::string buildSeqTimingLine(
    const PreviewOverlayContext& ctx, int64_t draw_mono_ns,
    std::chrono::system_clock::time_point draw_wall) {
  const int64_t capture_ns =
      ctx.detection.capture_ns > 0 ? ctx.detection.capture_ns : ctx.stamp_ns;

  double age_ms = 0.0;
  std::chrono::system_clock::time_point capts_wall = draw_wall;
  if (capture_ns > 0 && draw_mono_ns >= capture_ns) {
    const int64_t age_ns = draw_mono_ns - capture_ns;
    age_ms = static_cast<double>(age_ns) / 1.0e6;
    capts_wall = draw_wall - std::chrono::nanoseconds(age_ns);
  }

  std::ostringstream ss;
  ss << "Seq frame=" << ctx.frame_seq << " det=" << ctx.detection.seq;
  if (capture_ns > 0) {
    ss << " capts=" << formatWallTimeMs(capts_wall);
  } else {
    ss << " capts=na";
  }
  ss << " drawts=" << formatWallTimeMs(draw_wall);
  ss << " age=" << std::fixed << std::setprecision(1) << age_ms << "ms";
  return ss.str();
}

}  // namespace

void drawPreviewOverlay(cv::Mat& bgr, const PreviewOverlayContext& ctx) {
  if (bgr.empty() || bgr.channels() != 3) {
    return;
  }

  constexpr double kRefWidth = 1280.0;
  constexpr double kRefHeight = 1024.0;
  const double scale = std::clamp(std::min(bgr.cols / kRefWidth,
                                           bgr.rows / kRefHeight),
                                  0.35, 1.2);
  const double font_scale = 0.7 * scale;
  const int text_thick = std::max(1, static_cast<int>(std::round(scale)));
  const int pad = std::max(4, static_cast<int>(8 * scale));
  const int line_gap = std::max(7, static_cast<int>(std::round(10 * scale)));
  const cv::Scalar state_color = phaseColor(ctx);

  const cv::Point2f aim_pt(
      static_cast<float>(bgr.cols) * 0.5F + ctx.aim_offset_x_px.value_or(0.0F),
      static_cast<float>(bgr.rows) * 0.5F + ctx.aim_offset_y_px.value_or(0.0F));
  drawImageCenterGuides(bgr, cv::Scalar(70, 70, 90), 1,
                        static_cast<float>(bgr.cols) * 0.5F,
                        static_cast<float>(bgr.rows) * 0.5F);
  drawDeadbandBox(bgr, ctx.deadband_half_w_px.value_or(0.0F),
                  ctx.deadband_half_h_px.value_or(0.0F), aim_pt);

  int det_count = 0;
  if (ctx.detection.valid && ctx.detection.width > 1.0F &&
      ctx.detection.height > 1.0F) {
    det_count = 1;
    const int x = std::clamp(
        static_cast<int>(std::lround(ctx.detection.cx - ctx.detection.width * 0.5F)),
        0, std::max(0, bgr.cols - 1));
    const int y = std::clamp(
        static_cast<int>(std::lround(ctx.detection.cy - ctx.detection.height * 0.5F)),
        0, std::max(0, bgr.rows - 1));
    const int w = std::clamp(static_cast<int>(std::lround(ctx.detection.width)),
                             1, std::max(1, bgr.cols - x));
    const int h = std::clamp(static_cast<int>(std::lround(ctx.detection.height)),
                             1, std::max(1, bgr.rows - y));
    const cv::Scalar det_color = ctx.has_target ? cv::Scalar(0, 255, 0)
                                                : cv::Scalar(0, 140, 255);
    cv::rectangle(bgr, cv::Rect(x, y, w, h), det_color, ctx.has_target ? 2 : 1,
                  cv::LINE_AA);

    std::ostringstream label;
    label << (ctx.has_target ? "target:" : "cand:")
          << (ctx.detection.class_name.empty() ? "target" : ctx.detection.class_name)
          << " " << std::fixed << std::setprecision(2) << ctx.detection.score;
    if (ctx.detection.track_id >= 0) {
      label << " #" << ctx.detection.track_id;
    }
    if (ctx.detection.tracker_predicted) {
      label << " PRED " << ctx.detection.tracker_lost_frames;
    }
    if (!ctx.has_target) {
      label << " [no target]";
    }
    int bl = 0;
    const cv::Size sz = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX,
                                        font_scale, text_thick, &bl);
    const int top = std::max(0, y - sz.height - pad * 2);
    cv::rectangle(bgr, cv::Rect(x, top, std::min(sz.width + pad * 2, bgr.cols - x),
                                sz.height + bl + pad),
                  det_color, cv::FILLED);
    cv::putText(bgr, label.str(), cv::Point(x + pad, top + sz.height + pad / 2),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 0),
                text_thick, cv::LINE_AA);

    if (ctx.has_target) {
      drawTargetErrorArrow(bgr, aim_pt,
                           cv::Point2f(ctx.detection.cx, ctx.detection.cy),
                           ctx.deadband_half_w_px.value_or(0.0F),
                           ctx.deadband_half_h_px.value_or(0.0F));
    }
  }

  {
    std::ostringstream banner;
    cv::Scalar banner_bg;
    if (det_count == 0) {
      banner << "YOLO: 0 det";
      banner_bg = cv::Scalar(0, 0, 180);
    } else if (ctx.has_target) {
      banner << det_count << " det -> #1 ACCEPT";
      banner_bg = cv::Scalar(0, 140, 0);
    } else {
      banner << det_count << " det -> NO TARGET";
      banner_bg = cv::Scalar(0, 80, 200);
    }
    int bl = 0;
    const cv::Size sz = cv::getTextSize(banner.str(), cv::FONT_HERSHEY_SIMPLEX,
                                        font_scale, text_thick, &bl);
    const int banner_w = sz.width + pad * 2;
    const int banner_h = sz.height + bl + pad;
    const int bx = std::max(0, bgr.cols - banner_w);
    cv::rectangle(bgr, cv::Point(bx, 0), cv::Point(bgr.cols, banner_h),
                  banner_bg, cv::FILLED);
    cv::putText(bgr, banner.str(), cv::Point(bx + pad, pad + sz.height),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255),
                text_thick, cv::LINE_AA);
  }

  std::vector<std::pair<std::string, cv::Scalar>> lines;
  {
    std::ostringstream ss;
    ss << ctx.source << " [" << (ctx.state.empty() ? "WaitingTarget" : ctx.state)
       << "] " << (ctx.armed ? "ARM" : "SAFE");
    if (ctx.fps.has_value()) {
      ss << " " << std::fixed << std::setprecision(1) << *ctx.fps << "fps";
    }
    lines.emplace_back(ss.str(), state_color);
  }
  {
    std::ostringstream ss;
    ss << "Tgt det:" << det_count;
    if (ctx.detection.valid && ctx.has_target) {
      ss << " best:"
         << (ctx.detection.class_name.empty() ? "target" : ctx.detection.class_name)
         << " " << std::fixed << std::setprecision(2) << ctx.detection.score;
    } else if (ctx.detection.valid) {
      ss << " cand:"
         << (ctx.detection.class_name.empty() ? "target" : ctx.detection.class_name)
         << " " << std::fixed << std::setprecision(2) << ctx.detection.score
         << " not-used";
    }
    lines.emplace_back(ss.str(), cv::Scalar(80, 255, 255));
  }
  {
    std::ostringstream ss;
    if (ctx.has_target) {
      const float dx = ctx.pixel_offset_x.value_or(
          ctx.detection.valid ? ctx.detection.cx - aim_pt.x : 0.0F);
      const float dy = ctx.pixel_offset_y.value_or(
          ctx.detection.valid ? ctx.detection.cy - aim_pt.y : 0.0F);
      ss << "Tgt ex=" << std::fixed << std::setprecision(0) << dx
         << "px ey=" << dy << "px";
    } else {
      ss << "Tgt no accepted target";
    }
    if (ctx.deadband_half_w_px.has_value() && ctx.deadband_half_h_px.has_value()) {
      ss << " db=" << *ctx.deadband_half_w_px << "x"
         << *ctx.deadband_half_h_px << "px";
    }
    lines.emplace_back(ss.str(), cv::Scalar(120, 200, 255));
  }
  if (ctx.roll_rate_rad_s.has_value() && ctx.pitch_rate_rad_s.has_value() &&
      ctx.yaw_rate_rad_s.has_value()) {
    std::ostringstream ss;
    ss << "Tgt[PD] r_dot=" << std::fixed << std::setprecision(1)
       << (*ctx.roll_rate_rad_s * kRad2Deg) << "deg/s p_dot="
       << (*ctx.pitch_rate_rad_s * kRad2Deg) << "deg/s yr="
       << (*ctx.yaw_rate_rad_s * kRad2Deg) << "deg/s";
    lines.emplace_back(ss.str(), cv::Scalar(120, 200, 255));
  }
  if (ctx.vehicle_roll_rad.has_value() && ctx.vehicle_pitch_rad.has_value() &&
      ctx.vehicle_yaw_rad.has_value()) {
    std::ostringstream ss;
    ss << "BF att r=" << std::fixed << std::setprecision(1)
       << (*ctx.vehicle_roll_rad * kRad2Deg) << "deg p="
       << (*ctx.vehicle_pitch_rad * kRad2Deg) << "deg y="
       << (*ctx.vehicle_yaw_rad * kRad2Deg) << "deg";
    lines.emplace_back(ss.str(), cv::Scalar(180, 220, 120));
  }
  if (ctx.vehicle_throttle_pwm.has_value()) {
    std::ostringstream ss;
    ss << "T_act pwm=" << std::fixed << std::setprecision(0)
       << *ctx.vehicle_throttle_pwm;
    if (ctx.vehicle_throttle_norm.has_value()) {
      ss << " norm=" << std::setprecision(2) << *ctx.vehicle_throttle_norm;
    }
    lines.emplace_back(ss.str(), cv::Scalar(180, 220, 120));
  }
  const bool has_perf = ctx.perf_prod_fps.has_value() || ctx.perf_inf_fps.has_value() ||
                        ctx.perf_wait_grab_ms.has_value() ||
                        ctx.perf_e2e_input_ms.has_value() ||
                        ctx.perf_e2e_wire_ms.has_value() ||
                        ctx.perf_e2e_algo_ms.has_value();
  if (has_perf) {
    const cv::Scalar perf_color(100, 180, 255);
    auto fmt_fps = [](std::ostringstream& ss, const char* label,
                      const std::optional<float>& v, bool allow_na) {
      ss << ' ' << label << '=';
      if (v.has_value() && std::isfinite(*v)) {
        ss << std::fixed << std::setprecision(0) << *v;
      } else if (allow_na) {
        ss << "n/a";
      }
    };
    {
      std::ostringstream ss;
      ss << "FPS:";
      fmt_fps(ss, "e2e", ctx.perf_e2e_fps, !ctx.perf_wire_path_active);
      fmt_fps(ss, "prod", ctx.perf_prod_fps, false);
      fmt_fps(ss, "inf", ctx.perf_inf_fps, false);
      fmt_fps(ss, "ctrl", ctx.perf_ctrl_fps, false);
      fmt_fps(ss, "msp", ctx.perf_msp_fps, !ctx.perf_wire_path_active);
      lines.emplace_back(ss.str(), perf_color);
    }
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(1) << "Lat:";
      if (ctx.perf_wait_grab_ms.has_value()) {
        ss << " grab=" << *ctx.perf_wait_grab_ms << "ms";
      }
      if (ctx.perf_e2e_input_ms.has_value()) {
        ss << " ctrl_in=" << *ctx.perf_e2e_input_ms << "ms";
      }
      if (ctx.perf_e2e_input_p50_ms.has_value()) {
        ss << " p50=" << *ctx.perf_e2e_input_p50_ms << "ms";
      }
      lines.emplace_back(ss.str(), perf_color);
    }
    if (ctx.perf_queue_wait_ms.has_value() || ctx.perf_producer_ms.has_value() ||
        ctx.perf_cnn_ms.has_value() || ctx.perf_ctrl_ms.has_value() ||
        ctx.perf_msp_gate_ms.has_value()) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(1) << "Step:";
      if (ctx.perf_queue_wait_ms.has_value()) {
        ss << " queue=" << *ctx.perf_queue_wait_ms << "ms";
      }
      if (ctx.perf_producer_ms.has_value()) {
        ss << " decode=" << *ctx.perf_producer_ms << "ms";
      }
      if (ctx.perf_cnn_ms.has_value()) {
        ss << " cnn=" << *ctx.perf_cnn_ms << "ms";
      }
      if (ctx.perf_ctrl_ms.has_value()) {
        ss << " ctrl=" << *ctx.perf_ctrl_ms << "ms";
      }
      if (ctx.perf_wire_path_active && ctx.perf_msp_gate_ms.has_value()) {
        ss << " msp=" << *ctx.perf_msp_gate_ms << "ms";
      }
      lines.emplace_back(ss.str(), perf_color);
    }
    if (ctx.perf_pipe_zero_copy && ctx.perf_pipe_slot_count.has_value()) {
      std::ostringstream ss;
      ss << "Pipe: slots=" << *ctx.perf_pipe_slot_count;
      if (ctx.perf_pipe_slot_busy.has_value()) {
        ss << " slot_drop=" << *ctx.perf_pipe_slot_busy;
      }
      if (ctx.perf_pipe_ready_drop.has_value()) {
        ss << " ready_drop=" << *ctx.perf_pipe_ready_drop;
      }
      lines.emplace_back(ss.str(), perf_color);
    }
  }
  if (ctx.preprocess_time_ms.has_value() || ctx.inference_time_ms.has_value() ||
      ctx.postprocess_time_ms.has_value()) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << "CNN:";
    if (ctx.preprocess_time_ms.has_value()) {
      ss << " pre=" << *ctx.preprocess_time_ms << "ms";
    }
    if (ctx.inference_time_ms.has_value()) {
      ss << " inf=" << *ctx.inference_time_ms << "ms";
    }
    if (ctx.postprocess_time_ms.has_value()) {
      ss << " post=" << *ctx.postprocess_time_ms << "ms";
    }
    lines.emplace_back(ss.str(), cv::Scalar(100, 180, 255));
  }

  if (ctx.thrust_z.has_value() || ctx.vehicle_throttle_norm.has_value()) {
    const bool bf_source = ctx.source == "bf";
    const float cmd =
        ctx.thrust_z.has_value()
            ? (bf_source ? *ctx.thrust_z : -*ctx.thrust_z)
            : 0.0F;
    const float act = ctx.vehicle_throttle_norm.value_or(cmd);
    drawThrustBar(bgr, cmd, act);
  }

  // Timing line is appended after all overlay elements so drawts/age reflect
  // full preview draw completion.
  {
    const int64_t draw_mono_ns = circle::types::monotonicNowNs();
    const auto draw_wall = std::chrono::system_clock::now();
    lines.emplace_back(buildSeqTimingLine(ctx, draw_mono_ns, draw_wall),
                     cv::Scalar(210, 210, 230));
  }
  drawInfoPanel(bgr, lines, font_scale, text_thick, pad, line_gap);
}

PreviewOverlayShmData fromPreviewOverlayContext(const PreviewOverlayContext& ctx) {
  PreviewOverlayShmData data{};

  auto optToShmFloat = [](const std::optional<float>& opt) -> float {
    return opt.has_value() ? *opt : std::numeric_limits<float>::quiet_NaN();
  };
  auto optToShmDouble = [](const std::optional<double>& opt) -> double {
    return opt.has_value() ? *opt : std::numeric_limits<double>::quiet_NaN();
  };
  auto optToShmInt = [](const std::optional<int>& opt) -> int32_t {
    return opt.has_value() ? static_cast<int32_t>(*opt) : INT32_MIN;
  };
  auto optToShmUint64 = [](const std::optional<uint64_t>& opt) -> uint64_t {
    return opt.has_value() ? *opt : 0ULL;
  };
  auto copyStrToShm = [](char* dst, size_t max, const std::string& src) {
    std::strncpy(dst, src.c_str(), max - 1);
    dst[max - 1] = '\0';
  };

  copyStrToShm(data.source, sizeof(data.source), ctx.source);
  copyStrToShm(data.mode_tag, sizeof(data.mode_tag), ctx.mode_tag);
  copyStrToShm(data.state, sizeof(data.state), ctx.state);
  data.armed = ctx.armed ? 1 : 0;
  data.has_target = ctx.has_target ? 1 : 0;
  data.frame_seq = ctx.frame_seq;
  data.stamp_ns = ctx.stamp_ns;
  data.fps = optToShmDouble(ctx.fps);

  data.detection.valid = ctx.detection.valid ? 1 : 0;
  data.detection.cx = ctx.detection.cx;
  data.detection.cy = ctx.detection.cy;
  data.detection.width = ctx.detection.width;
  data.detection.height = ctx.detection.height;
  data.detection.score = ctx.detection.score;
  data.detection.class_id = static_cast<int32_t>(ctx.detection.class_id);
  copyStrToShm(data.detection.class_name, sizeof(data.detection.class_name),
               ctx.detection.class_name);
  data.detection.seq = ctx.detection.seq;
  data.detection.capture_ns = ctx.detection.capture_ns;
  data.detection.track_id = static_cast<int32_t>(ctx.detection.track_id);
  data.detection.tracker_predicted =
      ctx.detection.tracker_predicted ? 1 : 0;
  data.detection.tracker_lost_frames =
      static_cast<int32_t>(ctx.detection.tracker_lost_frames);

  data.pixel_offset_x = optToShmFloat(ctx.pixel_offset_x);
  data.pixel_offset_y = optToShmFloat(ctx.pixel_offset_y);
  data.image_ex = optToShmFloat(ctx.image_ex);
  data.image_ey = optToShmFloat(ctx.image_ey);
  data.roll_rate_rad_s = optToShmFloat(ctx.roll_rate_rad_s);
  data.pitch_rate_rad_s = optToShmFloat(ctx.pitch_rate_rad_s);
  data.yaw_rate_rad_s = optToShmFloat(ctx.yaw_rate_rad_s);
  data.thrust_z = optToShmFloat(ctx.thrust_z);
  data.throttle_algo_norm = optToShmFloat(ctx.throttle_algo_norm);
  data.throttle_cmd_norm = optToShmFloat(ctx.throttle_cmd_norm);

  data.vehicle_roll_rad = optToShmFloat(ctx.vehicle_roll_rad);
  data.vehicle_pitch_rad = optToShmFloat(ctx.vehicle_pitch_rad);
  data.vehicle_yaw_rad = optToShmFloat(ctx.vehicle_yaw_rad);
  data.vehicle_roll_rate_rad_s = optToShmFloat(ctx.vehicle_roll_rate_rad_s);
  data.vehicle_pitch_rate_rad_s = optToShmFloat(ctx.vehicle_pitch_rate_rad_s);
  data.vehicle_yaw_rate_rad_s = optToShmFloat(ctx.vehicle_yaw_rate_rad_s);
  data.vehicle_throttle_pwm = optToShmFloat(ctx.vehicle_throttle_pwm);
  data.vehicle_throttle_norm = optToShmFloat(ctx.vehicle_throttle_norm);

  data.cam_pipeline_ms = optToShmFloat(ctx.cam_pipeline_ms);
  data.capture_time_ms = optToShmFloat(ctx.capture_time_ms);
  data.decode_time_ms = optToShmFloat(ctx.decode_time_ms);
  data.preprocess_time_ms = optToShmFloat(ctx.preprocess_time_ms);
  data.inference_time_ms = optToShmFloat(ctx.inference_time_ms);
  data.postprocess_time_ms = optToShmFloat(ctx.postprocess_time_ms);
  data.executor_ms = optToShmFloat(ctx.executor_ms);
  data.e2e_ms = optToShmFloat(ctx.e2e_ms);

  data.perf_wire_path_active = ctx.perf_wire_path_active ? 1 : 0;
  data.perf_pipe_zero_copy = ctx.perf_pipe_zero_copy ? 1 : 0;
  data.perf_e2e_fps = optToShmFloat(ctx.perf_e2e_fps);
  data.perf_prod_fps = optToShmFloat(ctx.perf_prod_fps);
  data.perf_inf_fps = optToShmFloat(ctx.perf_inf_fps);
  data.perf_ctrl_fps = optToShmFloat(ctx.perf_ctrl_fps);
  data.perf_msp_fps = optToShmFloat(ctx.perf_msp_fps);
  data.perf_wait_grab_ms = optToShmFloat(ctx.perf_wait_grab_ms);
  data.perf_e2e_input_ms = optToShmFloat(ctx.perf_e2e_input_ms);
  data.perf_e2e_input_p50_ms = optToShmFloat(ctx.perf_e2e_input_p50_ms);
  data.perf_e2e_wire_ms = optToShmFloat(ctx.perf_e2e_wire_ms);
  data.perf_e2e_wire_p50_ms = optToShmFloat(ctx.perf_e2e_wire_p50_ms);
  data.perf_e2e_algo_ms = optToShmFloat(ctx.perf_e2e_algo_ms);
  data.perf_e2e_algo_p50_ms = optToShmFloat(ctx.perf_e2e_algo_p50_ms);
  data.perf_queue_wait_ms = optToShmFloat(ctx.perf_queue_wait_ms);
  data.perf_producer_ms = optToShmFloat(ctx.perf_producer_ms);
  data.perf_cnn_ms = optToShmFloat(ctx.perf_cnn_ms);
  data.perf_ctrl_ms = optToShmFloat(ctx.perf_ctrl_ms);
  data.perf_msp_gate_ms = optToShmFloat(ctx.perf_msp_gate_ms);
  data.perf_pipe_slot_count = optToShmInt(ctx.perf_pipe_slot_count);
  data.perf_pipe_slot_busy = optToShmUint64(ctx.perf_pipe_slot_busy);
  data.perf_pipe_ready_drop = optToShmUint64(ctx.perf_pipe_ready_drop);

  data.deadband_half_w_px = optToShmFloat(ctx.deadband_half_w_px);
  data.deadband_half_h_px = optToShmFloat(ctx.deadband_half_h_px);
  data.aim_offset_x_px = optToShmFloat(ctx.aim_offset_x_px);
  data.aim_offset_y_px = optToShmFloat(ctx.aim_offset_y_px);

  return data;
}

PreviewOverlayContext toPreviewOverlayContext(const PreviewOverlayShmData& data) {
  PreviewOverlayContext ctx;

  auto shmToOptFloat = [](float v) -> std::optional<float> {
    return std::isnan(v) ? std::nullopt : std::make_optional(v);
  };
  auto shmToOptDouble = [](double v) -> std::optional<double> {
    return std::isnan(v) ? std::nullopt : std::make_optional(v);
  };
  auto shmToOptInt = [](int32_t v) -> std::optional<int> {
    return (v == INT32_MIN) ? std::nullopt : std::make_optional(static_cast<int>(v));
  };
  auto shmToOptUint64 = [](uint64_t v) -> std::optional<uint64_t> {
    return (v == 0ULL) ? std::nullopt : std::make_optional(v);
  };
  auto shmStrToStd = [](const char* src, size_t max) -> std::string {
    size_t len = 0;
    while (len < max && src[len] != '\0') ++len;
    return std::string(src, len);
  };

  ctx.source = shmStrToStd(data.source, sizeof(data.source));
  ctx.mode_tag = shmStrToStd(data.mode_tag, sizeof(data.mode_tag));
  ctx.state = shmStrToStd(data.state, sizeof(data.state));
  ctx.armed = data.armed != 0;
  ctx.has_target = data.has_target != 0;
  ctx.frame_seq = data.frame_seq;
  ctx.stamp_ns = data.stamp_ns;
  ctx.fps = shmToOptDouble(data.fps);

  ctx.detection.valid = data.detection.valid != 0;
  ctx.detection.cx = data.detection.cx;
  ctx.detection.cy = data.detection.cy;
  ctx.detection.width = data.detection.width;
  ctx.detection.height = data.detection.height;
  ctx.detection.score = data.detection.score;
  ctx.detection.class_id = static_cast<int>(data.detection.class_id);
  ctx.detection.class_name =
      shmStrToStd(data.detection.class_name, sizeof(data.detection.class_name));
  ctx.detection.seq = data.detection.seq;
  ctx.detection.capture_ns = data.detection.capture_ns;
  ctx.detection.track_id = static_cast<int>(data.detection.track_id);
  ctx.detection.tracker_predicted = data.detection.tracker_predicted != 0;
  ctx.detection.tracker_lost_frames =
      static_cast<int>(data.detection.tracker_lost_frames);

  ctx.pixel_offset_x = shmToOptFloat(data.pixel_offset_x);
  ctx.pixel_offset_y = shmToOptFloat(data.pixel_offset_y);
  ctx.image_ex = shmToOptFloat(data.image_ex);
  ctx.image_ey = shmToOptFloat(data.image_ey);
  ctx.roll_rate_rad_s = shmToOptFloat(data.roll_rate_rad_s);
  ctx.pitch_rate_rad_s = shmToOptFloat(data.pitch_rate_rad_s);
  ctx.yaw_rate_rad_s = shmToOptFloat(data.yaw_rate_rad_s);
  ctx.thrust_z = shmToOptFloat(data.thrust_z);
  ctx.throttle_algo_norm = shmToOptFloat(data.throttle_algo_norm);
  ctx.throttle_cmd_norm = shmToOptFloat(data.throttle_cmd_norm);

  ctx.vehicle_roll_rad = shmToOptFloat(data.vehicle_roll_rad);
  ctx.vehicle_pitch_rad = shmToOptFloat(data.vehicle_pitch_rad);
  ctx.vehicle_yaw_rad = shmToOptFloat(data.vehicle_yaw_rad);
  ctx.vehicle_roll_rate_rad_s = shmToOptFloat(data.vehicle_roll_rate_rad_s);
  ctx.vehicle_pitch_rate_rad_s = shmToOptFloat(data.vehicle_pitch_rate_rad_s);
  ctx.vehicle_yaw_rate_rad_s = shmToOptFloat(data.vehicle_yaw_rate_rad_s);
  ctx.vehicle_throttle_pwm = shmToOptFloat(data.vehicle_throttle_pwm);
  ctx.vehicle_throttle_norm = shmToOptFloat(data.vehicle_throttle_norm);

  ctx.cam_pipeline_ms = shmToOptFloat(data.cam_pipeline_ms);
  ctx.capture_time_ms = shmToOptFloat(data.capture_time_ms);
  ctx.decode_time_ms = shmToOptFloat(data.decode_time_ms);
  ctx.preprocess_time_ms = shmToOptFloat(data.preprocess_time_ms);
  ctx.inference_time_ms = shmToOptFloat(data.inference_time_ms);
  ctx.postprocess_time_ms = shmToOptFloat(data.postprocess_time_ms);
  ctx.executor_ms = shmToOptFloat(data.executor_ms);
  ctx.e2e_ms = shmToOptFloat(data.e2e_ms);

  ctx.perf_wire_path_active = data.perf_wire_path_active != 0;
  ctx.perf_pipe_zero_copy = data.perf_pipe_zero_copy != 0;
  ctx.perf_e2e_fps = shmToOptFloat(data.perf_e2e_fps);
  ctx.perf_prod_fps = shmToOptFloat(data.perf_prod_fps);
  ctx.perf_inf_fps = shmToOptFloat(data.perf_inf_fps);
  ctx.perf_ctrl_fps = shmToOptFloat(data.perf_ctrl_fps);
  ctx.perf_msp_fps = shmToOptFloat(data.perf_msp_fps);
  ctx.perf_wait_grab_ms = shmToOptFloat(data.perf_wait_grab_ms);
  ctx.perf_e2e_input_ms = shmToOptFloat(data.perf_e2e_input_ms);
  ctx.perf_e2e_input_p50_ms = shmToOptFloat(data.perf_e2e_input_p50_ms);
  ctx.perf_e2e_wire_ms = shmToOptFloat(data.perf_e2e_wire_ms);
  ctx.perf_e2e_wire_p50_ms = shmToOptFloat(data.perf_e2e_wire_p50_ms);
  ctx.perf_e2e_algo_ms = shmToOptFloat(data.perf_e2e_algo_ms);
  ctx.perf_e2e_algo_p50_ms = shmToOptFloat(data.perf_e2e_algo_p50_ms);
  ctx.perf_queue_wait_ms = shmToOptFloat(data.perf_queue_wait_ms);
  ctx.perf_producer_ms = shmToOptFloat(data.perf_producer_ms);
  ctx.perf_cnn_ms = shmToOptFloat(data.perf_cnn_ms);
  ctx.perf_ctrl_ms = shmToOptFloat(data.perf_ctrl_ms);
  ctx.perf_msp_gate_ms = shmToOptFloat(data.perf_msp_gate_ms);
  ctx.perf_pipe_slot_count = shmToOptInt(data.perf_pipe_slot_count);
  ctx.perf_pipe_slot_busy = shmToOptUint64(data.perf_pipe_slot_busy);
  ctx.perf_pipe_ready_drop = shmToOptUint64(data.perf_pipe_ready_drop);

  ctx.deadband_half_w_px = shmToOptFloat(data.deadband_half_w_px);
  ctx.deadband_half_h_px = shmToOptFloat(data.deadband_half_h_px);
  ctx.aim_offset_x_px = shmToOptFloat(data.aim_offset_x_px);
  ctx.aim_offset_y_px = shmToOptFloat(data.aim_offset_y_px);

  return ctx;
}

}  // namespace circle::debug
