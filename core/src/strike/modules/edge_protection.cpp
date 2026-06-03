#include "circle/strike/modules/edge_protection.hpp"

#include <algorithm>
#include <cmath>

#include "circle/strike/math_utils.hpp"

namespace circle::strike {

EdgeProtectionOutput EdgeProtection::compute(
    bool final_approach_active, bool fresh_detection,
    const circle::types::FrameDetection& detection,
    const circle::types::FcState& vehicle,
    float, float image_ey,
    float ex_dot_filt, float ey_dot_filt,
    float lateral_output_sign, float longitudinal_output_sign,
    const FAEdgeProtectParams& ep,
    const FABottomPitchGuardParams& bpg,
    float commit_min_margin_x_px, float commit_min_margin_y_px) const {
  EdgeProtectionOutput out;

  const float edge_margin_x_px = std::max(ep.margin_x_px, commit_min_margin_x_px);
  const float edge_margin_y_px = std::max(ep.margin_y_px, commit_min_margin_y_px);

  if (ep.enable && final_approach_active && fresh_detection &&
      (edge_margin_x_px > 0.0F || edge_margin_y_px > 0.0F) &&
      detection.intrinsics.fx > 1.0e-6F &&
      detection.intrinsics.fy > 1.0e-6F) {
    const float bbox_w = detection.detection.width;
    const float bbox_h = detection.detection.height;
    const float bbox_cx = detection.detection.cx;
    const float bbox_cy = detection.detection.cy;
    const float bbox_left_margin_px = bbox_cx - bbox_w * 0.5F;
    const float bbox_right_margin_px =
        static_cast<float>(detection.image_width) - (bbox_cx + bbox_w * 0.5F);
    const float bbox_top_margin_px = bbox_cy - bbox_h * 0.5F;
    const float bbox_bottom_margin_px =
        static_cast<float>(detection.image_height) - (bbox_cy + bbox_h * 0.5F);

    const float bbox_center_x_dot_px_s =
        ex_dot_filt * detection.intrinsics.fx;
    const float bbox_center_y_dot_px_s =
        ey_dot_filt * detection.intrinsics.fy;
    const float left_margin_pred_px =
        bbox_left_margin_px + bbox_center_x_dot_px_s * ep.predict_s;
    const float right_margin_pred_px =
        bbox_right_margin_px - bbox_center_x_dot_px_s * ep.predict_s;
    const float top_margin_pred_px =
        bbox_top_margin_px + bbox_center_y_dot_px_s * ep.predict_s;
    const float bottom_margin_pred_px =
        bbox_bottom_margin_px - bbox_center_y_dot_px_s * ep.predict_s;

    const float guarded_top_margin_px =
        std::min(bbox_top_margin_px, top_margin_pred_px);
    const float guarded_bottom_margin_px =
        std::min(bbox_bottom_margin_px, bottom_margin_pred_px);
    const float guarded_left_margin_px =
        std::min(bbox_left_margin_px, left_margin_pred_px);
    const float guarded_right_margin_px =
        std::min(bbox_right_margin_px, right_margin_pred_px);

    const float left_deficit_px = edge_margin_x_px - guarded_left_margin_px;
    const float right_deficit_px = edge_margin_x_px - guarded_right_margin_px;
    const float top_deficit_px = edge_margin_y_px - guarded_top_margin_px;
    const float bottom_deficit_px = edge_margin_y_px - guarded_bottom_margin_px;

    if (left_deficit_px > 0.0F || right_deficit_px > 0.0F) {
      out.active = true;
      float edge_x_error_px = 0.0F;
      if (right_deficit_px >= left_deficit_px) {
        edge_x_error_px = std::max(0.0F, right_deficit_px);
      } else {
        edge_x_error_px = -std::max(0.0F, left_deficit_px);
      }
      const float edge_x_error_norm =
          edge_x_error_px / detection.intrinsics.fx;
      out.roll_boost = lateral_output_sign * ep.roll_kp_rate *
                       edge_x_error_norm;
      out.thrust_scale = ep.thrust_scale;
    }

    if (top_deficit_px > 0.0F || bottom_deficit_px > 0.0F) {
      out.active = true;
      float edge_error_px = 0.0F;
      if (top_deficit_px >= bottom_deficit_px) {
        edge_error_px = -std::max(0.0F, top_deficit_px);
      } else {
        edge_error_px = std::max(0.0F, bottom_deficit_px);
      }
      const float edge_error_mag_norm =
          std::abs(edge_error_px) / detection.intrinsics.fy;
      const float signed_boost =
          longitudinal_output_sign * ep.pitch_kp_rate *
          std::copysign(edge_error_mag_norm, edge_error_px);
      out.pitch_boost = std::clamp(signed_boost,
          -ep.pitch_boost_max_rad_s, ep.pitch_boost_max_rad_s);
      out.thrust_scale = ep.thrust_scale;
    }

    if (edge_margin_x_px > 1.0F) {
      out.taper_score = std::max(out.taper_score,
          std::max(left_deficit_px, right_deficit_px) / edge_margin_x_px);
    }
    if (edge_margin_y_px > 1.0F) {
      out.taper_score = std::max(out.taper_score,
          std::max(top_deficit_px, bottom_deficit_px) / edge_margin_y_px);
    }
    out.taper_score = std::clamp(out.taper_score, 0.0F, 10.0F);
  }

  // Bottom pitch guard
  if (final_approach_active && bpg.enable &&
      bpg.max_rad_s > 0.0F && bpg.level_kp > 0.0F &&
      fresh_detection && detection.intrinsics.fy > 1.0e-6F) {
    const float bbox_cy = detection.detection.cy;
    const float bbox_h = detection.detection.height;
    float bottom_margin_px =
        static_cast<float>(detection.image_height) - (bbox_cy + bbox_h * 0.5F);
    if (out.active) {
      const float bbox_center_y_dot_px_s =
          ey_dot_filt * detection.intrinsics.fy;
      const float bottom_margin_pred_px =
          bottom_margin_px - bbox_center_y_dot_px_s * ep.predict_s;
      bottom_margin_px = std::min(bottom_margin_px, bottom_margin_pred_px);
    }

    const float bottom_margin_deficit_px = bpg.margin_px - bottom_margin_px;
    const float bottom_margin_score = bpg.margin_px > 1.0F
        ? smoothstep01(bottom_margin_deficit_px / bpg.margin_px) : 0.0F;
    const float fa_align_error_y_px =
        std::abs(image_ey * detection.intrinsics.fy);
    const float y_error_score = smoothstep01(
        (fa_align_error_y_px - bpg.error_start_px) /
        (bpg.error_full_px - bpg.error_start_px));

    const float top_deficit = bpg.margin_px - (bbox_cy - bbox_h * 0.5F);
    const float bottom_deficit = bpg.margin_px -
        (static_cast<float>(detection.image_height) - (bbox_cy + bbox_h * 0.5F));
    const bool target_lower_side =
        image_ey > 0.0F || bottom_deficit > top_deficit;
    out.bottom_pitch_guard_blend =
        target_lower_side ? std::max(bottom_margin_score, y_error_score) : 0.0F;
    if (out.bottom_pitch_guard_blend > 0.0F) {
      out.bottom_pitch_guard_active = true;
      out.bottom_pitch_guard_rate = std::clamp(
          -bpg.level_kp * vehicle.pitch_rad, 0.0F, bpg.max_rad_s);
    }
  }

  return out;
}

}  // namespace circle::strike
