#pragma once

#include <cstdint>
#include <cmath>

namespace circle::strike_png {

struct TargetLossHoldState {
  bool has_seen_target{false};
  uint64_t lost_since_ns{0};
  bool hold_requested{false};

  void reset() {
    has_seen_target = false;
    lost_since_ns = 0;
    hold_requested = false;
  }
};

inline bool updateTargetLossHold(TargetLossHoldState& state,
                                 bool active,
                                 bool has_target,
                                 uint64_t now_ns,
                                 uint64_t delay_ns,
                                 bool enable) {
  if (!enable || !active || now_ns == 0U) {
    state.reset();
    return false;
  }

  if (has_target) {
    state.has_seen_target = true;
    state.lost_since_ns = 0;
    state.hold_requested = false;
    return false;
  }

  if (!state.has_seen_target) {
    return false;
  }

  if (state.lost_since_ns == 0U) {
    state.lost_since_ns = now_ns;
    return false;
  }

  if (state.hold_requested) {
    return false;
  }

  if (now_ns - state.lost_since_ns < delay_ns) {
    return false;
  }

  state.hold_requested = true;
  return true;
}

struct TargetLossImageContext {
  bool valid{false};
  float image_width_px{0.0F};
  float image_height_px{0.0F};
  float cx_px{0.0F};
  float cy_px{0.0F};
  float width_px{0.0F};
  float height_px{0.0F};
  float bbox_area_ratio{0.0F};
};

inline const char* classifyTargetLossImageContext(
    const TargetLossImageContext& ctx,
    float edge_margin_px,
    float large_bbox_area_ratio) {
  if (!ctx.valid) {
    return "no_detection";
  }
  if (ctx.image_width_px <= 0.0F || ctx.image_height_px <= 0.0F) {
    return "no_camera";
  }
  if (ctx.width_px <= 0.0F || ctx.height_px <= 0.0F ||
      !std::isfinite(ctx.cx_px) || !std::isfinite(ctx.cy_px)) {
    return "invalid_bbox";
  }

  const float left = ctx.cx_px - 0.5F * ctx.width_px;
  const float right = ctx.image_width_px - (ctx.cx_px + 0.5F * ctx.width_px);
  const float top = ctx.cy_px - 0.5F * ctx.height_px;
  const float bottom =
      ctx.image_height_px - (ctx.cy_px + 0.5F * ctx.height_px);
  const float margin = std::fmin(std::fmin(left, right), std::fmin(top, bottom));
  if (margin <= edge_margin_px) {
    if (top <= left && top <= right && top <= bottom) {
      return "edge_top";
    }
    if (bottom <= left && bottom <= right && bottom <= top) {
      return "edge_bottom";
    }
    if (left <= right && left <= top && left <= bottom) {
      return "edge_left";
    }
    return "edge_right";
  }

  if (ctx.bbox_area_ratio >= large_bbox_area_ratio) {
    return "large_bbox";
  }
  return "stale_center";
}

}  // namespace circle::strike_png
