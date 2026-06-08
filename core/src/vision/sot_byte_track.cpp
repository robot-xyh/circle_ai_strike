#include "circle/vision/sot_byte_track.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace circle::vision {
namespace {

float detectionArea(const circle::types::Detection& d) {
  return std::max(0.0F, d.width) * std::max(0.0F, d.height);
}

float detectionAspect(const circle::types::Detection& d) {
  const float w = std::max(1.0F, d.width);
  const float h = std::max(1.0F, d.height);
  return std::max(w, h) / std::min(w, h);
}

float clampFinite(float v, float lo, float hi) {
  if (!std::isfinite(v)) {
    return lo;
  }
  return std::clamp(v, lo, hi);
}

}  // namespace

SotByteTrack::SotByteTrack(SotByteTrackParams params)
    : params_(params) {}

void SotByteTrack::setParams(const SotByteTrackParams& params) {
  params_ = params;
}

void SotByteTrack::reset() {
  track_ = Track{};
  last_processed_capture_ns_ = 0;
}

std::optional<circle::types::Detection> SotByteTrack::update(
    const std::vector<circle::types::Detection>& detections,
    const DetectionFilterParams& filter,
    circle::types::TimestampNs capture_ns,
    uint32_t image_width,
    uint32_t image_height) {
  if (capture_ns == 0 ||
      (last_processed_capture_ns_ != 0 && capture_ns <= last_processed_capture_ns_)) {
    return std::nullopt;
  }
  const float dt = computeDt(capture_ns);
  float freeze_width = 0.0F;
  float freeze_height = 0.0F;
  if (track_.active) {
    if (dt <= 0.0F || dt > params_.max_dt_s) {
      destroyTrack();
    } else {
      freeze_width = track_.x(2);
      freeze_height = track_.x(3);
      predict(dt, /*freeze_scale=*/false);
    }
  }
  last_processed_capture_ns_ = capture_ns;

  const auto high =
      acceptedCandidates(detections, filter, params_.high_score_threshold);
  const auto low =
      acceptedCandidates(detections, filter, params_.low_score_threshold,
                         params_.high_score_threshold);

  if (!track_.active) {
    const auto it = std::max_element(
        high.begin(), high.end(),
        [](const Candidate& a, const Candidate& b) {
          if (a.area == b.area) {
            return a.detection.score < b.detection.score;
          }
          return a.area < b.area;
        });
    if (it == high.end() || it->detection.score < params_.new_track_threshold) {
      return std::nullopt;
    }
    initializeTrack(it->detection, capture_ns);
    return predictedDetection(/*predicted=*/false, image_width, image_height);
  }

  circle::types::Detection predicted =
      predictedDetection(/*predicted=*/true, image_width, image_height);

  auto matchCandidate = [&](const std::vector<Candidate>& candidates,
                            float iou_threshold)
      -> const Candidate* {
    const Candidate* best = nullptr;
    float best_iou = iou_threshold;
    for (const auto& candidate : candidates) {
      const float iou = boxIou(predicted, candidate.detection);
      if (iou >= best_iou) {
        best_iou = iou;
        best = &candidate;
      }
    }
    return best;
  };

  if (const Candidate* best = matchCandidate(high, params_.match_iou_threshold)) {
    correct(best->detection, capture_ns);
    return predictedDetection(/*predicted=*/false, image_width, image_height);
  }
  if (const Candidate* best =
          matchCandidate(low, params_.second_match_iou_threshold)) {
    correct(best->detection, capture_ns);
    return predictedDetection(/*predicted=*/false, image_width, image_height);
  }

  ++track_.lost_frames;
  track_.last_capture_ns = capture_ns;
  track_.x(2) =
      std::max(params_.min_box_size_px,
               freeze_width > 0.0F ? freeze_width : track_.last_detection.width);
  track_.x(3) =
      std::max(params_.min_box_size_px,
               freeze_height > 0.0F ? freeze_height : track_.last_detection.height);
  track_.x(6) = 0.0F;
  track_.x(7) = 0.0F;
  if (track_.lost_frames > params_.max_lost_frames) {
    destroyTrack();
    return std::nullopt;
  }
  if (!params_.emit_prediction_on_miss ||
      track_.lost_frames > params_.emit_prediction_max_frames) {
    return std::nullopt;
  }
  return predictedDetection(/*predicted=*/true, image_width, image_height);
}

float SotByteTrack::computeDt(circle::types::TimestampNs capture_ns) const {
  if (!track_.active || track_.last_capture_ns == 0 ||
      capture_ns <= track_.last_capture_ns) {
    return 0.0F;
  }
  return static_cast<float>(
      static_cast<double>(capture_ns - track_.last_capture_ns) * 1.0e-9);
}

void SotByteTrack::initializeTrack(const circle::types::Detection& detection,
                                   circle::types::TimestampNs capture_ns) {
  track_ = Track{};
  track_.active = true;
  track_.id = next_track_id_++;
  track_.last_capture_ns = capture_ns;
  track_.last_detection = detection;
  track_.x.setZero();
  track_.x(0) = detection.cx;
  track_.x(1) = detection.cy;
  track_.x(2) = std::max(params_.min_box_size_px, detection.width);
  track_.x(3) = std::max(params_.min_box_size_px, detection.height);
  track_.p = Eigen::Matrix<float, 8, 8>::Identity();
  track_.p.diagonal() << 25.0F, 25.0F, 25.0F, 25.0F, 100.0F, 100.0F, 100.0F,
      100.0F;
}

void SotByteTrack::predict(float dt, bool freeze_scale) {
  Eigen::Matrix<float, 8, 8> f = Eigen::Matrix<float, 8, 8>::Identity();
  f(0, 4) = dt;
  f(1, 5) = dt;
  if (!freeze_scale) {
    f(2, 6) = dt;
    f(3, 7) = dt;
  } else {
    track_.x(6) = 0.0F;
    track_.x(7) = 0.0F;
  }

  Eigen::Matrix<float, 8, 8> q = Eigen::Matrix<float, 8, 8>::Zero();
  const float pos_q = 4.0F + 80.0F * dt;
  const float scale_q = freeze_scale ? 0.5F : 4.0F + 40.0F * dt;
  const float vel_q = 20.0F + 80.0F * dt;
  q.diagonal() << pos_q, pos_q, scale_q, scale_q, vel_q, vel_q, scale_q, scale_q;

  track_.x = f * track_.x;
  track_.p = f * track_.p * f.transpose() + q;
  track_.x(2) = std::max(params_.min_box_size_px, track_.x(2));
  track_.x(3) = std::max(params_.min_box_size_px, track_.x(3));
}

void SotByteTrack::correct(const circle::types::Detection& detection,
                           circle::types::TimestampNs capture_ns) {
  Eigen::Matrix<float, 4, 8> h = Eigen::Matrix<float, 4, 8>::Zero();
  h(0, 0) = 1.0F;
  h(1, 1) = 1.0F;
  h(2, 2) = 1.0F;
  h(3, 3) = 1.0F;

  Eigen::Matrix<float, 4, 1> z;
  z << detection.cx, detection.cy, std::max(params_.min_box_size_px, detection.width),
      std::max(params_.min_box_size_px, detection.height);

  Eigen::Matrix<float, 4, 4> r = Eigen::Matrix<float, 4, 4>::Zero();
  r.diagonal() << 16.0F, 16.0F, 25.0F, 25.0F;

  const Eigen::Matrix<float, 4, 1> y = z - h * track_.x;
  const Eigen::Matrix<float, 4, 4> s = h * track_.p * h.transpose() + r;
  const Eigen::Matrix<float, 8, 4> k = track_.p * h.transpose() * s.inverse();
  track_.x = track_.x + k * y;
  const Eigen::Matrix<float, 8, 8> i = Eigen::Matrix<float, 8, 8>::Identity();
  track_.p = (i - k * h) * track_.p;

  track_.last_capture_ns = capture_ns;
  track_.lost_frames = 0;
  track_.last_detection = detection;
  track_.x(2) = std::max(params_.min_box_size_px, track_.x(2));
  track_.x(3) = std::max(params_.min_box_size_px, track_.x(3));
}

circle::types::Detection SotByteTrack::predictedDetection(
    bool predicted,
    uint32_t image_width,
    uint32_t image_height) const {
  circle::types::Detection out = track_.last_detection;
  const float max_w = image_width > 0 ? static_cast<float>(image_width)
                                      : std::numeric_limits<float>::max();
  const float max_h = image_height > 0 ? static_cast<float>(image_height)
                                       : std::numeric_limits<float>::max();
  out.cx = image_width > 0
               ? clampFinite(track_.x(0), 0.0F, static_cast<float>(image_width))
               : track_.x(0);
  out.cy = image_height > 0
               ? clampFinite(track_.x(1), 0.0F, static_cast<float>(image_height))
               : track_.x(1);
  out.width = clampFinite(track_.x(2), params_.min_box_size_px, max_w);
  out.height = clampFinite(track_.x(3), params_.min_box_size_px, max_h);
  out.track_id = track_.id;
  out.tracker_predicted = predicted;
  out.tracker_lost_frames = predicted ? track_.lost_frames : 0;
  return out;
}

void SotByteTrack::destroyTrack() {
  track_ = Track{};
}

std::vector<SotByteTrack::Candidate> SotByteTrack::acceptedCandidates(
    const std::vector<circle::types::Detection>& detections,
    const DetectionFilterParams& filter,
    float min_score,
    float max_score_exclusive) const {
  std::vector<Candidate> out;
  out.reserve(detections.size());
  for (const auto& det : detections) {
    const float area = detectionArea(det);
    if (det.score < min_score || det.score >= max_score_exclusive ||
        !isAcceptedClass(det.class_name, filter.target_class_name,
                         filter.target_class_names) ||
        area < static_cast<float>(filter.min_bbox_area) ||
        detectionAspect(det) > static_cast<float>(filter.max_bbox_aspect_ratio)) {
      continue;
    }
    out.push_back(Candidate{det, area});
  }
  return out;
}

float SotByteTrack::boxIou(const circle::types::Detection& a,
                           const circle::types::Detection& b) {
  const float ax1 = a.cx - a.width * 0.5F;
  const float ay1 = a.cy - a.height * 0.5F;
  const float ax2 = a.cx + a.width * 0.5F;
  const float ay2 = a.cy + a.height * 0.5F;
  const float bx1 = b.cx - b.width * 0.5F;
  const float by1 = b.cy - b.height * 0.5F;
  const float bx2 = b.cx + b.width * 0.5F;
  const float by2 = b.cy + b.height * 0.5F;
  const float ix1 = std::max(ax1, bx1);
  const float iy1 = std::max(ay1, by1);
  const float ix2 = std::min(ax2, bx2);
  const float iy2 = std::min(ay2, by2);
  const float iw = std::max(0.0F, ix2 - ix1);
  const float ih = std::max(0.0F, iy2 - iy1);
  const float inter = iw * ih;
  const float area_a = std::max(0.0F, ax2 - ax1) * std::max(0.0F, ay2 - ay1);
  const float area_b = std::max(0.0F, bx2 - bx1) * std::max(0.0F, by2 - by1);
  const float denom = area_a + area_b - inter;
  return denom > 0.0F ? inter / denom : 0.0F;
}

}  // namespace circle::vision
