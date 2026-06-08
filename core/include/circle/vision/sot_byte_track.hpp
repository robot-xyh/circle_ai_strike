#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "circle/types/detection.hpp"
#include "circle/types/time.hpp"
#include "circle/vision/detection_filter.hpp"

namespace circle::vision {

struct SotByteTrackParams {
  bool enabled{false};
  float high_score_threshold{0.25F};
  float low_score_threshold{0.10F};
  float new_track_threshold{0.25F};
  float match_iou_threshold{0.30F};
  float second_match_iou_threshold{0.20F};
  int max_lost_frames{5};
  bool emit_prediction_on_miss{true};
  int emit_prediction_max_frames{5};
  float min_box_size_px{4.0F};
  float max_dt_s{0.12F};
};

class SotByteTrack {
 public:
  explicit SotByteTrack(SotByteTrackParams params = {});

  void setParams(const SotByteTrackParams& params);
  const SotByteTrackParams& params() const { return params_; }

  void reset();

  std::optional<circle::types::Detection> update(
      const std::vector<circle::types::Detection>& detections,
      const DetectionFilterParams& filter,
      circle::types::TimestampNs capture_ns,
      uint32_t image_width,
      uint32_t image_height);

 private:
  struct Track {
    bool active{false};
    int id{-1};
    circle::types::TimestampNs last_capture_ns{0};
    int lost_frames{0};
    circle::types::Detection last_detection{};
    Eigen::Matrix<float, 8, 1> x{Eigen::Matrix<float, 8, 1>::Zero()};
    Eigen::Matrix<float, 8, 8> p{Eigen::Matrix<float, 8, 8>::Identity()};
  };

  struct Candidate {
    circle::types::Detection detection{};
    float area{0.0F};
  };

  float computeDt(circle::types::TimestampNs capture_ns) const;
  void initializeTrack(const circle::types::Detection& detection,
                       circle::types::TimestampNs capture_ns);
  void predict(float dt, bool freeze_scale);
  void correct(const circle::types::Detection& detection,
               circle::types::TimestampNs capture_ns);
  circle::types::Detection predictedDetection(bool predicted,
                                              uint32_t image_width,
                                              uint32_t image_height) const;
  void destroyTrack();

  std::vector<Candidate> acceptedCandidates(
      const std::vector<circle::types::Detection>& detections,
      const DetectionFilterParams& filter,
      float min_score,
      float max_score_exclusive =
          std::numeric_limits<float>::infinity()) const;
  static float boxIou(const circle::types::Detection& a,
                      const circle::types::Detection& b);

  SotByteTrackParams params_{};
  Track track_{};
  int next_track_id_{1};
  circle::types::TimestampNs last_processed_capture_ns_{0};
};

}  // namespace circle::vision
