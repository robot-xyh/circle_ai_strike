#pragma once

#include <limits>

#include <Eigen/Dense>

#include "circle/types/detection.hpp"
#include "circle/types/time.hpp"

namespace circle::strike {

class DelayedPixelKalman {
 public:
  struct Params {
    bool enable{true};
    float process_accel_noise{4.0F};
    float meas_noise_px{4.0F};
    float predict_extra_delay_s{0.03F};
    float max_cov_trace{0.25F};
  };

  struct Measurement {
    circle::types::TimestampNs image_stamp_ns{0};
    circle::types::TimestampNs receive_stamp_ns{0};
    float ex{0.0F};
    float ey{0.0F};
    float bbox_area_px{0.0F};
    float score{0.0F};
  };

  struct Estimate {
    bool valid{false};
    circle::types::TimestampNs stamp_ns{0};
    float ex{0.0F};
    float ey{0.0F};
    float ex_dot{0.0F};
    float ey_dot{0.0F};
    float cov_trace{0.0F};
  };

  void reset();
  bool addMeasurement(const Measurement& m,
                      const circle::types::CameraIntrinsics& intr,
                      const Params& p);
  [[nodiscard]] Estimate predict(circle::types::TimestampNs now_ns,
                                 const Params& p) const;

 private:
  static void predictInPlace(Eigen::Matrix<float, 4, 1>& x,
                             Eigen::Matrix<float, 4, 4>& P,
                             float dt_s,
                             float accel_noise);

  bool valid_{false};
  circle::types::TimestampNs state_stamp_ns_{0};
  circle::types::TimestampNs last_measurement_stamp_ns_{0};
  Eigen::Matrix<float, 4, 1> x_{Eigen::Matrix<float, 4, 1>::Zero()};
  Eigen::Matrix<float, 4, 4> P_{Eigen::Matrix<float, 4, 4>::Identity()};
};

}  // namespace circle::strike
