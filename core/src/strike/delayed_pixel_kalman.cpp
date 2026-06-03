#include "circle/strike/delayed_pixel_kalman.hpp"

#include <algorithm>
#include <cmath>

#include "circle/types/time.hpp"

namespace circle::strike {

void DelayedPixelKalman::reset() {
  valid_ = false;
  state_stamp_ns_ = 0;
  last_measurement_stamp_ns_ = 0;
  x_.setZero();
  P_.setIdentity();
}

void DelayedPixelKalman::predictInPlace(Eigen::Matrix<float, 4, 1>& x,
                                        Eigen::Matrix<float, 4, 4>& P,
                                        float dt_s,
                                        float accel_noise) {
  const float dt = std::clamp(dt_s, 0.0F, 0.5F);
  if (dt <= 0.0F) {
    return;
  }

  Eigen::Matrix<float, 4, 4> F = Eigen::Matrix<float, 4, 4>::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  const float q = std::max(0.0F, accel_noise);
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;
  const float dt4 = dt2 * dt2;
  Eigen::Matrix<float, 4, 4> Q = Eigen::Matrix<float, 4, 4>::Zero();
  Q(0, 0) = 0.25F * dt4 * q;
  Q(1, 1) = 0.25F * dt4 * q;
  Q(0, 2) = 0.5F * dt3 * q;
  Q(2, 0) = Q(0, 2);
  Q(1, 3) = 0.5F * dt3 * q;
  Q(3, 1) = Q(1, 3);
  Q(2, 2) = dt2 * q;
  Q(3, 3) = dt2 * q;

  x = F * x;
  P = F * P * F.transpose() + Q;
}

bool DelayedPixelKalman::addMeasurement(const Measurement& m,
                                        const circle::types::CameraIntrinsics& intr,
                                        const Params& p) {
  if (!std::isfinite(m.ex) || !std::isfinite(m.ey) || intr.fx <= 1.0F ||
      intr.fy <= 1.0F) {
    return false;
  }

  if (valid_ && m.image_stamp_ns <= last_measurement_stamp_ns_) {
    return false;
  }

  if (!valid_) {
    valid_ = true;
    state_stamp_ns_ = m.image_stamp_ns;
    last_measurement_stamp_ns_ = m.image_stamp_ns;
    x_ << m.ex, m.ey, 0.0F, 0.0F;
    P_.setZero();
    P_(0, 0) = 1.0e-4F;
    P_(1, 1) = 1.0e-4F;
    P_(2, 2) = 0.05F;
    P_(3, 3) = 0.05F;
    return true;
  }

  const float dt = static_cast<float>(
      circle::types::secondsBetween(state_stamp_ns_, m.image_stamp_ns));
  predictInPlace(x_, P_, dt, p.process_accel_noise);
  state_stamp_ns_ = m.image_stamp_ns;
  last_measurement_stamp_ns_ = m.image_stamp_ns;

  const float area_scale =
      std::clamp(400.0F / std::max(1.0F, m.bbox_area_px), 0.25F, 6.0F);
  const float score_scale =
      std::clamp(1.0F / std::max(0.05F, m.score), 1.0F, 4.0F);
  const float sigma_px =
      std::max(0.1F, p.meas_noise_px) * area_scale * score_scale;
  Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero();
  R(0, 0) = (sigma_px / intr.fx) * (sigma_px / intr.fx);
  R(1, 1) = (sigma_px / intr.fy) * (sigma_px / intr.fy);

  Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
  H(0, 0) = 1.0F;
  H(1, 1) = 1.0F;
  Eigen::Matrix<float, 2, 1> z;
  z << m.ex, m.ey;
  const Eigen::Matrix<float, 2, 1> y = z - H * x_;
  const Eigen::Matrix<float, 2, 2> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<float, 4, 2> K = P_ * H.transpose() * S.inverse();
  x_ += K * y;
  const Eigen::Matrix<float, 4, 4> I = Eigen::Matrix<float, 4, 4>::Identity();
  P_ = (I - K * H) * P_;
  return true;
}

DelayedPixelKalman::Estimate DelayedPixelKalman::predict(
    circle::types::TimestampNs now_ns, const Params& p) const {
  Estimate out;
  if (!valid_) {
    return out;
  }

  Eigen::Matrix<float, 4, 1> x = x_;
  Eigen::Matrix<float, 4, 4> P = P_;
  float dt = static_cast<float>(circle::types::secondsBetween(state_stamp_ns_, now_ns));
  dt += std::max(0.0F, p.predict_extra_delay_s);
  predictInPlace(x, P, dt, p.process_accel_noise);

  out.valid = P.trace() <= std::max(1.0e-6F, p.max_cov_trace);
  out.stamp_ns = now_ns;
  out.ex = x(0);
  out.ey = x(1);
  out.ex_dot = x(2);
  out.ey_dot = x(3);
  out.cov_trace = P.trace();
  return out;
}

}  // namespace circle::strike
