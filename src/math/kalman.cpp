#include "aeroforge/math.hpp"

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// 1D Kalman Filter
// ──────────────────────────────────────────────────────────────────────────

KalmanFilter1D::KalmanFilter1D(double q, double r) : q_(q), r_(r)
{
  x_.setZero();
  P_ = Eigen::Matrix2d::Identity() * 1.0;
}

void KalmanFilter1D::init(double x0, double v0)
{
  x_(0) = x0;
  x_(1) = v0;
  P_ = Eigen::Matrix2d::Identity() * 1.0;
}

void KalmanFilter1D::predict(double dt)
{
  // State transition: x_k = F * x_{k-1}
  // F = [1, dt]
  //     [0,  1]
  Eigen::Matrix2d F;
  F << 1.0, dt, 0.0, 1.0;

  x_ = F * x_;

  // Covariance: P_k = F * P_{k-1} * F^T + Q
  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * q_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter1D::update(double measurement)
{
  // Measurement matrix H = [1, 0] (we measure position only)
  Eigen::Vector2d H(1.0, 0.0);

  // Innovation: y = z - H * x
  double y = measurement - H.dot(x_);

  // Innovation covariance: S = H * P * H^T + R
  double S = H.dot(P_ * H) + r_;

  // Kalman gain: K = P * H^T * S^{-1}
  Eigen::Vector2d K = P_ * H / S;

  // State update: x = x + K * y
  x_ = x_ + K * y;

  // Covariance update: P = (I - K * H) * P
  Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
  P_ = (I - K * H.transpose()) * P_;
}

// ──────────────────────────────────────────────────────────────────────────
// 3D Kalman Filter (constant velocity model)
// ──────────────────────────────────────────────────────────────────────────

KalmanFilter3D::KalmanFilter3D(double q, double r) : q_(q), r_(r)
{
  x_.setZero();
  P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;
}

void KalmanFilter3D::init(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel)
{
  x_.head<3>() = pos;
  x_.tail<3>() = vel;
  P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;
}

void KalmanFilter3D::predict(double dt)
{
  // State transition matrix F (6x6)
  // [I3, dt*I3]
  // [0,  I3   ]
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

  x_ = F * x_;

  // Process noise Q
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity() * q_;

  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter3D::update(const Eigen::Vector3d& measurement)
{
  // Measurement matrix H (3x6): we measure position only
  // H = [I3, 0]
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  // Innovation: y = z - H * x
  Eigen::Vector3d y = measurement - H * x_;

  // Innovation covariance: S = H * P * H^T + R
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * r_;
  Eigen::Matrix3d S = H * P_ * H.transpose() + R;

  // Kalman gain: K = P * H^T * S^{-1}
  Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

  // State update
  x_ = x_ + K * y;

  // Covariance update
  Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
  P_ = (I - K * H) * P_;
}

} // namespace af
