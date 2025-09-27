#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// PID Controller (per-axis)
// ──────────────────────────────────────────────────────────────────────────

class PIDController
{
public:
  PIDController(const Eigen::Vector3d& kp, const Eigen::Vector3d& ki, const Eigen::Vector3d& kd,
                double max_output);

  Eigen::Vector3d compute(const Eigen::Vector3d& error, double dt);
  void reset();

private:
  Eigen::Vector3d kp_;
  Eigen::Vector3d ki_;
  Eigen::Vector3d kd_;
  double max_output_;
  Eigen::Vector3d integral_{0, 0, 0};
  Eigen::Vector3d prev_error_{0, 0, 0};
  bool first_call_{true};
};

// ──────────────────────────────────────────────────────────────────────────
// Kalman Filter (1D or 3D constant velocity model)
// ──────────────────────────────────────────────────────────────────────────

class KalmanFilter1D
{
public:
  KalmanFilter1D(double q, double r);

  void init(double x0, double v0 = 0.0);
  void predict(double dt);
  void update(double measurement);
  double state() const { return x_(0); }
  double velocity() const { return x_(1); }

private:
  Eigen::Vector2d x_; // [position, velocity]
  Eigen::Matrix2d P_; // covariance
  double q_;          // process noise
  double r_;          // measurement noise
};

class KalmanFilter3D
{
public:
  KalmanFilter3D(double q, double r);

  void init(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel = Eigen::Vector3d::Zero());
  void predict(double dt);
  void update(const Eigen::Vector3d& measurement);
  Eigen::Vector3d position() const { return x_.head<3>(); }
  Eigen::Vector3d velocity() const { return x_.tail<3>(); }

private:
  Eigen::Matrix<double, 6, 1> x_; // [px,py,pz,vx,vy,vz]
  Eigen::Matrix<double, 6, 6> P_; // covariance
  double q_;
  double r_;
};

} // namespace af
