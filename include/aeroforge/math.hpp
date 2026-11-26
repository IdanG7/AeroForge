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

  // `dt` must be finite and > 0. With an invalid `dt` the integral and derivative terms
  // are skipped for this call (P + previous I only). A non-finite `error` returns zero
  // and leaves the controller state untouched.
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
// Coordinate frames
// ──────────────────────────────────────────────────────────────────────────
//
// camera: X right, Y down, Z forward (optical axis)   - OpenCV convention
// body:   X forward, Y right, Z down (FRD)            - aircraft convention
// NED:    X north, Y east, Z down                     - local navigation frame
//
// camera_to_body assumes a forward-looking camera with no mount rotation (gimbal level
// and aligned with the airframe). Gimbal pitch is NOT compensated here.

Eigen::Vector3d camera_to_body(const Eigen::Vector3d& v_cam);

// Rotate a body-frame vector into NED using heading only (roll and pitch assumed zero).
Eigen::Vector3d body_to_ned(const Eigen::Vector3d& v_body, double yaw_deg);

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
