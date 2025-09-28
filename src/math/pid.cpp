#include "aeroforge/math.hpp"
#include <algorithm>

namespace af
{

PIDController::PIDController(const Eigen::Vector3d& kp, const Eigen::Vector3d& ki,
                             const Eigen::Vector3d& kd, double max_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output)
{
}

Eigen::Vector3d PIDController::compute(const Eigen::Vector3d& error, double dt)
{
  if (first_call_)
  {
    prev_error_ = error;
    first_call_ = false;
  }

  // Integral with anti-windup (simple clamping)
  integral_ += error * dt;
  for (int i = 0; i < 3; ++i)
  {
    integral_(i) = std::clamp(integral_(i), -max_output_, max_output_);
  }

  // Derivative
  Eigen::Vector3d derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  // PID output
  Eigen::Vector3d output = kp_.cwiseProduct(error) + ki_.cwiseProduct(integral_) +
                           kd_.cwiseProduct(derivative);

  // Clamp per-axis
  for (int i = 0; i < 3; ++i)
  {
    output(i) = std::clamp(output(i), -max_output_, max_output_);
  }

  return output;
}

void PIDController::reset()
{
  integral_.setZero();
  prev_error_.setZero();
  first_call_ = true;
}

} // namespace af
