#include "aeroforge/math.hpp"
#include <cmath>

namespace af
{

Eigen::Vector3d camera_to_body(const Eigen::Vector3d& v_cam)
{
  // body X (forward) = camera Z, body Y (right) = camera X, body Z (down) = camera Y
  return {v_cam.z(), v_cam.x(), v_cam.y()};
}

Eigen::Vector3d body_to_ned(const Eigen::Vector3d& v_body, double yaw_deg)
{
  const double pi = 3.14159265358979323846;
  const double yaw = yaw_deg * pi / 180.0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v_body.x() - s * v_body.y(), s * v_body.x() + c * v_body.y(), v_body.z()};
}

} // namespace af
