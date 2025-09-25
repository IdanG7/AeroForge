#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>

namespace af
{

struct Telemetry
{
  double timestamp_s{0.0};
  Eigen::Vector3d position_m{0, 0, 0}; // NED or local frame
  Eigen::Vector3d velocity_mps{0, 0, 0};
  Eigen::Vector3d euler_deg{0, 0, 0}; // roll, pitch, yaw
  double altitude_m{0.0};
  double speed_mps{0.0};
  double yaw_deg{0.0};
  bool gps_valid{false};
  uint8_t satellite_count{0};
  uint8_t flight_mode{0}; // platform-specific
};

} // namespace af
