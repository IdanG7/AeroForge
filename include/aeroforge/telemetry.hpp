#pragma once

#include "validity.hpp"
#include <Eigen/Core>
#include <cstdint>
#include <string>

namespace af
{

struct Telemetry
{
  // Defaults describe "no telemetry": consumers must check `validity` first.
  Validity validity{Validity::Unavailable};
  bool simulated{false};               // true when produced by a simulator, never an aircraft
  double timestamp_s{0.0};             // monotonic seconds (af::now_seconds or simulation time)
  Eigen::Vector3d position_m{0, 0, 0}; // local NED relative to the home point
  Eigen::Vector3d velocity_mps{0, 0, 0};
  Eigen::Vector3d euler_deg{0, 0, 0}; // roll, pitch, yaw
  double altitude_m{0.0};             // height above the home point, positive up
  double speed_mps{0.0};
  double yaw_deg{0.0};
  bool gps_valid{false};
  uint8_t satellite_count{0};
  uint8_t flight_mode{0}; // platform-specific
};

} // namespace af
