#pragma once

#include "telemetry.hpp"

#include <Eigen/Core>
#include <memory>

namespace af
{

// What a vehicle backend can do. Callers must check these instead of assuming: a backend
// that cannot do something says so here rather than silently ignoring the request.
struct DroneCapabilities
{
  bool simulated{false};        // true: no physical aircraft is involved
  bool telemetry{false};        // read_telemetry() returns real data
  bool velocity_control{false}; // send_velocity_cmd() has an effect
};

// ──────────────────────────────────────────────────────────────────────────
// Abstract Drone Interface
// ──────────────────────────────────────────────────────────────────────────

class IDrone
{
public:
  virtual ~IDrone() = default;

  virtual bool connect() = 0;
  virtual bool disconnect() = 0;
  virtual bool is_connected() const = 0;

  virtual DroneCapabilities capabilities() const = 0;

  // Telemetry::validity is Unavailable when not connected.
  virtual Telemetry read_telemetry() = 0;

  // Velocity in the local NED frame, m/s. Returns false if the command was not accepted
  // (not connected, not supported, non-finite).
  virtual bool send_velocity_cmd(const Eigen::Vector3d& vel_ned_mps) = 0;
  virtual void hold() = 0; // hover/brake

  // Advance simulated time. No-op for physical vehicles.
  virtual void step(double dt_s) { (void)dt_s; }

  virtual const char* name() const = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Factory (gated by build flag AEROFORGE_WITH_DJI)
// ──────────────────────────────────────────────────────────────────────────

#ifdef AEROFORGE_WITH_DJI
std::unique_ptr<IDrone> make_dji_drone();
#endif

// Kinematic simulator: velocity follows the last command with a first-order lag and
// position integrates velocity. Starts hovering 10 m above the home point. If no command
// arrives for 0.5 s of simulated time the vehicle brakes to a hover.
std::unique_ptr<IDrone> make_mock_drone();

} // namespace af
