#pragma once

#include "telemetry.hpp"
#include <Eigen/Core>
#include <memory>

namespace af
{

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

  virtual Telemetry read_telemetry() = 0;
  virtual bool send_velocity_cmd(const Eigen::Vector3d& vel_ned_mps) = 0;
  virtual void hold() = 0; // hover/brake

  virtual const char* name() const = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Factory (gated by build flag AEROFORGE_WITH_DJI)
// ──────────────────────────────────────────────────────────────────────────

#ifdef AEROFORGE_WITH_DJI
std::unique_ptr<IDrone> make_dji_drone();
#endif

std::unique_ptr<IDrone> make_mock_drone();

} // namespace af
