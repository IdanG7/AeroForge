#include "aeroforge/dji_iface.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace af
{
namespace
{

class MockDrone : public IDrone
{
public:
  bool connect() override
  {
    spdlog::info("[MockDrone] Connected (simulation)");
    connected_ = true;
    return true;
  }

  bool disconnect() override
  {
    spdlog::info("[MockDrone] Disconnected");
    connected_ = false;
    cmd_ned_.setZero();
    return true;
  }

  bool is_connected() const override { return connected_; }

  DroneCapabilities capabilities() const override
  {
    return {.simulated = true, .telemetry = true, .velocity_control = true};
  }

  Telemetry read_telemetry() override
  {
    Telemetry tel;
    tel.simulated = true;
    if (!connected_)
    {
      return tel; // validity stays Unavailable
    }

    tel.validity = Validity::Fresh;
    tel.timestamp_s = sim_time_s_;
    tel.position_m = position_ned_;
    tel.velocity_mps = velocity_ned_;
    tel.euler_deg = Eigen::Vector3d(0, 0, yaw_deg_);
    tel.altitude_m = -position_ned_.z();
    tel.speed_mps = velocity_ned_.norm();
    tel.yaw_deg = yaw_deg_;
    tel.gps_valid = true;
    tel.satellite_count = 12;
    tel.flight_mode = 0;
    return tel;
  }

  bool send_velocity_cmd(const Eigen::Vector3d& vel_ned_mps) override
  {
    if (!connected_ || !vel_ned_mps.allFinite())
    {
      return false;
    }
    cmd_ned_ = vel_ned_mps;
    last_cmd_time_s_ = sim_time_s_;
    return true;
  }

  void hold() override
  {
    spdlog::info("[MockDrone] HOLD command (hover)");
    cmd_ned_.setZero();
  }

  void step(double dt_s) override
  {
    if (!connected_ || !std::isfinite(dt_s) || dt_s <= 0.0)
    {
      return;
    }
    // Integrate in bounded sub-steps so a long frame gap cannot destabilise the lag model
    double remaining = std::min(dt_s, kMaxStepS);
    while (remaining > 0.0)
    {
      const double h = std::min(remaining, kSubStepS);
      remaining -= h;
      sim_time_s_ += h;

      if (sim_time_s_ - last_cmd_time_s_ > kCmdTimeoutS)
      {
        cmd_ned_.setZero(); // command watchdog: brake to hover
      }

      const double alpha = h / (kVelocityLagS + h);
      velocity_ned_ += alpha * (cmd_ned_ - velocity_ned_);
      position_ned_ += velocity_ned_ * h;
    }
  }

  const char* name() const override { return "MockDrone"; }

private:
  static constexpr double kVelocityLagS = 0.3;
  static constexpr double kCmdTimeoutS = 0.5;
  static constexpr double kSubStepS = 0.01;
  static constexpr double kMaxStepS = 1.0;

  bool connected_{false};
  double sim_time_s_{0.0};
  double last_cmd_time_s_{-1.0e9};
  double yaw_deg_{0.0};
  Eigen::Vector3d position_ned_{0, 0, -10}; // 10 m above home
  Eigen::Vector3d velocity_ned_{0, 0, 0};
  Eigen::Vector3d cmd_ned_{0, 0, 0};
};

} // namespace

std::unique_ptr<IDrone> make_mock_drone()
{
  return std::make_unique<MockDrone>();
}

} // namespace af
