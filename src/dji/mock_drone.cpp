#include "aeroforge/dji_iface.hpp"
#include <spdlog/spdlog.h>

namespace af
{

class MockDrone : public IDrone
{
public:
  MockDrone() = default;

  bool connect() override
  {
    spdlog::info("[MockDrone] Connected");
    connected_ = true;
    return true;
  }

  bool disconnect() override
  {
    spdlog::info("[MockDrone] Disconnected");
    connected_ = false;
    return true;
  }

  bool is_connected() const override { return connected_; }

  Telemetry read_telemetry() override
  {
    // Return dummy telemetry
    Telemetry tel;
    tel.timestamp_s = 0.0;
    tel.position_m = Eigen::Vector3d(0, 0, -10); // 10m altitude
    tel.velocity_mps = Eigen::Vector3d(0, 0, 0);
    tel.euler_deg = Eigen::Vector3d(0, 0, 0);
    tel.altitude_m = 10.0;
    tel.speed_mps = 0.0;
    tel.yaw_deg = 0.0;
    tel.gps_valid = true;
    tel.satellite_count = 12;
    tel.flight_mode = 0;
    return tel;
  }

  bool send_velocity_cmd(const Eigen::Vector3d& vel_ned_mps) override
  {
    spdlog::debug("[MockDrone] Velocity command: [{:.2f}, {:.2f}, {:.2f}] m/s",
                  vel_ned_mps.x(), vel_ned_mps.y(), vel_ned_mps.z());
    last_cmd_ = vel_ned_mps;
    return true;
  }

  void hold() override
  {
    spdlog::info("[MockDrone] HOLD command (hover)");
    last_cmd_ = Eigen::Vector3d::Zero();
  }

  const char* name() const override { return "MockDrone"; }

private:
  bool connected_{false};
  Eigen::Vector3d last_cmd_{0, 0, 0};
};

std::unique_ptr<IDrone> make_mock_drone()
{
  return std::make_unique<MockDrone>();
}

} // namespace af
