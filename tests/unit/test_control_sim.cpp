// Closed-loop checks of the follow controller against the built-in kinematic simulator.
// No aircraft, no camera: the "observation" is computed from simulator ground truth.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include "aeroforge/dji_iface.hpp"
#include "aeroforge/math.hpp"
#include "aeroforge/modules.hpp"
#include <algorithm>
#include <limits>

using Catch::Approx;

namespace
{

// Camera-frame position of a NED point seen from a level vehicle heading north
af::Pose3D observe(const Eigen::Vector3d& target_ned, const af::Telemetry& tel, double t)
{
  const Eigen::Vector3d rel = target_ned - tel.position_m; // body == NED at yaw 0
  af::Pose3D pose;
  pose.position_m = Eigen::Vector3d(rel.y(), rel.z(), rel.x()); // right, down, forward
  pose.timestamp_s = t;
  pose.validity = af::Validity::Fresh;
  return pose;
}

std::unique_ptr<af::IController> make_controller(double max_speed = 3.0)
{
  return af::make_pid_velocity_controller(
      Eigen::Vector3d(-3.0, 0.0, 0.0), Eigen::Vector3d(0.8, 0.8, 0.6), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.1, 0.1, 0.08), max_speed);
}

} // namespace

TEST_CASE("Mock drone simulator basics", "[sim]")
{
  auto drone = af::make_mock_drone();

  SECTION("No telemetry and no commands while disconnected")
  {
    REQUIRE(drone->read_telemetry().validity == af::Validity::Unavailable);
    REQUIRE_FALSE(drone->send_velocity_cmd(Eigen::Vector3d(1, 0, 0)));
  }

  drone->connect();
  REQUIRE(drone->capabilities().simulated);
  REQUIRE(drone->read_telemetry().simulated);
  REQUIRE(drone->read_telemetry().validity == af::Validity::Fresh);
  REQUIRE(drone->read_telemetry().altitude_m == Approx(10.0));

  SECTION("Non-finite commands are refused")
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(drone->send_velocity_cmd(Eigen::Vector3d(nan, 0, 0)));
  }

  SECTION("Velocity follows the command and position integrates it")
  {
    for (int i = 0; i < 300; ++i) // 3 s at 100 Hz
    {
      REQUIRE(drone->send_velocity_cmd(Eigen::Vector3d(2.0, 0, 0)));
      drone->step(0.01);
    }
    const auto tel = drone->read_telemetry();
    REQUIRE(tel.velocity_mps.x() == Approx(2.0).margin(0.01));
    REQUIRE(tel.position_m.x() > 4.5);
    REQUIRE(tel.position_m.x() < 6.0);
    REQUIRE(tel.timestamp_s == Approx(3.0));
  }

  SECTION("Command watchdog brakes to a hover when commands stop")
  {
    for (int i = 0; i < 200; ++i)
    {
      drone->send_velocity_cmd(Eigen::Vector3d(2.0, 0, 0));
      drone->step(0.01);
    }
    for (int i = 0; i < 300; ++i) // 3 s with no commands
    {
      drone->step(0.01);
    }
    REQUIRE(drone->read_telemetry().speed_mps < 0.01);
  }

  SECTION("Invalid time steps are ignored")
  {
    const double x0 = drone->read_telemetry().position_m.x();
    drone->send_velocity_cmd(Eigen::Vector3d(2.0, 0, 0));
    drone->step(-1.0);
    drone->step(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(drone->read_telemetry().position_m.x() == Approx(x0));
  }
}

TEST_CASE("PID follow controller converges in simulation", "[sim][control]")
{
  auto drone = af::make_mock_drone();
  drone->connect();
  auto controller = make_controller();

  // Stationary target 10 m north, 2 m east, 1 m below the vehicle's altitude
  const Eigen::Vector3d target(10.0, 2.0, -9.0);
  const double dt = 1.0 / 30.0;
  double max_speed_seen = 0.0;

  for (int i = 0; i < 30 * 30; ++i) // 30 s
  {
    const auto tel = drone->read_telemetry();
    const auto cmd = controller->velocity_cmd(observe(target, tel, i * dt), tel);
    REQUIRE(cmd.has_value());
    REQUIRE(cmd->allFinite());
    REQUIRE(cmd->norm() <= 3.0 + 1e-9);
    drone->send_velocity_cmd(af::body_to_ned(*cmd, tel.yaw_deg));
    drone->step(dt);
    max_speed_seen = std::max(max_speed_seen, drone->read_telemetry().speed_mps);
  }

  // Expected resting point: 3 m behind (south of) the target, level with it
  const auto tel = drone->read_telemetry();
  REQUIRE(tel.position_m.x() == Approx(7.0).margin(0.05));
  REQUIRE(tel.position_m.y() == Approx(2.0).margin(0.05));
  REQUIRE(tel.position_m.z() == Approx(-9.0).margin(0.05));
  REQUIRE(tel.speed_mps < 0.05);
  REQUIRE(max_speed_seen <= 3.0 + 1e-6);
}

TEST_CASE("PID follow controller input validation", "[control]")
{
  auto controller = make_controller();
  af::Telemetry tel;

  af::Pose3D pose;
  pose.position_m = Eigen::Vector3d(0, 0, 5);
  pose.timestamp_s = 1.0;

  SECTION("Only fresh poses produce commands")
  {
    for (auto v : {af::Validity::Predicted, af::Validity::Stale, af::Validity::Invalid,
                   af::Validity::Unavailable})
    {
      pose.validity = v;
      REQUIRE_FALSE(controller->velocity_cmd(pose, tel).has_value());
    }
  }

  SECTION("Non-finite poses produce no command")
  {
    pose.position_m.x() = std::numeric_limits<double>::infinity();
    REQUIRE_FALSE(controller->velocity_cmd(pose, tel).has_value());
  }

  SECTION("Direction: target further ahead than desired means fly forward")
  {
    const auto cmd = controller->velocity_cmd(pose, tel); // 5 m ahead, want 3 m
    REQUIRE(cmd.has_value());
    REQUIRE(cmd->x() > 0.0);
    REQUIRE(cmd->y() == Approx(0.0).margin(1e-12));
    REQUIRE(cmd->z() == Approx(0.0).margin(1e-12));
  }

  SECTION("Repeated or backwards timestamps do not blow up the derivative")
  {
    REQUIRE(controller->velocity_cmd(pose, tel).has_value());
    pose.position_m.z() = 6.0;
    auto same_time = controller->velocity_cmd(pose, tel); // dt == 0
    REQUIRE(same_time.has_value());
    REQUIRE(same_time->allFinite());
    REQUIRE(same_time->norm() <= 3.0 + 1e-9);
    pose.timestamp_s = 0.5; // time went backwards
    auto backwards = controller->velocity_cmd(pose, tel);
    REQUIRE(backwards.has_value());
    REQUIRE(backwards->x() == Approx(0.8 * 3.0)); // proportional only: kp * (6 - 3)
  }

  SECTION("Bad parameters are rejected with an exception")
  {
    const Eigen::Vector3d z = Eigen::Vector3d::Zero();
    REQUIRE_THROWS_AS(af::make_pid_velocity_controller(z, z, z, z, 0.0), std::invalid_argument);
    REQUIRE_THROWS_AS(
        af::make_pid_velocity_controller(z, Eigen::Vector3d(-1, 0, 0), z, z, 3.0),
        std::invalid_argument);
  }
}
