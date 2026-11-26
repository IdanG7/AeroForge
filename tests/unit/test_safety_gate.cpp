#include "aeroforge/safety.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <string>

using Catch::Approx;
using State = af::SafetyGate::State;

namespace
{

af::SafetyConfig config(bool hold)
{
  af::SafetyConfig c;
  c.require_hold_to_enable = hold;
  c.max_speed_mps = 2.0;
  c.min_follow_dist_m = 2.0;
  c.max_follow_dist_m = 10.0;
  c.min_alt_m = 3.0;
  c.max_alt_m = 60.0;
  c.max_radius_m = 120.0;
  c.stale_timeout_s = 0.5;
  return c;
}

af::DroneCapabilities sim_caps()
{
  af::DroneCapabilities caps;
  caps.simulated = true;
  caps.telemetry = true;
  caps.velocity_control = true;
  return caps;
}

af::Telemetry good_telemetry()
{
  af::Telemetry tel;
  tel.validity = af::Validity::Fresh;
  tel.simulated = true;
  tel.altitude_m = 10.0;
  tel.position_m = Eigen::Vector3d(0, 0, -10);
  return tel;
}

af::Pose3D good_target(double t = 100.0)
{
  af::Pose3D pose;
  pose.position_m = Eigen::Vector3d(0, 0, 5);
  pose.timestamp_s = t;
  pose.validity = af::Validity::Fresh;
  return pose;
}

const Eigen::Vector3d kCmd(1.0, 0.0, 0.0);

} // namespace

TEST_CASE("SafetyGate hold-to-enable", "[safety]")
{
  af::SafetyGate gate(config(true));
  REQUIRE(gate.state() == State::Disabled);

  gate.update(true, false);
  REQUIRE(gate.state() == State::Enabled);
  gate.update(true, false); // still held
  REQUIRE(gate.state() == State::Enabled);
  gate.update(false, false); // released
  REQUIRE(gate.state() == State::Disabled);

  SECTION("E-stop latches while the enable key stays down")
  {
    gate.update(true, false);
    gate.update(true, true);
    REQUIRE(gate.state() == State::EStopped);
    gate.update(true, false); // e-stop released, enable still held: must NOT re-enable
    REQUIRE(gate.state() == State::EStopped);
    gate.update(false, false);
    REQUIRE(gate.state() == State::Disabled);
    gate.update(true, false); // deliberate new press
    REQUIRE(gate.state() == State::Enabled);
  }
}

TEST_CASE("SafetyGate toggle mode", "[safety]")
{
  af::SafetyGate gate(config(false));

  gate.update(true, false);
  REQUIRE(gate.state() == State::Enabled);
  gate.update(true, false); // key held across frames must not flip the state again
  REQUIRE(gate.state() == State::Enabled);
  gate.update(false, false);
  REQUIRE(gate.state() == State::Enabled);
  gate.update(true, false);
  REQUIRE(gate.state() == State::Disabled);
  gate.update(false, false);

  SECTION("After an e-stop the first press only clears the latch")
  {
    gate.update(true, false);
    gate.update(false, false);
    REQUIRE(gate.state() == State::Enabled);
    gate.update(false, true);
    REQUIRE(gate.state() == State::EStopped);
    gate.update(false, false);
    REQUIRE(gate.state() == State::EStopped);
    gate.update(true, false);
    REQUIRE(gate.state() == State::Disabled);
    gate.update(false, false);
    gate.update(true, false);
    REQUIRE(gate.state() == State::Enabled);
  }
}

TEST_CASE("SafetyGate command evaluation", "[safety]")
{
  af::SafetyGate gate(config(true));
  const auto tel = good_telemetry();
  const auto target = good_target();
  const double now = 100.1;

  SECTION("Nothing passes while disabled")
  {
    const auto r = gate.evaluate(kCmd, target, tel, sim_caps(), now);
    REQUIRE_FALSE(r.cmd.has_value());
    REQUIRE(std::string(r.reason) == "operator has not enabled control");
  }

  gate.update(true, false);
  REQUIRE(gate.state() == State::Enabled);

  SECTION("A valid command passes unchanged")
  {
    const auto r = gate.evaluate(kCmd, target, tel, sim_caps(), now);
    REQUIRE(r.cmd.has_value());
    REQUIRE(r.cmd->x() == Approx(1.0));
    REQUIRE(std::string(r.reason).empty());
  }

  SECTION("Speed is limited, direction preserved")
  {
    const auto r = gate.evaluate(Eigen::Vector3d(6.0, 8.0, 0.0), target, tel, sim_caps(), now);
    REQUIRE(r.cmd.has_value());
    REQUIRE(r.cmd->norm() == Approx(2.0));
    REQUIRE(r.cmd->x() / r.cmd->y() == Approx(6.0 / 8.0));
  }

  SECTION("A physical (non-simulated) backend is always refused")
  {
    af::DroneCapabilities real = sim_caps();
    real.simulated = false;
    const auto r = gate.evaluate(kCmd, target, tel, real, now);
    REQUIRE_FALSE(r.cmd.has_value());
    REQUIRE(std::string(r.reason) == "actuation is simulation-only");
  }

  SECTION("A backend without velocity control is refused")
  {
    af::DroneCapabilities caps = sim_caps();
    caps.velocity_control = false;
    REQUIRE_FALSE(gate.evaluate(kCmd, target, tel, caps, now).cmd.has_value());
  }

  SECTION("E-stop refuses")
  {
    gate.update(true, true);
    const auto r = gate.evaluate(kCmd, target, tel, sim_caps(), now);
    REQUIRE_FALSE(r.cmd.has_value());
    REQUIRE(std::string(r.reason) == "e-stop latched");
  }

  SECTION("Telemetry that is not fresh refuses")
  {
    af::Telemetry bad = tel;
    bad.validity = af::Validity::Stale;
    REQUIRE_FALSE(gate.evaluate(kCmd, target, bad, sim_caps(), now).cmd.has_value());
    REQUIRE_FALSE(gate.evaluate(kCmd, target, af::Telemetry{}, sim_caps(), now).cmd.has_value());
  }

  SECTION("Geofence: altitude and radius")
  {
    af::Telemetry low = tel;
    low.altitude_m = 1.0;
    REQUIRE_FALSE(gate.evaluate(kCmd, target, low, sim_caps(), now).cmd.has_value());

    af::Telemetry high = tel;
    high.altitude_m = 61.0;
    REQUIRE_FALSE(gate.evaluate(kCmd, target, high, sim_caps(), now).cmd.has_value());

    af::Telemetry far = tel;
    far.position_m = Eigen::Vector3d(100, 100, -10);
    const auto r = gate.evaluate(kCmd, target, far, sim_caps(), now);
    REQUIRE_FALSE(r.cmd.has_value());
    REQUIRE(std::string(r.reason) == "outside geofence");
  }

  SECTION("Missing, predicted or stale targets refuse")
  {
    REQUIRE_FALSE(gate.evaluate(kCmd, std::nullopt, tel, sim_caps(), now).cmd.has_value());

    af::Pose3D predicted = target;
    predicted.validity = af::Validity::Predicted;
    REQUIRE_FALSE(gate.evaluate(kCmd, predicted, tel, sim_caps(), now).cmd.has_value());

    const auto stale = gate.evaluate(kCmd, target, tel, sim_caps(), 100.6); // 0.6 s old
    REQUIRE_FALSE(stale.cmd.has_value());
    REQUIRE(std::string(stale.reason) == "target is stale");

    // A target "from the future" means the clocks do not match: refuse
    REQUIRE_FALSE(gate.evaluate(kCmd, target, tel, sim_caps(), 99.0).cmd.has_value());
  }

  SECTION("Targets outside the follow range refuse")
  {
    af::Pose3D close = target;
    close.position_m = Eigen::Vector3d(0, 0, 1.0);
    REQUIRE_FALSE(gate.evaluate(kCmd, close, tel, sim_caps(), now).cmd.has_value());

    af::Pose3D far = target;
    far.position_m = Eigen::Vector3d(0, 0, 25.0);
    REQUIRE_FALSE(gate.evaluate(kCmd, far, tel, sim_caps(), now).cmd.has_value());
  }

  SECTION("Missing or non-finite commands refuse")
  {
    REQUIRE_FALSE(gate.evaluate(std::nullopt, target, tel, sim_caps(), now).cmd.has_value());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(gate.evaluate(Eigen::Vector3d(nan, 0, 0), target, tel, sim_caps(), now).cmd.has_value());
  }
}
