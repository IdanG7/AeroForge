#include "aeroforge/safety.hpp"

#include <cmath>

namespace af
{

SafetyGate::SafetyGate(const SafetyConfig& config) : config_(config) {}

void SafetyGate::update(bool enable_key_down, bool estop_key_down)
{
  const bool enable_pressed = enable_key_down && !prev_enable_down_;
  prev_enable_down_ = enable_key_down;

  if (estop_key_down)
  {
    state_ = State::EStopped;
    return;
  }

  if (config_.require_hold_to_enable)
  {
    if (state_ == State::EStopped)
    {
      // Stay latched until the operator lets go of the enable key
      if (!enable_key_down)
      {
        state_ = State::Disabled;
      }
      return;
    }
    state_ = enable_key_down ? State::Enabled : State::Disabled;
    return;
  }

  if (enable_pressed)
  {
    // From E-stop the first press only clears the latch
    state_ = (state_ == State::Disabled) ? State::Enabled : State::Disabled;
  }
}

bool SafetyGate::geofence_ok(const Telemetry& telemetry) const
{
  if (telemetry.validity != Validity::Fresh || !std::isfinite(telemetry.altitude_m) ||
      !telemetry.position_m.allFinite())
  {
    return false;
  }
  const double radius = std::hypot(telemetry.position_m.x(), telemetry.position_m.y());
  return telemetry.altitude_m >= config_.min_alt_m && telemetry.altitude_m <= config_.max_alt_m &&
         radius <= config_.max_radius_m;
}

SafetyGate::Result SafetyGate::evaluate(const std::optional<Eigen::Vector3d>& cmd_body,
                                        const std::optional<Pose3D>& target, const Telemetry& telemetry,
                                        const DroneCapabilities& caps, double now_s) const
{
  auto refuse = [](const char* reason)
  {
    Result r;
    r.reason = reason;
    return r;
  };

  if (state_ == State::EStopped)
  {
    return refuse("e-stop latched");
  }
  if (state_ != State::Enabled)
  {
    return refuse("operator has not enabled control");
  }
  if (!caps.simulated)
  {
    return refuse("actuation is simulation-only");
  }
  if (!caps.velocity_control)
  {
    return refuse("backend does not accept velocity commands");
  }
  if (telemetry.validity != Validity::Fresh)
  {
    return refuse("telemetry is not fresh");
  }
  if (!geofence_ok(telemetry))
  {
    return refuse("outside geofence");
  }
  if (!target.has_value() || target->validity != Validity::Fresh || !target->position_m.allFinite())
  {
    return refuse("no fresh target");
  }
  const double age_s = now_s - target->timestamp_s;
  if (!std::isfinite(age_s) || age_s < 0.0 || age_s > config_.stale_timeout_s)
  {
    return refuse("target is stale");
  }
  const double range = target->position_m.norm();
  if (range < config_.min_follow_dist_m || range > config_.max_follow_dist_m)
  {
    return refuse("target outside follow range");
  }
  if (!cmd_body.has_value() || !cmd_body->allFinite())
  {
    return refuse("no valid command");
  }

  Result r;
  Eigen::Vector3d cmd = *cmd_body;
  const double mag = cmd.norm();
  if (mag > config_.max_speed_mps)
  {
    cmd *= config_.max_speed_mps / mag;
  }
  r.cmd = cmd;
  return r;
}

} // namespace af
