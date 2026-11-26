#pragma once

#include "config.hpp"
#include "dji_iface.hpp"
#include "plugin.hpp"
#include <optional>
#include <string>

namespace af
{

// Single place where a velocity command is allowed, limited or refused.
//
// Operator intent (update):
//   hold mode   (safety.require_hold_to_enable: true)  - enabled only while the enable key
//               is held down.
//   toggle mode (safety.require_hold_to_enable: false) - each press of the enable key
//               flips enabled/disabled.
//   E-stop latches in both modes. In hold mode the latch clears once both keys have been
//   released; in toggle mode the next enable press clears the latch (to Disabled) and a
//   second press enables.
//
// Command checks (evaluate), first failure wins:
//   e-stop, operator not enabling, backend is not a simulator, backend cannot take
//   velocity commands, telemetry not fresh, outside geofence, no fresh target, target
//   stale, target outside the follow range, command missing or non-finite.
//   A passing command is limited to safety.limits.max_speed_mps.
//
// Physical actuation: evaluate() refuses every backend whose capabilities do not say
// `simulated`. There is no configuration switch for this.
class SafetyGate
{
public:
  enum class State
  {
    Disabled,
    Enabled,
    EStopped,
  };

  struct Result
  {
    std::optional<Eigen::Vector3d> cmd; // body frame, limited; nullopt when refused
    const char* reason{""};             // why the command was refused; "" when allowed
  };

  explicit SafetyGate(const SafetyConfig& config);

  void update(bool enable_key_down, bool estop_key_down);

  State state() const { return state_; }
  bool hold_to_enable() const { return config_.require_hold_to_enable; }

  // `now_s` must be on the same clock as `target->timestamp_s` (IFrameSource::now_s()).
  Result evaluate(const std::optional<Eigen::Vector3d>& cmd_body,
                  const std::optional<Pose3D>& target, const Telemetry& telemetry,
                  const DroneCapabilities& caps, double now_s) const;

  // Geofence check on its own, for display.
  bool geofence_ok(const Telemetry& telemetry) const;

private:
  SafetyConfig config_;
  State state_{State::Disabled};
  bool prev_enable_down_{false};
};

constexpr const char* to_string(SafetyGate::State s)
{
  switch (s)
  {
  case SafetyGate::State::Disabled:
    return "DISABLED";
  case SafetyGate::State::Enabled:
    return "ENABLED";
  case SafetyGate::State::EStopped:
    return "E-STOP";
  }
  return "?";
}

} // namespace af
