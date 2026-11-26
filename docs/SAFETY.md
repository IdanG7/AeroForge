# Safety

AeroForge today is a perception and simulation tool. **It cannot move an aircraft**: the
only vehicle backend in the repository is a simulator, and the command gate refuses any
backend that does not declare itself simulated.

This page describes what the code does, how that is tested, and what is not covered.

## Where commands can go

```
Pipeline (controller)  ->  SafetyGate::evaluate  ->  IDrone::send_velocity_cmd
   unlimited command        allow / limit / refuse      MockDrone only
```

- `Pipeline` produces an unlimited body-frame velocity command. It never talks to a vehicle.
- `SafetyGate` (`include/aeroforge/safety.hpp`) is the only path to `send_velocity_cmd`
  in the application.
- `IDrone::capabilities()` states what a backend is. `SafetyGate` refuses when
  `simulated` is false. There is no config key or build flag that changes this.
- `AEROFORGE_WITH_DJI=ON` stops the CMake configure with an explanation. There is no
  desktop DJI backend.

## Operator controls

| Key | Hold mode (`require_hold_to_enable: true`, default) | Toggle mode (`false`) |
|-----|------------------------------------------------------|------------------------|
| F | Control enabled only while held | Each press flips enabled / disabled |
| SPACE | E-stop, latched until F and SPACE are both released | E-stop, latched; next F press clears to disabled, the one after enables |
| R | Drop the target; controller history cleared | same |
| ESC | Exit; simulator told to hold | same |

Before this work the application ignored `require_hold_to_enable` and toggled on F, and it
read the key level instead of the key press, so holding F flipped the state every frame.

## Command checks

`SafetyGate::evaluate` returns either a limited command or a reason. First failure wins.

| Check | Reason shown in the UI |
|-------|------------------------|
| E-stop latched | `e-stop latched` |
| Operator not enabling | `operator has not enabled control` |
| Backend is not a simulator | `actuation is simulation-only` |
| Backend has no velocity control | `backend does not accept velocity commands` |
| Telemetry not `Fresh` | `telemetry is not fresh` |
| Altitude outside `[min_alt_m, max_alt_m]` or radius over `max_radius_m` | `outside geofence` |
| No target, target not `Fresh`, non-finite position | `no fresh target` |
| Target older than `stale_timeout_s`, or newer than "now" | `target is stale` |
| Range outside `[min_follow_dist_m, max_follow_dist_m]` | `target outside follow range` |
| Command missing or non-finite | `no valid command` |

A passing command is scaled down to `max_speed_mps`. When control is withdrawn the
simulator is told to hold immediately, and its own watchdog brakes to a hover if no
command arrives for 0.5 s.

## Observation validity

Every detection, pose and telemetry sample carries `af::Validity`:
`Fresh`, `Predicted`, `Stale`, `Invalid`, `Unavailable`. Only `Fresh` drives control.

- The tracker marks boxes it extrapolated through an occlusion as `Predicted`.
- Estimators copy the detection's validity into the pose and return nothing for
  degenerate input (zero-size box, ray above the horizon, non-finite values, telemetry
  that is not fresh).
- The controller returns nothing for a pose that is not `Fresh`, and resets itself when
  timestamps repeat, go backwards or jump by more than a second.
- A default-constructed `Telemetry` is `Unavailable`.

## Test evidence

Run `ctest` (see `docs/IMPLEMENTATION_STATUS.md` for the last recorded run).

| Behaviour | Test |
|-----------|------|
| Hold and toggle semantics, e-stop latch | `tests/unit/test_safety_gate.cpp` |
| Every refusal reason above, speed limit | `tests/unit/test_safety_gate.cpp` |
| Non-simulated backend refused | `tests/unit/test_safety_gate.cpp` |
| Simulator watchdog, refusal of non-finite commands, no telemetry when disconnected | `tests/unit/test_control_sim.cpp` |
| Controller ignores predicted / stale / invalid poses, survives bad timestamps | `tests/unit/test_control_sim.cpp` |
| PID with zero, negative and non-finite inputs | `tests/unit/test_pid.cpp` |
| Estimators reject degenerate geometry and stale telemetry | `tests/unit/test_estimators.cpp` |
| Lost target becomes predicted, then disappears; no invented measurements | `tests/integration/test_tracking_pipeline.cpp` |
| Missing config, missing video, bad arguments exit with an error | `smoke.*` CTest entries |

## Not covered, and known gaps

- **No physical vehicle has ever been connected.** Nothing here is evidence about
  behaviour with an aircraft.
- The GUI key handling itself (GLFW events to `SafetyGate::update`) is not under automated
  test; only the state machine is.
- The geofence uses simulator telemetry and a flat radius/altitude box around the home
  point. It is not a substitute for the aircraft's own limits.
- `camera_to_body` assumes a level, forward-looking camera. Gimbal pitch is ignored, so
  the simulated follow behaviour is only meaningful for a level camera.
- The size-based range assumes `object_diameter_m` is right. The interactive tracker lets
  you select any object; if its real size differs, range and commands are wrong by the
  same factor. The follow-range check then works on the wrong number.
- Staleness is judged on the frame source's clock. A source that delivers old frames with
  new timestamps (network buffering) would not be caught.
- The optional PyTracking detector blocks the loop for up to 1 s per frame when the Python
  process hangs (60 s on first selection while models load) before it is declared failed.
- A latched e-stop and the enabled state are not shown on the video itself, only in the
  Safety panel.

## Rules for future hardware work

1. Physical integration starts read-only: video, telemetry, connection state.
2. A physical backend reports `simulated = false` and `velocity_control = false`.
3. Removing the simulation-only check requires its own design review and is not part of
   any planned milestone.
4. Hardware tests follow the staged plan in `docs/HARDWARE_COMPATIBILITY.md`, propellers
   off for bench tests, a human pilot in control for any flight.
