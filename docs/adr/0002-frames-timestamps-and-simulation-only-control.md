# ADR 0002: Explicit frames, source-clock timestamps, simulation-only control

Date: 2026-11-26. Status: accepted.

## Problem

Three defects made the control half of the pipeline meaningless:

1. The controller subtracted a body-frame offset ("3 m behind", X forward) from a
   camera-frame position (X right). The follow distance was applied sideways.
2. The controller used a fixed `dt = 0.033 s` and the PID divided by `dt` unchecked, so
   behaviour depended on frame rate and a zero or negative step produced garbage.
3. `safety.require_hold_to_enable`, the speed limit, the follow-distance limits and the
   geofence were parsed and never used. The F key toggled on key level, so holding it
   flipped the state every frame. Commands went straight from the controller to `IDrone`.

## Alternatives

- Patch each site in the application's `main`.
- Introduce named frames and a single command gate in the library, with tests.

## Decision

- Frames are named and documented: camera (X right, Y down, Z forward), body FRD, NED.
  `camera_to_body` and `body_to_ned` are the only conversions. The controller works in the
  body frame; the application converts to NED for `IDrone`.
- Every `Frame` is stamped by its source: monotonic clock for live sources, media time for
  recordings. `dt` everywhere comes from those stamps. `IFrameSource::now_s()` gives "now"
  on the same clock.
- Detections, poses and telemetry carry `Validity`. Only `Fresh` drives control.
- `SafetyGate` is the single place a command is allowed, limited or refused, and it
  refuses any backend whose capabilities do not say `simulated`.
- `MockDrone` became a small kinematic simulator so the controller can be tested closed
  loop.

## Why

The fixes are only trustworthy if they can be tested without a window or a camera, which
means they have to live in the library. Media-time stamps make replays deterministic.

## Trade-offs

- `IController` gained `reset()`, `IDrone` gained `capabilities()` and `step()`. Any
  out-of-tree implementation has to add them.
- The ground-plane estimator now returns the camera-frame point (Y = height below the
  camera) and honours the configured tilt. It used to return Y = 0 and ignore tilt.
- `size_based` no longer has a default object size. A config without
  `object_diameter_m` is rejected.
- Hold-to-enable is now really the default, as the configs always said.

## Limits

`camera_to_body` assumes a level, forward-looking camera and `body_to_ned` uses yaw only.
A gimbal model is needed before any of this is compared with a real aircraft.
