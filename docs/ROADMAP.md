# Roadmap

Updated 2026-11-26. A milestone is done when its exit checks have been run and recorded in
`IMPLEMENTATION_STATUS.md`, not when the code is written. Backlog item numbers refer to
`TECHNICAL_DEBT.md`.

The released version is still **0.1.0**. The version number in `CMakeLists.txt` has not
been changed, because the release workflow publishes automatically when it changes
(item 1.13).

## v0.1.1 - Implementation recovery

Goal: a clean checkout builds, the demo tracks something, the documentation is true.

| Work | State |
|------|-------|
| Working tree compiles; tests run | Done on Windows / MSVC 2019 |
| Native tracker is the default; PyTracking optional | Done |
| Video file replay and headless mode | Done |
| Modules reachable through headers and factories; duplicates removed | Done |
| Frame conventions, timestamps, controller and estimator fixes | Done |
| Safety gate, simulation-only actuation | Done |
| Audit documents, architecture, safety, hardware research | Done |
| Webcam and GUI checked by a person on this machine | **Open** (owner: see the checklist in `IMPLEMENTATION_STATUS.md`) |
| CI green on Windows | **Open** (workflow rewritten, needs a push) |
| Reproducible dependencies (0.5, 0.7) | **Open** |
| Decide release automation (1.13), then bump to 0.1.1 | **Open**, owner decision |

Exit checks: `ctest` green locally and in CI on Windows; a person has run the GUI with a
webcam and with a video file; README commands work from a fresh clone.

## v0.2.0 - Reliable perception framework

| Work | Backlog |
|------|---------|
| macOS and Linux builds verified in CI, sanitizer job green | 0.6, 2.8 |
| Trim OpenCV features, pin vcpkg baseline | 0.7, 2.1 |
| Camera calibration tool and calibration file; undistortion | 1.12 |
| Recording and replay of frames + detections + telemetry | 2.11 |
| PyTracking: worker thread, versioned protocol, POSIX support, or removal | 1.7, 1.8, 1.9 |
| Tracker evaluation on recorded, non-synthetic clips with ground-truth boxes | new |
| GUI: scaled video, tracking-state and log panels | 2.5, 3.4 |
| Format the tree, turn the format check on | 2.10 |

Exit checks: tracking accuracy and loss rate reported on a fixed set of recorded clips;
all three desktop platforms green in CI.

## v0.3.0 - Simulation and mathematical verification

| Work | Backlog |
|------|---------|
| Gimbal and mount model; full attitude in the frame conversions | 1.11 |
| Kalman process noise scaled by `dt`; documented tuning | 2.3 |
| Synthetic camera in the simulator (project a simulated target into the image) so the whole loop, perception included, runs closed-loop | new |
| Fault-injection scenarios: frame drops, latency, target loss, telemetry loss | new |
| Decide whether the kinematic simulator is enough or PX4 SITL is needed | 3.6 |

Exit checks: closed-loop scenarios with stated error bounds run in CI; every estimator
has its error model documented and tested.

## v0.4.0 - DJI Mini 2 observation (read-only)

Gated on the feasibility test in `HARDWARE_COMPATIBILITY.md`. Nothing here starts before
steps 1 to 9 of that test pass on the real phone and aircraft.

| Work | Notes |
|------|-------|
| `apps/ios-bridge/`: SDK registration, connection state, video feed, telemetry | Swift, MSDK V4.16.2, device builds only |
| Versioned telemetry message (source id, sequence, timestamp, validity, measurements, link health) | schema in `protocols/` |
| Video transport to the desktop | decide after measuring: forward the SDK's H.264 as-is (RTP or framed TCP) and decode on the desktop |
| Desktop: network frame source and telemetry source behind the existing interfaces | `IFrameSource`, `Telemetry` |
| Mock bridge that replays a recording, for tests without hardware | |
| Latency measurement end to end | |

The bridge never sends anything to the aircraft, and the desktop keeps refusing commands
to non-simulated backends.

## v0.5.0 - Performance and portability

| Work | Backlog |
|------|---------|
| `benchmarks/` with a repeatable run and `PERFORMANCE.md` | 3.3 |
| Profile before optimising; hardware decode / acceleration only where measurements justify it | |
| Long-run stability (hours) with memory tracking | |

## v1.0.0 - Stable framework

Stable public headers, verified builds on the supported platforms, hardware checklist
passed and recorded, releases produced by a deliberate, documented step.

## Not planned

- Automated control of a physical aircraft.
- Undocumented DJI protocols or firmware changes.
- GPU inference before the CPU pipeline has benchmarks that show the need.
