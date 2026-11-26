# Architecture

Describes the code as it is on 2026-11-26. For what works and what does not, see
`IMPLEMENTATION_STATUS.md`.

## One frame, start to finish

```
IFrameSource ──frame──▶ Pipeline ──────────────────────────────▶ SafetyGate ──▶ IDrone
 webcam                  detect ▶ track ▶ estimate ▶ control       allow /        MockDrone
 video file              (IDetector, ITracker,                     limit /        (simulator)
 stream URL               IEstimator, IController)                 refuse
                                   ▲                                               │
                                   └──────────── Telemetry ◀───────────────────────┘
```

`apps/track/aeroforge-track.cpp` owns this loop in a small `Engine` class and puts one of
two front ends on it: the ImGui window, or `--headless` which prints a JSON report.
Everything runs on one thread, one frame at a time.

Startup: parse arguments, `load_config`, build `Pipeline` (throws on a bad module), open the
frame source, connect the simulator. Shutdown: simulator `hold`, disconnect, close source.
Any exception during startup is printed and the process exits with code 1.

## Libraries

| Target | Sources | Role |
|--------|---------|------|
| `af_core` | `src/core`, `src/math`, `src/config`, `src/io`, `src/dji/mock_drone.cpp` | types, time, PID and Kalman maths, frame helpers, config, frame sources, simulator |
| `af_vision` | `src/vision` | colour-blob and ArUco detectors |
| `af_tracking` | `src/tracking` | native template tracker, Kalman smoother, optional PyTracking bridge |
| `af_estimate` | `src/estimate` | size-based and ground-plane range |
| `af_control` | `src/control`, `src/safety` | PID follow controller, `SafetyGate` |
| `af_pipeline` | `src/pipeline` | builds the chain from a config and runs it |
| `af_ui` | `src/ui` | GLFW + ImGui window (optional, `AEROFORGE_WITH_IMGUI`) |

Dependencies point one way: `af_pipeline` → vision / tracking / estimate / control →
`af_core`. `af_ui` is used only by the applications. Concrete module classes are private
to their `.cpp` files; other code gets them through the factories in
`include/aeroforge/modules.hpp`.

## Public headers

| Header | Contents |
|--------|----------|
| `validity.hpp` | `Validity`: Fresh, Predicted, Stale, Invalid, Unavailable |
| `core.hpp` | `Frame`, `Detection`, `SpscRingBuffer`, `FramePool`, `now_seconds` |
| `plugin.hpp` | `IDetector`, `ITargetTracker`, `ITracker`, `IEstimator`, `IController`, `Pose3D` |
| `modules.hpp` | factories for every built-in module, `CameraIntrinsics` |
| `io.hpp` | `IFrameSource`, `make_frame_source` |
| `pipeline.hpp` | `Pipeline`, `PipelineOutput`, `StageTimings` |
| `safety.hpp` | `SafetyGate` |
| `dji_iface.hpp` | `IDrone`, `DroneCapabilities`, `make_mock_drone` |
| `telemetry.hpp` | `Telemetry` (carries `validity` and `simulated`) |
| `config.hpp` | config structs, `load_config`, `parse_config`, numeric accessors |
| `math.hpp` | `PIDController`, Kalman filters, `camera_to_body`, `body_to_ned` |

## Time

- `now_seconds()` is `std::chrono::steady_clock`. Only differences mean anything.
- Live sources stamp frames with `now_seconds()` when the read returns.
- Recorded sources stamp frames with media time, `seq / fps`, so a replay produces the same
  timestamps, the same `dt` and therefore the same filter and controller behaviour on
  every run and at any processing speed.
- `IFrameSource::now_s()` is "now" on the same clock as that source's frames. Frame age and
  target staleness are always computed against it.

## Coordinate frames

| Frame | Axes | Used by |
|-------|------|---------|
| image | u right, v down, pixels | detections |
| camera | X right, Y down, Z forward, metres | `Pose3D`, estimators |
| body (FRD) | X forward, Y right, Z down | controller output, `SafetyGate` |
| NED | X north, Y east, Z down, origin at home | `Telemetry`, `IDrone::send_velocity_cmd` |

`camera_to_body` assumes a level camera looking straight ahead. `body_to_ned` rotates by
yaw only. Gimbal pitch, roll and vehicle tilt are not modelled yet.

## Modules

**Detectors.** `interactive` (default): the native tracker, follows up to five
user-selected regions with a per-target Kalman prediction, multi-scale template matching
and ORB feature matching fused by confidence; during an occlusion it emits `Predicted`
boxes and drops the target after about 45 frames. `colorball`: largest HSV blob.
`aruco`: marker ids and corners through OpenCV's `objdetect` module. `pytracking`:
optional, see below.

**Tracker.** `kalman`: constant-velocity smoothing of the first detection's centroid, two
independent 1-D filters, process noise `q·I` per step (not scaled by `dt`).

**Estimators.** `size_based`: `Z = f·D / d`, needs the real object size.
`ground_plane`: intersects the pixel ray with a flat ground plane using telemetry altitude
and a fixed camera tilt. Both are documented with their assumptions at the top of their
source files and neither models lens distortion.

**Controller.** `pid_velocity`: PID on the body-frame position error, time step from
target timestamps.

**Simulator.** `MockDrone`: velocity follows the command through a 0.3 s first-order lag,
position integrates velocity in 10 ms sub-steps, commands expire after 0.5 s.

## Why there are no worker threads

Measured per-frame cost of the native pipeline is a few milliseconds (see
`IMPLEMENTATION_STATUS.md`), far below the 33 ms frame period. A capture thread and queue
would add a hand-off and one more place for frames to go stale without raising throughput.
`SpscRingBuffer` and `FramePool` exist and are unit-tested but nothing uses them. The case
that would justify a second thread is a slow or blocking stage, which today means the
PyTracking bridge; that is recorded in `TECHNICAL_DEBT.md`.

## Configuration

YAML, validated on load. Errors name the file, the key and the accepted values, for
example `Config 'x.yml': pipeline.input.type 'dji' is not supported (use webcam, file or
stream)`. Module parameters are checked when the pipeline is built; a required parameter
that is missing is an error, not a silent default. Integer and floating-point YAML numbers
are interchangeable. See `configs/track_interactive.yml` for a commented example.

## PyTracking bridge protocol (optional detector)

`src/tracking/pytracking_iface.cpp` starts `python/pytracking_wrapper.py` and exchanges
one line of JSON per request over the child's stdin and stdout. It is **not** Norfair,
MessagePack or a socket; none of those exist in this repository.

- Transport: child process pipes. Requests: one JSON object per line on stdin. Replies:
  one line on stdout starting with the literal prefix `AFJSON ` followed by a JSON
  object. Stdout lines without the prefix are logged and skipped. Stderr is logged.
- Strictly request/reply, one outstanding request at a time. There is no request id and
  **no protocol version field**.

| Request | Fields | Reply |
|---------|--------|-------|
| `{"type":"init"}` | `frame` (base64 JPEG), `bbox` `[x,y,w,h]`, `tracker_name` | `{"status":"ok","tracker":...}` or `{"status":"error","message":...}` |
| `{"type":"track"}` | `frame` (base64 JPEG) | `{"status":"ok","bbox":[x,y,w,h],"confidence":c}` or error |
| `{"type":"reset"}` | none | `{"status":"ok"}` |
| `{"type":"quit"}` | none | `{"status":"ok"}`, then the process exits |

- Timeouts (Windows): 60 s for `init` (model loading), 1 s for `track` and `reset`. After
  a timeout, a closed pipe or a malformed reply the bridge stops tracking, kills the child
  and starts a new one on the next target selection. A failed bridge never produces
  detections.
- Platform: implemented and previously run on Windows. The POSIX branch opens a write-only
  pipe and cannot receive replies; it fails on first use. See `TECHNICAL_DEBT.md`.

## Tests

`tests/unit` exercises one component each with synthetic ground truth. `tests/integration`
runs detectors, the tracker, video replay and the whole pipeline on generated images and a
generated Motion-JPEG clip (`tests/support/synthetic.hpp`). The `smoke.*` CTest entries run
the real `aeroforge-track` executable headless on that clip and on failure cases. No test
needs a camera, a window, a GPU, Python or a drone.

## Decisions

Recorded in `docs/adr/`.
