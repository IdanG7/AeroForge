# AeroForge

A C++20 framework for real-time visual tracking, range estimation and follow-control
research, with a built-in simulator. Version 0.1.0.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**What it is today:** a desktop application that tracks a target you select in a webcam
feed or a recorded video, estimates where it is relative to the camera, and computes
follow commands for a *simulated* vehicle.

**What it is not:** it does not fly anything. There is no connection to a real aircraft
in this repository, and the command path refuses any vehicle that is not a simulator.
See [docs/SAFETY.md](docs/SAFETY.md).

The honest state of every subsystem, with the evidence, is in
[docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md).

## Features

- Click-and-drag target selection, tracked by a CPU tracker (Kalman prediction, template
  matching and ORB features). No GPU and no Python required.
- Colour-blob and ArUco marker detectors.
- Webcam input and video file replay with deterministic timestamps.
- Range from apparent size or from a ground-plane intersection.
- PID follow controller, exercised closed-loop against a kinematic simulator.
- A single safety gate: hold-to-enable or toggle, latched e-stop, speed limit, geofence,
  stale-target rejection, simulation-only actuation.
- ImGui window with video, tracking state, per-stage timings, telemetry and safety state.
- Headless mode with a JSON report, used by the automated tests.
- YAML configuration with validation and readable error messages.
- Optional deep-learning tracker through a Python subprocess (PyTracking, Windows only).

## Quick start

Full instructions, platform status and troubleshooting:
[BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md). The first build compiles all dependencies
and takes 30 to 60 minutes.

```powershell
git clone https://github.com/IdanG7/AeroForge.git
cd AeroForge
git clone https://github.com/microsoft/vcpkg.git vcpkg
git -C vcpkg checkout adf54cf279585302d29f6789b85f56a9c4b7c75b
.\vcpkg\bootstrap-vcpkg.bat -disableMetrics

cmake --preset win-rel
cmake --build --preset win-rel --parallel
ctest --preset win-tests
```

On macOS and Linux use `./vcpkg/bootstrap-vcpkg.sh` and the `mac-rel` / `linux-rel`
presets. Windows with Visual Studio 2022 is the only configuration verified for this
revision.

Run it:

```powershell
# Webcam
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_interactive.yml

# Recorded video
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_interactive.yml --input clip.mp4

# No window, JSON report on stdout
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --headless --config configs\track_interactive.yml --input clip.avi --roi 100,120,64,64
```

Controls in the window:

| Input | Action |
|-------|--------|
| Drag on the image | Select a target (up to five) |
| R | Drop all targets |
| F (hold) | Enable simulated control. Set `safety.require_hold_to_enable: false` to make F a toggle |
| SPACE | E-stop (latched) |
| ESC | Exit |

## Configuration

`configs/track_interactive.yml` is the commented reference. In short:

```yaml
pipeline:
  input:
    type: webcam            # webcam | file | stream
    device: 0
    size: { width: 1280, height: 720, fps: 30 }
  modules:
    - { name: detector, type: interactive }          # or colorball, aruco, pytracking
    - { name: tracker, type: kalman, params: { q: 0.02, r: 0.20 } }
    - name: estimator
      type: size_based                               # or ground_plane
      params: { object_diameter_m: 0.20, fx: 920, fy: 920, cx: 640, cy: 360 }
    - name: controller
      type: pid_velocity
      params:
        desired_offset_m: [-3, 0, 0]                 # 3 m behind the target
        kp: [0.8, 0.8, 0.6]
        kd: [0.1, 0.1, 0.08]
        max_speed_mps: 3.0
safety:
  require_hold_to_enable: true
  stale_timeout_s: 0.5
  limits: { max_speed_mps: 6.0, min_follow_dist_m: 2.0, max_follow_dist_m: 10.0 }
  geofence: { min_alt_m: 3.0, max_alt_m: 60.0, max_radius_m: 120.0 }
```

`object_diameter_m` must be the real size of the object you track. Range from size is
only as good as that number.

## Documentation

| Document | Contents |
|----------|----------|
| [docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md) | What is verified, unverified, broken or missing, with test evidence |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Data flow, libraries, frames, timestamps, IPC protocol |
| [docs/SAFETY.md](docs/SAFETY.md) | What the safety gate does and what is not covered |
| [docs/TECHNICAL_DEBT.md](docs/TECHNICAL_DEBT.md) | Prioritised backlog |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Milestones and their exit checks |
| [docs/HARDWARE_COMPATIBILITY.md](docs/HARDWARE_COMPATIBILITY.md) | DJI Mini 2 / iOS SDK research and the bench test plan (nothing tested on hardware yet) |
| [docs/adr/](docs/adr/) | Design decisions |

## Hardware

No aircraft is supported. The first planned hardware target is read-only video and
telemetry from an original DJI Mini 2 through an iPhone; that work is at the research
stage. If you fly anything near this software, a human pilot flies it, within your local
rules, and AeroForge only watches.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) and [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## License

MIT, see [LICENSE](LICENSE).

Built on [OpenCV](https://opencv.org/), [Eigen](https://eigen.tuxfamily.org/),
[Dear ImGui](https://github.com/ocornut/imgui), [GLFW](https://www.glfw.org/),
[yaml-cpp](https://github.com/jbeder/yaml-cpp), [spdlog](https://github.com/gabime/spdlog),
[fmt](https://github.com/fmtlib/fmt), [nlohmann/json](https://github.com/nlohmann/json) and
[Catch2](https://github.com/catchorg/Catch2).
