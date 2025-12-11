# AeroForge

**Professional-grade, real-time vision + control framework for drone tracking applications**

[![CI](https://github.com/YOUR_USERNAME/aeroforge/workflows/CI/badge.svg)](https://github.com/YOUR_USERNAME/aeroforge/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

AeroForge is a cross-platform (macOS, Windows, Linux) C++20 framework for building vision-based drone control applications. It provides a modular, extensible pipeline for real-time object detection, tracking, pose estimation, and control with strict safety mechanisms.

**Key Features:**
- 🎯 **Interactive Object Selection**: Click and drag to track ANY object in real-time
- 🚀 High-performance template matching tracker (no OpenCV contrib needed)
- 🤖 **Norfair multi-object tracking** via Python service integration
- 📊 Complete pipeline: Detection → Tracking → 3D estimation → PID control
- 🎨 **Professional redesigned UI** with organized panels and real-time visualization
- 🛡️ Safety-first: hold-to-enable, e-stop, geofence, speed limits
- 📹 ImGui HUD with FPS, telemetry, 3D position, and velocity command displays
- ⚙️ YAML-based configuration (no recompile needed)
- 🧪 Unit & integration tests (Catch2)
- 🔌 DJI SDK integration (optional, gated by build flag)

---

## Quick Start

### Prerequisites

- **C++20 compiler**: MSVC 2022, Clang 14+, or GCC 11+
- **CMake** 3.21+
- **vcpkg** (managed by the project)
- **Webcam** for testing

### Build (Windows)

```powershell
# Clone the repository
git clone https://github.com/YOUR_USERNAME/aeroforge.git
cd aeroforge

# Bootstrap vcpkg
.\vcpkg\bootstrap-vcpkg.bat

# Configure and build
cmake --preset win-rel
cmake --build --preset win-rel -j

# Run demo viewer
.\build\win-rel\bin\demo_viewer.exe
```

### Build (macOS)

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/aeroforge.git
cd aeroforge

# Bootstrap vcpkg
./vcpkg/bootstrap-vcpkg.sh

# Configure and build
cmake --preset mac-rel
cmake --build --preset mac-rel -j

# Run demo viewer
./build/mac-rel/bin/demo_viewer
```

### Run AeroForge-Track

```bash
# Windows
.\build\win-rel\bin\aeroforge-track.exe --config configs\track_colorball.yml

# macOS/Linux
./build/mac-rel/bin/aeroforge-track --config configs/track_colorball.yml
```

**Controls:**
- **Click & Drag**: Select object to track on camera feed
- **R**: Reset tracking and select new target
- **F**: Toggle safety enable/disable (enables velocity commands)
- **SPACE**: Emergency stop (hold mode)
- **ESC**: Exit application

---

## Norfair Multi-Object Tracking

AeroForge supports advanced multi-object tracking using the [Norfair](https://github.com/tryolabs/norfair) Python library via a high-performance IPC bridge.

### Setup

1. **Start the Norfair service** (one-time setup):
```bash
cd services/norfair
./run.sh
```

The service will create a Python virtual environment, install dependencies, and start listening on `/tmp/aeroforge_norfair.sock`.

2. **Run AeroForge with Norfair tracking**:
```bash
# macOS/Linux
./build/mac-rel/bin/aeroforge-track --config configs/track_norfair.yml
```

### Features

- **Multi-object tracking**: Track multiple objects simultaneously with persistent IDs
- **Robust tracking**: Survives brief occlusions and cluttered scenes
- **Configurable**: Tune distance functions, thresholds, and tracking parameters
- **Graceful fallback**: Automatically falls back to pass-through mode if service unavailable
- **Auto-reconnection**: Reconnects to service with exponential backoff

### Configuration

The Norfair tracker can be configured in your YAML config:

```yaml
- name: tracker
  type: norfair
  params:
    socket_path: "/tmp/aeroforge_norfair.sock"
    timeout_ms: 500
    auto_reconnect: true
```

See `services/norfair/README.md` for detailed configuration options and tuning guide.

---

## Configuration

AeroForge uses YAML for runtime configuration. Example (`configs/track_colorball.yml`):

```yaml
pipeline:
  input:
    type: webcam
    device: 0
    size: { width: 1280, height: 720, fps: 30 }

  modules:
    - name: detector
      type: colorball
      params:
        hsv_lo: [30, 120, 80]   # Green ball
        hsv_hi: [90, 255, 255]
        min_area_px: 200

    - name: tracker
      type: kalman
      params: { q: 0.02, r: 0.20 }

    - name: estimator
      type: size_based
      params:
        object_diameter_m: 0.20
        fx: 920.0; fy: 920.0; cx: 640.0; cy: 360.0

    - name: controller
      type: pid_velocity
      params:
        desired_offset_m: [-3.0, 0.0, 0.0]  # 3m behind target
        kp: [0.8, 0.8, 0.6]
        kd: [0.1, 0.1, 0.08]
        max_speed_mps: 3.0

safety:
  require_hold_to_enable: true
  e_stop_key: "SPACE"
  limits:
    max_speed_mps: 6.0
    min_follow_dist_m: 2.0
    max_follow_dist_m: 10.0
  geofence:
    min_alt_m: 3.0
    max_alt_m: 60.0
    max_radius_m: 120.0
```

---

## Architecture

```
AeroForge Core (C++)                    Python Services
  ├─ Capture (webcam/RTSP/DJI)             ├─ Norfair Tracker
  │   └─ SPSC FrameRingBuffer              │   (Unix Socket + MessagePack)
  ├─ Pipeline (config-driven)              │
  │   ├─ Detector (Interactive / ColorBall / ArUco)
  │   ├─ Tracker (Kalman / Norfair*)       ◄──┘
  │   ├─ Estimator (size-based / ground-plane)
  │   ├─ Controller (PID velocity)
  │   └─ Sinks (HUD, recorder, telemetry)
  ├─ Flight Interface (IDrone → DJI | Mock)
  ├─ Safety Manager (hold-to-enable, e-stop, geofence)
  └─ UI/HUD (ImGui)

*Norfair requires Python service running
```

---

## Safety Disclaimer

⚠️ **IMPORTANT**: This software is provided AS-IS for research and development purposes. When using with real drones:

1. **Always maintain visual line of sight** with a human pilot holding the RC transmitter.
2. **Test in open areas** away from people, buildings, and obstacles.
3. **Start with low speeds** and gradually increase gains.
4. **Use the mock drone** for initial testing before connecting to real hardware.
5. **Comply with local UAV regulations** (FAA Part 107, etc.).

The authors are NOT liable for any damage, injury, or legal consequences. **Fly responsibly.**

---

## Testing

```bash
# Run all tests
ctest --preset mac-tests --output-on-failure  # macOS
ctest --preset win-tests --output-on-failure  # Windows

# Run specific test
./build/mac-rel/tests/unit/test_ring_buffer
```

---

## Roadmap

- [x] **v0.1.0**: Core framework, webcam, ColorBall/ArUco, PID, mock drone
- [ ] **v0.2.0**: TensorRT YOLO detector, Jetson Nano support, CUDA acceleration
- [ ] **v0.3.0**: Obstacle avoidance, ROS2 bridge, WebRTC streaming
- [ ] **v0.4.0**: Plugin marketplace, Python bindings (pybind11)

See [docs/ROADMAP.md](docs/ROADMAP.md) for detailed planning.

---

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repo
2. Create a feature branch (`git checkout -b feat/amazing-feature`)
3. Commit changes (`git commit -m 'feat: add amazing feature'`)
4. Push to branch (`git push origin feat/amazing-feature`)
5. Open a Pull Request

**Code of Conduct**: [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)

---

## License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) for details.

---

## Acknowledgments

- [OpenCV](https://opencv.org/) for vision primitives
- [Eigen](https://eigen.tuxfamily.org/) for linear algebra
- [ImGui](https://github.com/ocornut/imgui) for HUD rendering
- [Norfair](https://github.com/tryolabs/norfair) for multi-object tracking
- [MessagePack](https://msgpack.org/) for efficient IPC serialization
- [Catch2](https://github.com/catchorg/Catch2) for testing
- [spdlog](https://github.com/gabime/spdlog) for logging
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) for config parsing

---

**Made with ❤️ for the drone developer community**
