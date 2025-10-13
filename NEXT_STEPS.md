# AeroForge - Next Steps Guide

**Last Updated**: 2025-10-13
**Current Status**: Phase 0 Complete, Ready for Build Verification

---

## Immediate Actions (This Session or Next)

### 1. Build Verification (CRITICAL - Do First!)

This will verify that all the code compiles and links correctly.

#### Windows
```powershell
# From D:\Projects\AeroForge

# Step 1: Bootstrap vcpkg (one-time, ~5-10 min)
.\vcpkg\bootstrap-vcpkg.bat

# Step 2: Configure (first run downloads dependencies, ~10-20 min)
cmake --preset win-rel

# Step 3: Build (~5 min)
cmake --build --preset win-rel -j

# Step 4: Run tests
ctest --preset win-tests --output-on-failure

# Step 5: Run demo viewer
.\build\win-rel\bin\RelWithDebInfo\demo_viewer.exe
```

**Expected Outcome**: Webcam window opens with live feed + green FPS counter in top-left.

#### macOS (if applicable)
```bash
./vcpkg/bootstrap-vcpkg.sh
cmake --preset mac-rel
cmake --build --preset mac-rel -j
ctest --preset mac-tests --output-on-failure
./build/mac-rel/bin/demo_viewer
```

#### Common Build Issues

**Problem**: vcpkg fails with "Could not detect vcpkg-root"
**Fix**: Ensure you're in the `D:\Projects\AeroForge` directory and vcpkg submodule is initialized:
```bash
git submodule update --init --recursive
```

**Problem**: CMake can't find OpenCV
**Fix**: vcpkg toolchain not specified. Re-run cmake with:
```powershell
cmake --preset win-rel  # or specify manually with -DCMAKE_TOOLCHAIN_FILE
```

**Problem**: Linker errors about "unresolved external symbol"
**Fix**: Clean and rebuild:
```powershell
cmake --build build --target clean
cmake --build --preset win-rel -j
```

**Problem**: demo_viewer crashes immediately
**Fix**: Webcam permissions. On macOS, grant camera access in System Preferences. On Windows, check Device Manager.

---

## Phase 1: Pipeline Orchestration (Next Priority)

### Goal
Make `aeroforge-track --config configs/track_colorball.yml` actually run the full pipeline (detect → track → estimate → control).

### Current State
- ✅ All modules are implemented as standalone classes
- ✅ Config loader parses YAML correctly
- ⏸️ **TODO**: Factory pattern to instantiate modules from config strings
- ⏸️ **TODO**: Pipeline execution engine with inter-stage buffers

### Implementation Steps

1. **Create Module Factories** (new file: `src/core/factory.cpp`)
   ```cpp
   namespace af {

   std::unique_ptr<IDetector> make_detector(const std::string& type, const ConfigMap& params) {
     if (type == "colorball") {
       cv::Scalar hsv_lo = get_vec3_as_scalar(params, "hsv_lo");
       cv::Scalar hsv_hi = get_vec3_as_scalar(params, "hsv_hi");
       double min_area = get_param(params, "min_area_px", 200.0);
       return std::make_unique<ColorBallDetector>(hsv_lo, hsv_hi, min_area);
     }
     else if (type == "aruco") {
       int dict_id = get_param(params, "dict_id", 10);
       return std::make_unique<ArUcoDetector>(dict_id);
     }
     return nullptr;
   }

   // Similar factories for ITracker, IEstimator, IController
   }
   ```

2. **Update apps/track/aeroforge-track.cpp**
   - Replace TODO markers with:
     ```cpp
     // Build pipeline from config
     auto detector = make_detector(config.modules[0].type, config.modules[0].params);
     auto tracker = make_tracker(config.modules[1].type, config.modules[1].params);
     auto estimator = make_estimator(config.modules[2].type, config.modules[2].params);
     auto controller = make_controller(config.modules[3].type, config.modules[3].params);

     // Inside main loop:
     auto detections = detector->detect(frame);
     if (!detections.empty()) {
       auto tracked = tracker->update(dt, detections);
       if (!tracked.empty()) {
         auto pose = estimator->estimate(tracked[0], frame, telemetry);
         if (pose && safety_enabled) {
           auto cmd = controller->velocity_cmd(*pose, telemetry);
           if (cmd) {
             drone->send_velocity_cmd(*cmd);
           }
         }
       }
     }
     hud.render_detections(detections);
     ```

3. **Add to CMakeLists.txt**
   ```cmake
   # In af_core target sources:
   src/core/factory.cpp
   ```

4. **Test with colorball config**
   ```powershell
   .\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_colorball.yml
   ```
   - Hold a green ball/object in front of webcam
   - Press 'F' to enable safety
   - Observe detection box + telemetry window showing mock velocity commands

### Acceptance Criteria
- [ ] Config parses without errors
- [ ] Detector finds ball/marker in live feed
- [ ] Tracker smooths centroid (verify in HUD overlay)
- [ ] Estimator computes range (log to console or display in HUD)
- [ ] Controller generates velocity commands (check debug logs)
- [ ] FPS remains > 20 at 720p

---

## Phase 4: Safety Manager (High Priority)

### Goal
Enforce geofence, speed limits, and follow-distance guards before sending commands to drone.

### Implementation

1. **Create include/aeroforge/safety.hpp**
   ```cpp
   namespace af {

   struct SafetyLimits {
     double max_speed_mps;
     double min_follow_dist_m;
     double max_follow_dist_m;
     double min_alt_m;
     double max_alt_m;
     double max_radius_m;
   };

   class SafetyManager {
   public:
     explicit SafetyManager(const SafetyLimits& limits);

     bool is_enabled() const;
     void enable();
     void disable();
     void emergency_stop();

     bool check_geofence(const Telemetry& tel) const;
     Eigen::Vector3d clamp_velocity(const Eigen::Vector3d& cmd) const;
     bool check_follow_distance(const Pose3D& target) const;

   private:
     SafetyLimits limits_;
     bool enabled_{false};
     std::chrono::steady_clock::time_point last_estop_;
   };

   }
   ```

2. **Implement in src/core/safety.cpp**
   - Hold-to-enable logic
   - E-stop cooldown (2 seconds)
   - Geofence checks (altitude, radius from home)
   - Speed clamping
   - Distance checks (min/max range to target)

3. **Integrate into aeroforge-track.cpp**
   ```cpp
   SafetyManager safety(config.safety);

   // Key handling:
   if (hud.is_key_pressed(GLFW_KEY_F)) {
     safety.enable();
   }
   if (hud.is_key_pressed(GLFW_KEY_SPACE)) {
     safety.emergency_stop();
     drone->hold();
   }

   // Before sending command:
   if (safety.is_enabled() && safety.check_geofence(telemetry) &&
       safety.check_follow_distance(*pose)) {
     auto safe_cmd = safety.clamp_velocity(*cmd);
     drone->send_velocity_cmd(safe_cmd);
   }
   ```

### Acceptance Criteria
- [ ] Cannot send commands unless 'F' is held/pressed
- [ ] E-stop on SPACE immediately calls drone->hold()
- [ ] Velocity commands clamped to max_speed_mps
- [ ] Out-of-geofence conditions logged and commands blocked

---

## Phase 5: Documentation & Release Prep

### Tasks

1. **Create docs/ARCHITECTURE.md**
   - Pipeline flow diagram
   - Class hierarchy
   - Data flow (Frame → Detection → Pose3D → Command)
   - Plugin interface ABI

2. **Record Demo Video**
   - 60-90 seconds
   - Show: webcam feed → ball/marker tracking → HUD overlays → safety controls
   - Tools: OBS Studio (free, cross-platform)
   - Export: MP4 at 1080p, 30 FPS
   - Create GIF with ffmpeg:
     ```bash
     ffmpeg -i demo.mp4 -vf "fps=10,scale=800:-1:flags=lanczos" -t 10 demo.gif
     ```

3. **Update README.md**
   - Add hero GIF: `![AeroForge Demo](docs/demo.gif)`
   - Add build status badge (once CI is live)

4. **Tag v0.1.0 Release**
   ```bash
   git tag -a v0.1.0 -m "AeroForge v0.1.0 - Initial release"
   git push origin v0.1.0
   ```

5. **GitHub Release Assets**
   - Source code (auto-generated by GitHub)
   - Optional: Pre-built binaries (Windows .exe, macOS .app)
   - Release notes with:
     - Features list
     - Known limitations
     - Safety disclaimer
     - Build instructions

---

## Optional Enhancements (Post v0.1.0)

### Pre-commit Hooks
```bash
# .pre-commit-config.yaml
repos:
  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: v14.0.0
    hooks:
      - id: clang-format
        types_or: [c++, c]
```

Install:
```bash
pip install pre-commit
pre-commit install
```

### Jetson Nano Cross-Compilation (v0.2.0)
- Toolchain file for aarch64
- CUDA-enabled OpenCV
- TensorRT integration for YOLO

### ROS2 Integration (v0.3.0)
- rclcpp node wrapper
- sensor_msgs/Image subscriber
- geometry_msgs/Twist publisher

---

## Testing Checklist Before v0.1.0 Release

- [ ] Builds successfully on Windows (MSVC 2022)
- [ ] Builds successfully on macOS (Clang 14+)
- [ ] All unit tests pass
- [ ] demo_viewer runs without crashes for 5 minutes
- [ ] aeroforge-track runs with colorball config
- [ ] aeroforge-track runs with aruco config
- [ ] FPS > 20 at 720p on laptop hardware
- [ ] Memory usage < 600 MB after 5 minutes
- [ ] No memory leaks (valgrind on Linux or Instruments on macOS)
- [ ] clang-format check passes
- [ ] CI pipeline green
- [ ] Documentation complete (README, BUILD_INSTRUCTIONS, ARCHITECTURE)
- [ ] Demo video recorded

---

## Questions to Answer

1. **Should we support Linux officially?**
   - Add linux-rel preset to CMakePresets.json
   - Test on Ubuntu 22.04

2. **Do we need CUDA/TensorRT for v0.1.0?**
   - No, defer to v0.2.0
   - OpenCV CPU path is sufficient for ArUco/colorball

3. **Should plugins be dynamically loaded?**
   - No for v0.1.0 (static linking)
   - Design plugin ABI for v0.4.0

4. **What's the testing strategy for field tests?**
   - See docs/SAFETY.md for protocol
   - Two-person rule: RC pilot + dev at laptop
   - Open field, low speed (< 2 m/s)
   - Mock drone first, then dry-run mode, then live

---

## Reference Documents

| Document | Purpose |
|----------|---------|
| `PROGRESS.md` | Detailed phase-by-phase status |
| `SESSION_SUMMARY.md` | Executive summary of what was built |
| `BUILD_INSTRUCTIONS.md` | Step-by-step build guide |
| `CONTRIBUTING.md` | Contribution guidelines |
| `README.md` | User-facing introduction |

---

## Quick Commands

```bash
# Build (Windows)
cmake --preset win-rel && cmake --build --preset win-rel -j

# Build (macOS)
cmake --preset mac-rel && cmake --build --preset mac-rel -j

# Test
ctest --preset [win|mac]-tests --output-on-failure

# Format code
find include src apps -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# Run demo
./build/.../demo_viewer

# Run track app
./build/.../aeroforge-track --config configs/track_colorball.yml
```

---

**Priority Order:**
1. Build verification ✅ (do this first!)
2. Pipeline orchestration (Phase 1)
3. Safety Manager (Phase 4)
4. Documentation & demo video (Phase 5)
5. Optional enhancements

**Estimated Time to v0.1.0**: 2-3 additional work sessions (4-6 hours)

**Good luck, and fly safe! 🚁**
