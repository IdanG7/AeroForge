# AeroForge Development Roadmap

## ✅ Phase 1: Foundation & Core Pipeline (COMPLETED)

### 1.1 Core Framework ✓
- [x] Cross-platform build system (CMake, vcpkg)
- [x] C++20 codebase with modern practices
- [x] Lock-free SPSC ring buffers for data flow
- [x] Timing utilities and frame management
- [x] Comprehensive unit tests (Catch2)
- [x] CI/CD pipeline (GitHub Actions)

### 1.2 Vision Pipeline ✓
- [x] **Interactive object selection** (click & drag)
- [x] Template matching tracker (no OpenCV contrib needed)
- [x] Kalman filter for smooth tracking
- [x] Size-based 3D pose estimation
- [x] Camera coordinate transformations

### 1.3 Control System ✓
- [x] PID velocity controller
- [x] Configurable gains (kp, ki, kd)
- [x] Safety interlock system
- [x] Emergency stop functionality
- [x] Mock drone interface for testing

### 1.4 User Interface ✓
- [x] **Professional redesigned UI**
- [x] Large camera feed window with interactive selection
- [x] Real-time telemetry display
- [x] Pipeline status monitoring
- [x] 3D position and velocity visualization
- [x] Color-coded status indicators
- [x] FPS and performance metrics

### 1.5 Configuration & Deployment ✓
- [x] YAML-based configuration system
- [x] Example configs (colorball tracking)
- [x] Comprehensive README and build instructions
- [x] GitHub Actions for automated builds
- [x] Artifact generation (.exe files)

---

## 🚧 Phase 2: Advanced Tracking & Robustness (NEXT)

### 2.1 Enhanced Tracking Algorithms
- [ ] **Multi-object tracking** (track multiple targets simultaneously)
- [ ] Improve template matching with scale adaptation
- [ ] Add feature-based tracking (ORB/SIFT) as alternative
- [ ] Implement tracking confidence scores
- [ ] Auto-reinitialization on tracking loss

**Priority**: HIGH
**Estimated Effort**: 2-3 weeks
**Why Next**: Current tracker works well but loses track on occlusions

### 2.2 Advanced Detection Methods
- [ ] Integrate YOLO for object detection
- [ ] Support custom trained models
- [ ] Add background subtraction for motion detection
- [ ] Color-based segmentation improvements
- [ ] Real-time performance profiling

**Priority**: MEDIUM
**Estimated Effort**: 3-4 weeks

### 2.3 Camera Calibration
- [ ] Built-in camera calibration tool
- [ ] Save/load calibration parameters
- [ ] Fisheye lens support
- [ ] Automatic calibration from checkerboard
- [ ] Distortion correction

**Priority**: MEDIUM
**Estimated Effort**: 1-2 weeks
**Why Important**: Improves 3D position accuracy

---

## 📊 Phase 3: Data Recording & Analysis

### 3.1 Data Recording
- [ ] Record video with synchronized telemetry
- [ ] Export tracking data to CSV/JSON
- [ ] Replay mode for offline analysis
- [ ] Frame-by-frame stepping
- [ ] Annotation tools

**Priority**: MEDIUM
**Estimated Effort**: 2 weeks
**Use Case**: Research, debugging, performance evaluation

### 3.2 Visualization & Debugging
- [ ] 3D visualization of drone and target positions
- [ ] Trajectory plotting (past N seconds)
- [ ] Performance graphs (latency, FPS over time)
- [ ] Debug mode with intermediate pipeline stages
- [ ] Export visualizations to image/video

**Priority**: LOW
**Estimated Effort**: 2-3 weeks

---

## 🚁 Phase 4: Real Drone Integration

### 4.1 DJI SDK Integration
- [ ] Complete DJI SDK wrapper implementation
- [ ] Test with DJI Tello
- [ ] Test with DJI Mavic series
- [ ] Implement all telemetry channels
- [ ] Handle connection loss gracefully

**Priority**: HIGH (if targeting real drones)
**Estimated Effort**: 3-4 weeks
**Dependency**: Access to DJI hardware

### 4.2 Safety Enhancements
- [ ] Geofencing with visual alerts
- [ ] Altitude limits enforcement
- [ ] Battery level monitoring
- [ ] Return-to-home on signal loss
- [ ] Collision avoidance basics

**Priority**: HIGH (for real deployment)
**Estimated Effort**: 2-3 weeks

### 4.3 Alternative Drone Platforms
- [ ] PX4/ArduPilot MAVLink support
- [ ] Betaflight integration
- [ ] Custom drone protocols
- [ ] Simulation integration (Gazebo, AirSim)

**Priority**: MEDIUM
**Estimated Effort**: 4-5 weeks

---

## ⚡ Phase 5: Performance & Optimization

### 5.1 CUDA Acceleration
- [ ] GPU-accelerated template matching
- [ ] CUDA-based image preprocessing
- [ ] TensorRT for neural network inference
- [ ] Multi-stream processing

**Priority**: LOW
**Estimated Effort**: 3-4 weeks
**Hardware Requirement**: NVIDIA GPU

### 5.2 Multi-threading Optimization
- [ ] Parallel pipeline stages
- [ ] Thread pool for image processing
- [ ] Lock-free data structures throughout
- [ ] CPU affinity optimization

**Priority**: MEDIUM
**Estimated Effort**: 2 weeks

---

## 🌐 Phase 6: Connectivity & Remote Control

### 6.1 Network Streaming
- [ ] WebRTC video streaming
- [ ] Remote control via web interface
- [ ] Multi-client support
- [ ] Low-latency H.264 encoding

**Priority**: LOW
**Estimated Effort**: 4-5 weeks

### 6.2 ROS2 Integration
- [ ] ROS2 bridge nodes
- [ ] Publish tracking data to ROS topics
- [ ] Subscribe to external commands
- [ ] Integration with ROS ecosystem

**Priority**: LOW
**Estimated Effort**: 2-3 weeks

---

## 🎯 Immediate Next Steps (Recommended Order)

### Sprint 1 (Week 1-2): Multi-Object Tracking
**Goal**: Track multiple objects simultaneously

**Tasks**:
1. Extend InteractiveTrackerDetector to manage multiple templates
2. Add UI for selecting multiple ROIs (Shift+Click)
3. Update Kalman tracker for multi-target
4. Display all tracked objects with unique IDs
5. Test with 2-3 objects in frame

**Deliverable**: Demo video tracking 3 different objects

---

### Sprint 2 (Week 3-4): Camera Calibration Tool
**Goal**: Improve 3D position accuracy

**Tasks**:
1. Create calibration application (separate from main app)
2. Implement checkerboard detection
3. Calculate camera matrix and distortion coefficients
4. Save calibration to YAML
5. Apply calibration in main app

**Deliverable**: Calibration tool + updated position estimates

---

### Sprint 3 (Week 5-6): Recording & Playback
**Goal**: Enable offline analysis

**Tasks**:
1. Add record button to UI
2. Save video + telemetry + tracking data
3. Implement playback mode
4. Frame-by-frame navigation
5. Export functionality

**Deliverable**: Recording system with playback

---

### Sprint 4 (Week 7-10): DJI Integration (Optional)
**Goal**: Deploy on real drone

**Tasks**:
1. Complete DJI SDK wrapper
2. Test with DJI Tello (easiest to start)
3. Implement safety checks
4. Field testing in open area
5. Documentation and demo

**Deliverable**: Working drone following system

---

## 📈 Success Metrics

### Phase 1 (Current) ✓
- ✅ Build success on Windows and macOS
- ✅ Interactive tracking working smoothly
- ✅ UI professional and responsive
- ✅ Full pipeline operational

### Phase 2 (Target)
- Track 3+ objects simultaneously at 30 FPS
- < 5% tracking loss rate in normal conditions
- Position accuracy within 10cm at 5m distance
- 95% test coverage

### Phase 3 (Target)
- Record/playback with no frame drops
- Export data in standard formats
- Complete debug visualization suite

### Phase 4 (Target)
- Successful outdoor flight tests
- No safety incidents in 100+ test flights
- Smooth following behavior in real conditions

---

## 💡 Feature Ideas (Backlog)

- Voice commands integration
- Gesture control for ROI selection
- Machine learning for tracking improvement
- Automatic object classification
- Formation flying (multiple drones)
- Obstacle detection via depth cameras
- Mobile app for remote monitoring
- Cloud telemetry dashboard

---

## 📞 Community & Contributions

We welcome contributions! Focus areas:
- Documentation improvements
- Example configurations
- Tutorial videos
- Bug reports and fixes
- Performance benchmarks
- New detector/tracker implementations

See [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

---

**Current Version**: v0.1.0
**Last Updated**: October 2025
**Maintainer**: IdanG7
