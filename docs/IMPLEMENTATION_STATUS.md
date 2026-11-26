# Implementation status

Last updated 2026-11-26. Everything on this page was checked against the source and, where
it says "verified", against a run on the machine described below. Nothing is claimed from
the older README or progress notes.

## Environment the evidence comes from

| | |
|-|-|
| Machine | Windows 11 Pro (10.0.26300) x64, Intel Core i5-11600K, 32 GB, GTX 1080 (unused) |
| Compiler | Visual Studio 2022, MSVC 14.44.35207. Visual Studio 2019 (14.29) is also installed |
| Tools | CMake 4.1.2, vcpkg at commit `adf54cf2` |
| Key libraries | msgpack-cxx (no Boost), OpenCV 4.11.0, Eigen 3.4.1, Catch2 3.11.0, Dear ImGui 1.91.9, GLFW 3.4, yaml-cpp 0.8.0, spdlog 1.15.3, fmt 12.0.0 |
| Camera | one USB webcam, 1280x720 at 30 FPS |

**This is not the MacBook named in the project brief.** There is no macOS, Xcode or iPhone
on this machine, so no Apple-side work (macOS build, DJI SDK sample, iOS app) was or could
be run. Those items are listed as blocked or unverified below.

## Build and test

| Check | Result |
|-------|--------|
| Baseline before changes | **Did not compile.** `pytracking_iface.cpp(492)`: unused variable under `/WX`. After that one-line fix: built, 5/5 tests passed, one of them a placeholder |
| Configure (`cmake --fresh --preset win-rel -G "Visual Studio 17 2022"`) | Passed |
| Dependency build from source | Passed, **44 minutes** |
| Project build (`cmake --build --preset win-rel`), `/W4 /WX` | Passed, no warnings |
| Tests (`ctest --preset win-tests`) | **45 of 45 passed**, 8 s (after merging the Norfair work from `master`; 38 of 38 before) |
| Debug configuration in a new build directory (`win-debug` preset): configure, build, `ctest --preset win-debug-tests` | Passed, 45 of 45 after the merge. Dependencies came from vcpkg's binary cache in 3.5 s, so only the first build on a machine pays the 44 minutes |
| Visual Studio 2019 generator | **Fails at link time** on this machine: vcpkg builds the dependencies with the newest installed Visual Studio (2022), and its static libraries (Catch2) need the 2022 runtime. The libraries themselves compiled. Use the 2022 generator, or pin vcpkg to the v142 toolset |
| GCC 13.3 `-std=c++20 -Wall -Wextra -Werror -pedantic -fsyntax-only` under WSL, against the vcpkg headers | Passed for every library, application and test source except `src/ui/hud.cpp` and `demo_viewer.cpp` (not included in the run). Repeated after the merge for the three Norfair sources only, including the POSIX socket code. Diagnostics only: nothing was linked or executed on Linux |
| macOS, Linux, CI | **Not run** |

Test inventory: 39 Catch2 cases (Norfair serializer, tracker fallback and pipeline wiring 7, ring buffer and frame pool 3, PID 2, Kalman 2, estimators and frames 5,
controller and simulator 3, safety gate 3, config and pipeline construction 5, detectors 4,
tracking / replay / pipeline 5) and 6 application smoke tests.

## Application checks

All with `build/win-rel/bin/RelWithDebInfo/aeroforge-track.exe`, config
`configs/track_interactive.yml`.

| Check | Result |
|-------|--------|
| Headless replay of the generated 90-frame clip (640x480), target given by `--roi` | 90/90 frames tracked as fresh. Pipeline time per frame: p50 2.1 ms, p95 2.7 ms, max 3.2 ms. Worst centroid error against ground truth in the tracker test: 0.73 px |
| Headless, live webcam, 150 frames, 200x200 px region | 150/150 frames, no failed reads. Camera open 0.6 s. Pipeline p50 9.2 ms, p95 12.0 ms, max 22.6 ms. 27 FPS including startup |
| Headless, live webcam, 900 frames (30 s), same region | 900/900 frames, no failed reads, **29.5 FPS** sustained. Pipeline p50 8.1 ms, p95 10.6 ms, p99 12.3 ms, max 26.7 ms. Process memory 78.4 MB at 8 s and 78.5 MB at 26 s |
| GUI window, replaying the clip for 60 frames (`--max-frames 60`) | Window created (1050x710), target selected, clean shutdown, exit code 0 |
| Missing config, missing video, unknown argument | Error message and non-zero exit, no crash |
| GUI looked at by a person; mouse selection; F / SPACE / R keys; webcam in the GUI | **Not checked.** See the checklist at the end |

These timings are single runs on one machine, good for "is it in the right range", not a
benchmark. There is no benchmark suite yet.

Found while checking the webcam: OpenCV's Media Foundation backend took about **50 s** to
deliver the first frame. Disabling its hardware-transform probe (done automatically in
`src/io/webcam_capture.cpp`, unless the environment already sets it) brings that to under
2 s at the same 30 FPS. DirectShow was measured too: it works at 30 FPS only if MJPEG is
requested after the frame size, otherwise 10 FPS.

## Inventory

States: **Verified** (implemented and exercised by a test or run recorded above),
**Unverified** (implemented, not exercised), **Partial**, **Stub**, **Broken**, **Missing**,
**Obsolete**.

### Core and runtime

| Subsystem | State | Notes |
|-----------|-------|-------|
| `Frame`, `Detection`, `Validity` | Verified | Validity carried through detector, estimator, controller and gate |
| Monotonic time (`now_seconds`) | Verified by use | Was `high_resolution_clock` |
| YAML config, validation, error messages | Verified | `test_config.cpp` |
| Frame source: video file replay, deterministic timestamps, looping | Verified | Motion-JPEG AVI; other codecs depend on the platform backend |
| Frame source: webcam | Verified headless on Windows | Reopen-after-failure path exists, **not exercised** (needs a camera unplugged mid-run) |
| Frame source: network stream (`type: stream`) | Unverified | Wired through OpenCV; this OpenCV build has no FFmpeg, most URLs will not open |
| `Pipeline` (config to module chain, per-stage timing) | Verified | |
| `SpscRingBuffer` | Verified functionally | Two-thread test: 200,000 items through a 16-slot buffer, in order, none lost. Not run under a thread sanitizer. Not used by anything |
| `FramePool` | Verified | Could not be instantiated outside its own source file (missing destructor definition); fixed and tested. Not used by anything |
| Threads, queues, back-pressure, dropped-frame accounting | Missing | Deliberately single-threaded for now, see ARCHITECTURE |
| Recording of frames / observations | Missing | |

### Perception

| Subsystem | State | Notes |
|-----------|-------|-------|
| Native interactive tracker | Verified on synthetic data | Follows a textured target, reports loss as predicted then gone, reset and reselect work. **Not evaluated on real footage** beyond the 150-frame webcam run |
| Colour-blob detector | Verified on synthetic data | |
| ArUco detector | Verified on a generated marker | Was disabled and written against a removed API. No pose (`rvec`/`tvec`) output |
| Kalman tracker (centroid smoothing) | Verified through the pipeline test | First detection only; noise not scaled by `dt` |
| PyTracking bridge, Windows | Unverified | Compiles. Bounded waits and restart added. Not run here: needs PyTorch, a pytracking checkout and model files |
| PyTracking bridge, macOS / Linux | Broken | Write-only pipe, cannot read replies |
| Norfair tracker (`type: norfair`), MessagePack serializer | Partly verified | Merged from `master` on 2026-11-26 and wired into the module factory and pipeline. Verified on Windows by unit tests: request encoding, response parsing including malformed payloads, parameter validation, pass-through when the service is unreachable. **Never run against the Python service here** |
| Norfair socket client (Unix domain socket) | Unverified on macOS / Linux, unavailable on Windows | POSIX code is as merged and has not been compiled or run on this machine. On Windows `connect()` always fails, so the tracker stays in pass-through |
| Norfair Python service (`services/norfair`) | Unverified | Not run here |
| Camera calibration, lens distortion | Missing | Intrinsics are typed into the config |
| Learned detectors (YOLO etc.) | Missing | Mentioned in old docs only |

### Estimation, control, simulation, safety

| Subsystem | State | Notes |
|-----------|-------|-------|
| Size-based range | Verified against synthetic ground truth | Requires the true object size |
| Ground-plane range | Verified against synthetic ground truth | Level ground, fixed tilt, no roll |
| PID controller | Verified | Invalid `dt` and non-finite input handled |
| Follow controller (body frame, real `dt`) | Verified closed-loop in simulation | Converges to 3 m behind a static target within 5 cm |
| Kinematic simulator (`MockDrone`) | Verified | First-order velocity lag; no attitude, wind or battery |
| `SafetyGate` state machine and checks | Verified | Every refusal reason has a test |
| Simulation-only actuation | Verified | Non-simulated backend is refused |
| Gimbal / mount model, attitude compensation | Missing | |
| Kalman 3-D filter (`KalmanFilter3D`) | Unverified | Three basic tests; not used by anything |

### Application and UI

| Subsystem | State | Notes |
|-----------|-------|-------|
| `aeroforge-track` headless mode and JSON report | Verified | |
| `aeroforge-track` GUI | Partial | Starts, renders, exits cleanly. Interaction not checked by a person. Video drawn 1:1, no scaling |
| `demo_viewer` | Unverified | Builds; not run in this pass |
| Panels for logs, configuration editing, recording | Missing | |

### Hardware

| Subsystem | State | Notes |
|-----------|-------|-------|
| DJI desktop backend (`AEROFORGE_WITH_DJI`) | Obsolete | Pointed at a file that never existed; the option now stops the configure with an explanation |
| DJI Mini 2 compatibility | Researched only | `HARDWARE_COMPATIBILITY.md`; nothing tested |
| iOS SDK feasibility prototype | **Blocked** | Needs the MacBook, Xcode, the iPhone, a DJI developer account and the aircraft |
| iOS bridge app, drone video and telemetry ingest | Missing | Not started, by design, until the feasibility test passes |

### Build, CI, documentation

| Subsystem | State | Notes |
|-----------|-------|-------|
| Windows build with VS 2022 | Verified | |
| macOS build | Unverified | Last known good was before October 2025's PyTracking work |
| Linux build | Unverified | Never built |
| Sanitizer builds | Unverified | Presets exist for Clang/GCC |
| `ci.yml` | Unverified | Rewritten; has never run (the old one never triggered on `master`) |
| `build.yml` (build + automatic release) | Unverified | Not touched. Publishes a release when the version changes |
| clang-format / clang-tidy | Not run | Not installed locally |
| README, BUILD_INSTRUCTIONS | Rewritten to match the code | |
| `PROGRESS.md`, `SESSION_SUMMARY.md`, `NEXT_STEPS.md`, `FILE_TREE.txt`, `SECURITY.md`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md` | Removed | Deleted on `master` by the owner; the merge keeps that |

## Defects found in the audit

Fixed in this pass, each with a test unless noted:

1. Working tree did not compile.
2. Only the Python tracker was reachable; without PyTorch nothing could be tracked.
3. Config module `type` was ignored; `track_colorball.yml` did not run the colour detector.
4. Controller mixed camera and body axes, so "3 m behind" was applied sideways.
5. Controller assumed 30 Hz; PID divided by an unchecked time step.
6. `require_hold_to_enable`, speed limit, follow range and geofence were never enforced.
7. Holding F flipped the enable state every frame (no test: GUI key path).
8. Frame sequence number was reset every second by the FPS counter.
9. Ground-plane estimator ignored tilt and accepted points above the horizon.
10. ArUco detector did not build against OpenCV 4.7+; example config named the wrong dictionary.
11. Integer YAML lists were silently replaced by defaults.
12. `size_based` silently assumed a 0.20 m object.
13. Overlays were drawn into the frame before it could be reused; they are now drawn on a copy.
14. Webcam took about 50 s to open (measured, no automated test).
15. GUI window was created at video size while panels were placed outside it.
16. `python/venv/` was one `git add .` away from being committed.
17. `FramePool` could not be constructed from any other translation unit.

Open defects are in `TECHNICAL_DEBT.md`.

## Independent review

A separate reviewer read the change set on 2026-11-26 without building it. It confirmed
the frame conversions, the validity gating, the `SafetyGate` state machine, the single
command path and the clock pairing, and reported no blockers on the default path. Its
findings and what happened to them:

| Finding | Action |
|---------|--------|
| PyTracking POSIX: write-only pipe | Already known (TECHNICAL_DEBT 1.7), still open |
| PyTracking POSIX: stream closed twice on shutdown | Fixed |
| PyTracking Windows: script path not quoted | Fixed |
| PyTracking: fixed 1 s sleep at startup | Recorded (TECHNICAL_DEBT 1.9) |
| CI format job used an unpinned clang-format | Pinned to 18 |
| Template update blends 8-bit images | Not changed: OpenCV rounds to nearest there, so there is no systematic drift. Pre-existing tracker behaviour |
| Simulator telemetry uses simulation time | Not changed: documented in `telemetry.hpp` |

Not covered by that review: the CMake files, the two detectors and the Python wrapper.

## Open checks

- Nothing has been linked or run on macOS or Linux.
- CI has not run.

## Checklist for a person at the machine (about ten minutes)

Run `.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_interactive.yml`
and note what happens:

1. Does the window fit the screen, with the video on the left and four panels on the right?
2. Drag a box around an object with visible texture. Does a green box follow it?
3. Cover the object. Does the box turn orange ("predicted") and then disappear?
4. Press R, select something else. Does tracking restart cleanly?
5. Hold F. Does the Safety panel show `CONTROL: ENABLED` only while held, and a simulated
   velocity command appear when the target is 2 to 10 m away by the range estimate?
6. While holding F, tap SPACE. Does it show `E-STOP` and stay there until F is released?
7. Unplug the webcam for a few seconds and plug it back in. Does the window say
   `NO FRAMES`, then recover?
8. Run again with `--input <some .mp4 you have>`. Does it play at normal speed and stop at
   the end with "End of recording"?
