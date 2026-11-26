# Technical debt and remediation backlog

Updated 2026-11-26. Priorities: **P0** blocks building, running or reliable testing.
**P1** functional correctness and critical architecture. **P2** performance, reliability,
maintainability. **P3** enhancements.

"Fixed" items stay listed for one release so the history is visible; their evidence is in
`IMPLEMENTATION_STATUS.md`.

## P0

| # | Item | State |
|---|------|-------|
| 0.1 | Working tree did not compile: unused variable in `pytracking_iface.cpp` under warnings-as-errors | Fixed |
| 0.2 | No tracking without a PyTorch/PyTracking install; C++ fallback unreachable; config `type` ignored | Fixed (ADR 0001) |
| 0.3 | Only webcam input; no replay, no headless mode, so nothing could be tested automatically | Fixed (`file` input, `--headless`) |
| 0.4 | Library module classes had no headers or factories and were duplicated inside the application | Fixed (`modules.hpp`, duplicates removed) |
| 0.5 | README and BUILD_INSTRUCTIONS tell you to run `vcpkg/bootstrap-vcpkg`, but `vcpkg/` is git-ignored and is not a submodule: a clean clone has no such directory | Docs fixed (clone step added). Decide: submodule, or `VCPKG_ROOT` |
| 0.6 | `ci.yml` only triggered on `main`/`develop`; the repository's branch is `master`, so it never ran. Windows preset asked for Ninja without an MSVC environment | Rewritten, **not yet run** (needs a push) |
| 0.7 | A CMake reconfigure on 2026-11-26 made vcpkg rebuild every dependency from source (44 minutes). Cause: vcpkg always uses the newest installed Visual Studio, which is now 2022 (14.44), while the build directory was generated for 2019. The mixed result does not link. The build directory was regenerated for 2022 | Mitigated and documented. Open: add `builtin-baseline` to `vcpkg.json` so versions are pinned; decide whether to pin the toolset in a triplet |

## P1

| # | Item | State |
|---|------|-------|
| 1.1 | Controller mixed camera and body frames; fixed `dt = 0.033`; PID divided by unchecked `dt` | Fixed (ADR 0002) |
| 1.2 | Safety config parsed but never enforced; F key toggled every frame while held | Fixed (`SafetyGate`) |
| 1.3 | Ground-plane estimator ignored tilt, accepted rays above the horizon, returned Y = 0 | Fixed |
| 1.4 | ArUco detector used an API removed in OpenCV 4.7 and was commented out of the build; example config had the wrong dictionary id | Fixed (uses `objdetect`), tested on a generated marker only |
| 1.5 | `now_seconds()` used `high_resolution_clock` (wall clock on some standard libraries); frame sequence number was reset every second by the FPS counter | Fixed |
| 1.6 | Integer-valued YAML lists (`kp: [1, 1, 1]`) were silently replaced by defaults | Fixed (`get_number(s)`) |
| 1.7 | **PyTracking POSIX path is broken**: `popen(..., "w")` gives a write-only pipe and replies are read from it. Never worked on macOS or Linux | Open. Rewrite with `fork`/`pipe`/`poll`, or state Windows-only in the build |
| 1.8 | PyTracking is synchronous: up to 1 s stall per frame on a hung child, 60 s on first selection. No protocol version, no request id. Frames go as base64 JPEG inside JSON, read back one byte at a time | Partly mitigated (bounded waits, restart). Open: move to a worker thread with a latest-frame slot and timestamped results; add `version` and `id` fields; length-prefixed binary frames |
| 1.9 | PyTracking shutdown closes the stderr handle while the drain thread may be reading it; startup always sleeps 1 s instead of waiting for a ready message | Open |
| 1.10 | Range from size trusts `object_diameter_m` for whatever the user selects | Mitigated (no default, documented). Open: per-selection size entry in the UI, or disable range for interactive targets |
| 1.11 | No gimbal or mount model: `camera_to_body` assumes a level forward camera, `body_to_ned` uses yaw only | Open. Needed before any comparison with aircraft telemetry |
| 1.12 | No camera calibration or lens distortion handling; intrinsics are hand-typed in configs | Open. Add an OpenCV calibration tool and a calibration file |
| 1.13 | `build.yml` tags and publishes a GitHub release automatically when the version in `CMakeLists.txt` changes on `master` | Open, **owner decision**: keep automatic, or require a manual trigger. Do not bump the version until decided |
| 1.14 | `python/venv/` (thousands of files) was untracked and not ignored: one `git add .` away from being committed | Fixed (`.gitignore`) |
| 1.15 | **Norfair link is POSIX only.** `SocketClient` uses Unix domain sockets, which CPython does not offer on Windows, so on Windows `type: norfair` only ever passes detections through | Open. Add a loopback TCP transport on both sides (service and client), keep the same length-prefixed MessagePack framing |
| 1.16 | Norfair is synchronous on the frame loop: each update waits up to `timeout_ms` (500 ms default) and a reconnect attempt up to 5 s. No request id, so a late reply to a timed-out request would be read as the answer to the next one (today the client disconnects on timeout, which avoids it) | Open. Same worker-thread design as 1.8 |
| 1.17 | Norfair tracker skips the service when a frame has no detections, so the service's tracks do not age during gaps; while in pass-through, ids are the detector's, and they change meaning when the service reconnects. Results carry no `Validity` (always `Fresh`) | Open |
| 1.18 | The Norfair path has never been run end to end in this architecture: POSIX socket code not compiled here, Python service not started here | Open. Run on the Mac: service + `configs/track_norfair.yml`, then add a CTest that starts the service |

## P2

| # | Item |
|---|------|
| 2.1 | `vcpkg.json` asks OpenCV for `contrib`, `nonfree` and `dnn`. Nothing uses them any more. They pull in Tesseract, HDF5, Protobuf and roughly triple the dependency build. Remove after confirming no local experiment needs them (costs one full rebuild) |
| 2.2 | `SpscRingBuffer` and `FramePool` are tested but unused. Either use them for an async capture stage (see 1.8) and run the two-thread test under TSan on Linux/macOS, or delete them |
| 2.3 | Kalman process noise is `q·I` per step, not scaled by `dt`; smoothing strength changes with frame rate. Position and velocity noise share one value |
| 2.4 | `KalmanTracker` follows only the first detection; the native tracker can report five |
| 2.5 | The GUI draws the video 1:1. Sources larger than the screen do not fit; mouse selection assumes no scaling |
| 2.6 | `HUD::is_key_pressed` returns key level, same as `is_key_down`. The application does its own edge detection. Fix the HUD API or remove the duplicate |
| 2.7 | Template tracker constants (search window, thresholds, 45-frame loss) are hard-coded; its motion model works in pixels per frame, so behaviour changes with frame rate |
| 2.8 | No sanitizer run has been done. Presets exist for Clang/GCC (`*-debug`); MSVC `/fsanitize=address` is not wired up |
| 2.9 | Two overlapping workflows (`ci.yml`, `build.yml`) build the same thing on every push; `build.yml` clones vcpkg at HEAD instead of a pinned commit |
| 2.10 | `clang-format`: fixed, all sources formatted with clang-format 18.1.8 (the version CI pins) |
| 2.11 | No recording: frames, detections and telemetry cannot be saved for replay. Replay of plain video exists |
| 2.12 | Root directory clutter (`PROGRESS.md`, `SESSION_SUMMARY.md`, `NEXT_STEPS.md`, `FILE_TREE.txt`): fixed, deleted on `master` |
| 2.13 | Placeholder contact e-mail in `SECURITY.md`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`: closed, the files were deleted on `master`. The repository now has no security contact or contribution guide |
| 2.14 | `stream` input (`rtsp://`, `udp://`) is wired but untested, and the vcpkg OpenCV build has no FFmpeg backend, so most URLs will not open |
| 2.15 | `demo_viewer` duplicates what `aeroforge-track` does without a pipeline |

## P3

| # | Item |
|---|------|
| 3.1 | iOS bridge app and telemetry schema (blocked on the hardware feasibility test) |
| 3.2 | Compressed video ingest from the bridge (H.264 over RTP or a simple framed TCP stream; needs an FFmpeg or platform decoder) |
| 3.3 | Benchmarks directory with a repeatable latency run and `docs/PERFORMANCE.md` |
| 3.4 | Dockable panels, log panel, config editor in the GUI |
| 3.5 | Learned detector (ONNX Runtime) once the CPU pipeline has benchmarks to compare against |
| 3.6 | PX4 SITL / Gazebo through MAVSDK if the kinematic simulator stops being enough |

## Correction (2026-11-26)

An earlier version of this file said Norfair, MessagePack IPC and Unix-socket communication
were not in the repository. That was wrong: the audit was done on a local `master` that
was two commits behind `origin/master`, where that work had already landed. It has since
been merged; see 1.15 to 1.18.
