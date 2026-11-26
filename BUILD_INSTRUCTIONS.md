# Build instructions

| Platform | Compiler | State |
|----------|----------|-------|
| Windows 11 x64 | Visual Studio 2022 (MSVC 14.44) | **Verified** 2026-11-26: builds, 38/38 tests pass |
| Windows x64 | Visual Studio 2019 | Fails at link time when a newer Visual Studio is also installed (see Troubleshooting) |
| macOS arm64 / x86-64 | Apple Clang | Presets provided, **not verified** since the October 2025 refactor |
| Linux x64 | GCC / Clang | Presets provided, **never verified** |

"Not verified" means nobody has built this revision there. Please report what happens.

The first configure builds every dependency from source through vcpkg. With the current
`vcpkg.json` (OpenCV with contrib) expect 30 to 60 minutes and several GB of disk. Later
builds reuse the result.

## Requirements

- CMake 3.23 or newer
- Git
- A C++20 compiler (see table)
- Windows: Visual Studio with the "Desktop development with C++" workload
- macOS: Xcode command line tools, `brew install ninja pkg-config nasm`
- Linux (Debian/Ubuntu): 
  ```bash
  sudo apt-get install -y build-essential cmake ninja-build git pkg-config nasm \
    autoconf autoconf-archive automake libtool libltdl-dev bison python3-jinja2 \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev \
    libgl1-mesa-dev libglu1-mesa-dev libwayland-dev libxkbcommon-dev libgtk-3-dev
  ```

No Python, GPU or DJI SDK is needed.

## 1. Get the source and vcpkg

vcpkg is not part of the repository. Clone it into `vcpkg/` (ignored by Git) at the commit
the dependencies were last tested with:

```bash
git clone https://github.com/IdanG7/AeroForge.git
cd AeroForge
git clone https://github.com/microsoft/vcpkg.git vcpkg
git -C vcpkg checkout adf54cf279585302d29f6789b85f56a9c4b7c75b
```

Then bootstrap it:

```powershell
.\vcpkg\bootstrap-vcpkg.bat -disableMetrics
```

```bash
./vcpkg/bootstrap-vcpkg.sh -disableMetrics
```

## 2. Configure, build, test

Windows (any shell; the preset uses the newest installed Visual Studio):

```powershell
cmake --preset win-rel
cmake --build --preset win-rel --parallel
ctest --preset win-tests
```

To pick a specific Visual Studio, name the generator on the first configure, for example
`cmake --preset win-rel -G "Visual Studio 17 2022"`. It must be the newest one installed,
because that is the one vcpkg uses for the dependencies.

macOS:

```bash
cmake --preset mac-rel
cmake --build --preset mac-rel --parallel
ctest --preset mac-tests
```

Linux:

```bash
cmake --preset linux-rel
cmake --build --preset linux-rel --parallel
ctest --preset linux-tests
```

Other presets: `win-debug`, `mac-debug` and `linux-debug` (Debug; the two Unix ones turn on
AddressSanitizer and UBSan), `linux-headless` (no GUI libraries linked, for servers and
CI). `cmake --list-presets` shows what applies to your platform.

## 3. Run

Executables are in `build/<preset>/bin/` (`bin/RelWithDebInfo/` with Visual Studio).

```powershell
# GUI, webcam, click-and-drag target selection
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_interactive.yml

# GUI, replaying a video file
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_interactive.yml --input path\to\clip.mp4

# No window: replay a file, follow the region x,y,w,h from the first frame, print a JSON report
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --headless --config configs\track_interactive.yml --input path\to\clip.avi --roi 100,120,64,64
```

`--help` lists all options. Run from the repository root so relative paths in configs
resolve.

Which video files open depends on the OpenCV backends on your platform (Media Foundation
on Windows, AVFoundation on macOS, GStreamer/V4L on Linux). Motion-JPEG `.avi` always
works because OpenCV decodes it itself.

## Options

| CMake option | Default | Meaning |
|--------------|---------|---------|
| `AEROFORGE_WITH_IMGUI` | `ON` | Build the GUI (GLFW + OpenGL + Dear ImGui). `OFF` gives a headless-only application |
| `AEROFORGE_BUILD_TESTS` | `ON` | Build tests |
| `AEROFORGE_BUILD_APPS` | `ON` | Build `aeroforge-track` and `demo_viewer` |
| `AEROFORGE_ENABLE_SANITIZERS` | `OFF` | ASan + UBSan, Clang/GCC Debug builds only |
| `AEROFORGE_WITH_DJI` | `OFF` | Not available: configuring with `ON` stops with an explanation |

Warnings are errors (`/W4 /WX`, `-Wall -Wextra -Werror -pedantic`).

## Troubleshooting

- **`Could not find toolchain file .../vcpkg/scripts/buildsystems/vcpkg.cmake`**: step 1
  was skipped; clone vcpkg into `vcpkg/`.
- **Configure seems to hang**: it is building dependencies. Progress is in
  `vcpkg/buildtrees/<package>/` and at the end in `build/<preset>/vcpkg-manifest-install.log`.
- **`generator does not match the generator used previously`**: the build directory was
  created with another generator. Reconfigure it with `cmake --fresh --preset <name>`.
- **`unresolved external symbol __std_find_trivial_1` (or similar `__std_*`) when linking
  tests**: the project is being built with an older Visual Studio than the one vcpkg used
  for the dependencies. Reconfigure with the newest installed generator:
  `cmake --fresh --preset win-rel -G "Visual Studio 17 2022"`.
- **The first frame from a webcam takes most of a minute on Windows**: fixed in the
  application (it disables OpenCV's Media Foundation hardware-transform probe). If you
  use OpenCV elsewhere, set `OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0`.
- **A dependency fails to build**: read the log path vcpkg prints. On Linux it is almost
  always a missing system package from the list above.
- **Webcam does not open on macOS**: grant camera access to your terminal in System
  Settings, Privacy & Security, Camera.
- **A video file does not open**: try a Motion-JPEG `.avi` to separate codec problems from
  path problems.

## Optional: PyTracking detector

A deep-learning tracker running in a Python process. Windows only at present. Setup is in
`PYTRACKING_INTEGRATION.md`; select it with `configs/track_pytracking.yaml`. The default
build and all tests work without it.
