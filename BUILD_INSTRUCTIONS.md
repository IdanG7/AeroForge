# AeroForge Build Instructions

Complete guide for building AeroForge from source on Windows, macOS, and Linux.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Windows Build](#windows-build)
3. [macOS Build](#macos-build)
4. [Linux Build](#linux-build)
5. [Build Options](#build-options)
6. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### All Platforms

- **CMake** 3.21 or newer
- **Git** (for cloning and vcpkg submodules)
- **C++20 compiler**:
  - Windows: MSVC 2022 (Visual Studio 17.0+)
  - macOS: Clang 14+ (Xcode 14+)
  - Linux: GCC 11+ or Clang 14+

### Windows-Specific

- **Visual Studio 2022** with "Desktop development with C++" workload
- **Ninja** (optional but recommended for faster builds)
  ```powershell
  choco install ninja
  ```
- **Git for Windows** or **GitHub Desktop**

### macOS-Specific

- **Xcode Command Line Tools**:
  ```bash
  xcode-select --install
  ```
- **Homebrew** (for Ninja):
  ```bash
  brew install ninja
  ```

### Linux-Specific (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build git \
  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
  libgl1-mesa-dev libglu1-mesa-dev
```

For Fedora/RHEL:
```bash
sudo dnf install -y cmake ninja-build git gcc-c++ \
  libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
  mesa-libGL-devel mesa-libGLU-devel
```

---

## Windows Build

### Step 1: Clone Repository

```powershell
git clone https://github.com/YOUR_USERNAME/aeroforge.git
cd aeroforge
git submodule update --init --recursive
```

### Step 2: Bootstrap vcpkg

```powershell
.\vcpkg\bootstrap-vcpkg.bat
```

This will build vcpkg and prepare it for dependency management.

### Step 3: Configure CMake

**Option A: Using Presets (Recommended)**

```powershell
cmake --preset win-rel
```

**Option B: Manual Configuration**

```powershell
cmake -S . -B build -G Ninja `
  -DCMAKE_BUILD_TYPE=RelWithDebInfo `
  -DCMAKE_TOOLCHAIN_FILE="$PWD\vcpkg\scripts\buildsystems\vcpkg.cmake" `
  -DAEROFORGE_WITH_IMGUI=ON `
  -DAEROFORGE_WITH_DJI=OFF
```

### Step 4: Build

```powershell
cmake --build --preset win-rel -j
```

Or manually:
```powershell
cmake --build build --config RelWithDebInfo -j
```

### Step 5: Run

```powershell
# Demo viewer
.\build\win-rel\bin\RelWithDebInfo\demo_viewer.exe

# AeroForge-Track
.\build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_colorball.yml
```

### Step 6: Run Tests

```powershell
ctest --preset win-tests --output-on-failure
```

---

## macOS Build

### Step 1: Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/aeroforge.git
cd aeroforge
git submodule update --init --recursive
```

### Step 2: Bootstrap vcpkg

```bash
./vcpkg/bootstrap-vcpkg.sh
```

### Step 3: Configure CMake

```bash
cmake --preset mac-rel
```

Or manually:
```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/vcpkg/scripts/buildsystems/vcpkg.cmake" \
  -DAEROFORGE_WITH_IMGUI=ON \
  -DAEROFORGE_WITH_DJI=OFF
```

### Step 4: Build

```bash
cmake --build --preset mac-rel -j
```

### Step 5: Run

```bash
# Demo viewer
./build/mac-rel/bin/demo_viewer

# AeroForge-Track
./build/mac-rel/bin/aeroforge-track --config configs/track_colorball.yml
```

### Step 6: Run Tests

```bash
ctest --preset mac-tests --output-on-failure
```

---

## Linux Build

### Step 1: Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/aeroforge.git
cd aeroforge
git submodule update --init --recursive
```

### Step 2: Bootstrap vcpkg

```bash
./vcpkg/bootstrap-vcpkg.sh
```

### Step 3: Configure CMake

Linux doesn't have a preset by default, so configure manually:

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_TOOLCHAIN_FILE="$(pwd)/vcpkg/scripts/buildsystems/vcpkg.cmake" \
  -DAEROFORGE_WITH_IMGUI=ON \
  -DAEROFORGE_WITH_DJI=OFF
```

### Step 4: Build

```bash
cmake --build build -j$(nproc)
```

### Step 5: Run

```bash
# Demo viewer
./build/bin/demo_viewer

# AeroForge-Track
./build/bin/aeroforge-track --config configs/track_colorball.yml
```

### Step 6: Run Tests

```bash
cd build
ctest --output-on-failure
```

---

## Build Options

### CMake Options

| Option                         | Default | Description                                   |
|--------------------------------|---------|-----------------------------------------------|
| `AEROFORGE_WITH_IMGUI`         | `ON`    | Enable ImGui HUD (requires GLFW + OpenGL)     |
| `AEROFORGE_WITH_DJI`           | `OFF`   | Enable DJI SDK integration (experimental)     |
| `AEROFORGE_BUILD_TESTS`        | `ON`    | Build unit and integration tests              |
| `AEROFORGE_BUILD_APPS`         | `ON`    | Build demo_viewer and aeroforge-track apps    |
| `AEROFORGE_ENABLE_SANITIZERS`  | `OFF`   | Enable AddressSanitizer + UBSan (Debug only)  |

Example with options:
```bash
cmake --preset mac-rel \
  -DAEROFORGE_BUILD_TESTS=OFF \
  -DAEROFORGE_WITH_DJI=ON
```

### Debug Build

For debugging with symbols and sanitizers (macOS/Linux):

```bash
cmake --preset mac-debug  # or configure manually with Debug
cmake --build --preset mac-debug -j
```

On macOS Debug builds, sanitizers are enabled by default.

---

## Troubleshooting

### Issue: vcpkg fails to bootstrap

**Solution**: Ensure you have internet access and Git is installed. Try:
```bash
git submodule update --init --recursive
rm -rf vcpkg/buildtrees vcpkg/packages
./vcpkg/bootstrap-vcpkg.sh  # or .bat on Windows
```

### Issue: CMake can't find OpenCV

**Symptom**: `Could not find a package configuration file provided by "OpenCV"`

**Solution**: vcpkg manages dependencies. Ensure vcpkg toolchain is specified:
```bash
-DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg>/scripts/buildsystems/vcpkg.cmake
```

### Issue: ImGui headers not found

**Symptom**: `fatal error: imgui.h: No such file or directory`

**Solution**: Rebuild vcpkg dependencies:
```bash
./vcpkg/vcpkg install --triplet x64-windows  # Windows
./vcpkg/vcpkg install --triplet x64-osx      # macOS
./vcpkg/vcpkg install --triplet x64-linux    # Linux
```

### Issue: Webcam access denied (macOS)

**Symptom**: App crashes or fails to open camera

**Solution**: Grant camera permissions in System Preferences → Security & Privacy → Camera

### Issue: Linking errors on Windows

**Symptom**: `unresolved external symbol` errors

**Solution**:
1. Ensure you're building with the same configuration (Debug vs Release)
2. Clean and rebuild:
   ```powershell
   cmake --build build --target clean
   cmake --build build --config RelWithDebInfo -j
   ```

### Issue: Tests fail with "Segmentation fault"

**Solution**: Check if you're running out of memory. Reduce parallel jobs:
```bash
cmake --build build -j2  # instead of -j
```

---

## Advanced: Cross-Compilation for Jetson Nano

(Future support - planned for v0.2.0)

```bash
# On host machine
export CMAKE_TOOLCHAIN_FILE=<path-to-jetson-toolchain.cmake>
cmake -S . -B build-jetson -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DAEROFORGE_WITH_CUDA=ON
```

---

## Need Help?

- **GitHub Issues**: [Report a build problem](https://github.com/YOUR_USERNAME/aeroforge/issues)
- **Discussions**: [Ask a question](https://github.com/YOUR_USERNAME/aeroforge/discussions)
- **Discord**: [Join our community](https://discord.gg/YOUR_INVITE)

---

**Happy Building! 🚀**
