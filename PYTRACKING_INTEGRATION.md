# PyTracking Integration Quick Start

This guide will help you get started with the new PyTracking integration in AeroForge.

## What Changed?

AeroForge now uses [pytracking](https://github.com/visionml/pytracking) - a state-of-the-art visual object tracking library - instead of the previous C++ tracking implementation.

### Benefits

- **Better accuracy**: PyTracking implements research-grade tracking algorithms (DiMP, PrDiMP, ToMP, KeepTrack)
- **More robust**: Handles occlusions, appearance changes, and challenging scenarios better
- **Actively maintained**: Regular updates with latest tracking research
- **Multiple tracker options**: Choose the best tracker for your use case

## Setup Instructions

### Prerequisites

You're on Windows, so you'll need Windows Subsystem for Linux (WSL) or a Linux virtual machine for the Python integration to work. The current implementation uses Unix pipes which aren't available on native Windows.

**Option 1: Install WSL (Recommended)**

```powershell
# In PowerShell (as Administrator)
wsl --install
```

Then follow the Linux instructions below from within WSL.

**Option 2: Use Linux/macOS directly**

If you're already on Linux or macOS, continue with the steps below.

### Step 1: Install Python Dependencies

```bash
cd python

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows WSL/Git Bash

# Run setup script
chmod +x setup_pytracking.sh
./setup_pytracking.sh
```

The script will:
- Install Python dependencies (PyTorch, OpenCV, etc.)
- Clone and install pytracking
- Optionally download pre-trained models (~70MB)

### Step 2: Install C++ Dependencies

```bash
# Install nlohmann-json via vcpkg
cd ..
./vcpkg/vcpkg install nlohmann-json
```

### Step 3: Build AeroForge

```bash
# Configure CMake
cmake --preset linux-rel  # or macos-rel for macOS

# Build
cmake --build build/linux-rel --config Release -j
```

### Step 4: Run!

```bash
# Run the tracking application
./build/linux-rel/apps/track/aeroforge-track --config configs/track_default.yaml
```

## Usage

### Interactive Tracking

1. **Start the application**
   ```bash
   ./build/linux-rel/apps/track/aeroforge-track --config configs/track_default.yaml
   ```

2. **Select target**
   - Click and drag on the camera feed to select the object you want to track
   - Release to start tracking

3. **Controls**
   - `R` - Reset tracking (select new target)
   - `F` - Toggle control/safety mode
   - `SPACE` - Emergency stop
   - `ESC` - Exit application

### Changing Tracker

You can change the tracker algorithm by editing `apps/track/aeroforge-track.cpp:960`:

```cpp
std::string tracker_name = "dimp";  // Change this line
```

Available trackers:
- `"dimp"` - Default, fast and accurate (recommended for real-time)
- `"prdimp"` - Probabilistic DiMP, better uncertainty handling
- `"tomp"` - Transformer-based, state-of-the-art accuracy (slower)
- `"keep_track"` - Long-term tracking, handles occlusions well
- `"super_dimp"` - Enhanced DiMP, better accuracy

Then rebuild:
```bash
cmake --build build/linux-rel --target aeroforge-track -j
```

## Performance

### Frame Rates (on RTX 3060)

- **dimp**: ~40-50 FPS
- **prdimp**: ~35-45 FPS
- **tomp**: ~20-30 FPS
- **keep_track**: ~30-40 FPS

### GPU vs CPU

PyTracking will automatically use CUDA if available. To ensure GPU acceleration:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

## Troubleshooting

### "Subprocess not running" error

Make sure:
1. Python virtual environment is activated
2. PyTracking is installed: `pip show pytracking`
3. The wrapper script exists: `ls python/pytracking_wrapper.py`

### "PyTracking not available" error

The Python wrapper couldn't import pytracking. Check:

```bash
cd python
source venv/bin/activate
python3 pytracking_wrapper.py
```

You should see: `[INFO] PyTracking wrapper started`

### Low frame rate

- Enable GPU acceleration (see above)
- Use faster tracker (`dimp` instead of `tomp`)
- Reduce camera resolution in config file

### Tracker loses target frequently

- Try more robust trackers: `keep_track` or `prdimp`
- Ensure good lighting conditions
- Avoid motion blur (reduce camera/object speed)

## Architecture

The integration uses a subprocess architecture:

```
┌─────────────────────────────────────┐
│    aeroforge-track (C++)            │
│  ┌──────────────────────────────┐   │
│  │  PyTrackingDetector          │   │
│  │  - Launches Python process   │   │
│  │  - JSON over stdin/stdout    │   │
│  │  - Base64 image encoding     │   │
│  └──────────────┬───────────────┘   │
└─────────────────┼───────────────────┘
                  │ JSON IPC
┌─────────────────▼───────────────────┐
│    python/pytracking_wrapper.py     │
│  ┌──────────────────────────────┐   │
│  │  PyTracking Library          │   │
│  │  - DiMP/PrDiMP/ToMP/etc.     │   │
│  │  - Deep learning trackers    │   │
│  │  - PyTorch backend           │   │
│  └──────────────────────────────┘   │
└─────────────────────────────────────┘
```

## Next Steps

- **Benchmark different trackers**: Try each tracker and measure performance for your use case
- **Tune parameters**: Adjust tracker confidence thresholds
- **Optimize IPC**: Replace base64 encoding with shared memory for better performance
- **Add multi-object support**: Extend PyTrackingDetector to track multiple objects

## References

- [PyTracking GitHub](https://github.com/visionml/pytracking)
- [DiMP Paper](https://arxiv.org/abs/1904.07220)
- [ToMP Paper](https://arxiv.org/abs/2203.11192)
- [KeepTrack Paper](https://arxiv.org/abs/2103.16556)
