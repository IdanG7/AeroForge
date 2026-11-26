# Windows Quick Start Guide - PyTracking Integration

## ✅ Good News!

The Windows native implementation is complete! You can now run PyTracking directly on Windows without WSL.

## 🚀 Quick Setup (15 minutes)

### Step 1: Install Python

Download and install Python 3.8 or later from https://www.python.org/downloads/

**Important:** Check "Add Python to PATH" during installation!

Verify installation:
```cmd
python --version
```

Should show: `Python 3.x.x`

### Step 2: Setup Python Environment

Open Command Prompt (cmd) and run:

```cmd
cd D:\Projects\AeroForge\python

REM Create virtual environment
python -m venv venv

REM Activate it
venv\Scripts\activate

REM Run the setup script
setup_windows.bat
```

The script will:
- Install Python dependencies (PyTorch, OpenCV, numpy, etc.)
- Clone pytracking repository to `%USERPROFILE%\pytracking`
- Install pytracking
- Optionally download pre-trained models (~70MB)

**When prompted about downloading models, type `y` and press Enter.**

This will take 5-10 minutes depending on your internet speed.

### Step 3: Test Python Wrapper

```cmd
REM Make sure you're in the python directory with venv activated
cd D:\Projects\AeroForge\python
python pytracking_wrapper.py
```

You should see:
```
[INFO] PyTracking wrapper started
[INFO] PyTracking available: True
```

**Success!** Press Ctrl+C to exit.

If you see errors, see the Troubleshooting section below.

### Step 4: Run AeroForge!

```cmd
cd D:\Projects\AeroForge

REM Make sure Python venv is activated
python\venv\Scripts\activate

REM Run the tracker
build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_default.yaml
```

The GUI should open showing your webcam feed!

## 🎮 How to Use

1. **Select a target**: Click and drag on the video to draw a box around an object
2. **Watch it track**: PyTracking will automatically track the selected object
3. **Reset**: Press `R` to select a new target
4. **Exit**: Press `ESC` to quit

## 🎯 Controls

| Key | Action |
|-----|--------|
| **Click & Drag** | Select object to track |
| **R** | Reset tracking (select new target) |
| **F** | Toggle control/safety mode |
| **SPACE** | Emergency stop |
| **ESC** | Exit application |

## 📊 Tracker Options

The default tracker is **DiMP** (fast and accurate). To try others, edit `apps\track\aeroforge-track.cpp` line 960:

```cpp
std::string tracker_name = "dimp";  // Change this!
```

Available options:
- `"dimp"` - Fast, accurate (30-50 FPS) - **Default**
- `"prdimp"` - Better uncertainty handling
- `"tomp"` - Best accuracy, slower
- `"keep_track"` - Best for occlusions

After changing, rebuild:
```cmd
cmake --build build\win-rel --config RelWithDebInfo --target aeroforge-track
```

## 🔧 Troubleshooting

### "Python is not recognized"

Python is not in your PATH. Either:
1. Reinstall Python with "Add to PATH" checked, OR
2. Use full path: `C:\Python39\python.exe` (adjust version)

### "PyTracking not available"

```cmd
cd python
venv\Scripts\activate
python -c "import pytracking"
```

If it fails:
```cmd
pip install -e %USERPROFILE%\pytracking
```

### "Failed to start subprocess"

Make sure:
1. Python virtual environment is activated
2. Wrapper script exists: `python\pytracking_wrapper.py`
3. Try running manually:
   ```cmd
   cd python
   venv\Scripts\activate
   python pytracking_wrapper.py
   ```

### Webcam not found

Edit `configs\track_default.yaml`:
```yaml
input:
  type: webcam
  device: 0  # Try changing to 1, 2, etc.
```

### Low FPS / Slow tracking

**Enable GPU acceleration:**
```cmd
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

This requires:
- NVIDIA GPU
- CUDA drivers installed

Check GPU is being used:
```cmd
python -c "import torch; print(torch.cuda.is_available())"
```

Should print `True`.

### "Model not found" error

Download models:
```cmd
cd %USERPROFILE%\pytracking\pytracking
python download_models.py
```

When prompted, download at least **dimp50**.

## 🎓 Next Steps

Once it's working:

1. **Try different trackers** - Edit the tracker name and rebuild
2. **Adjust camera settings** - Edit config file for resolution/FPS
3. **Test with different objects** - Try tracking faces, cars, balls, etc.
4. **Experiment with lighting** - See how robust the tracking is

## 📝 Quick Reference

**Every time you want to run:**
```cmd
cd D:\Projects\AeroForge
python\venv\Scripts\activate
build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_default.yaml
```

**Or create a shortcut:** Create `run_tracker.bat`:
```batch
@echo off
cd /d D:\Projects\AeroForge
call python\venv\Scripts\activate
build\win-rel\bin\RelWithDebInfo\aeroforge-track.exe --config configs\track_default.yaml
pause
```

Double-click to run!

## 💡 Tips

- **Better tracking**: Ensure good lighting and avoid motion blur
- **Faster startup**: Models are cached after first run
- **Multiple cameras**: Change `device: 0` in config to try other cameras
- **Record video**: Add video sink to config (see examples)

## 🐛 Still Having Issues?

Check the logs in the console output. Look for lines like:
- `[PyTrackingDetector]` - C++ subprocess communication
- `[INFO]` / `[ERROR]` - Python wrapper messages

Common issues:
- Python version < 3.8 (upgrade Python)
- Missing CUDA for GPU (install NVIDIA drivers)
- Antivirus blocking subprocess (add exception)

## 📚 More Information

- Full documentation: `PYTRACKING_INTEGRATION.md`
- PyTracking GitHub: https://github.com/visionml/pytracking
- Report issues: https://github.com/anthropics/aeroforge/issues

---

**Enjoy tracking! 🎯**
