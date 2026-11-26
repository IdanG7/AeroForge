# Windows Setup Guide for PyTracking Integration

This guide will help you test the PyTracking integration on Windows using WSL (Windows Subsystem for Linux).

## Why WSL?

The current implementation uses Unix pipes for subprocess communication. Native Windows support can be added later, but for now, WSL provides the fastest path to testing.

## Step 1: Install WSL (if not already installed)

Open PowerShell as Administrator and run:

```powershell
wsl --install
```

This installs Ubuntu by default. Restart your computer when prompted.

After restart, Ubuntu will open automatically. Create a username and password when prompted.

## Step 2: Access Your Project in WSL

Your Windows drives are mounted under `/mnt/` in WSL.

```bash
# Open WSL (search "Ubuntu" in Start menu)
cd /mnt/d/Projects/AeroForge
```

## Step 3: Install Linux Build Tools

```bash
# Update package lists
sudo apt update

# Install build essentials
sudo apt install -y build-essential cmake git pkg-config

# Install required libraries
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libfmt-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    nlohmann-json3-dev
```

## Step 4: Install Python and Dependencies

```bash
# Install Python 3 and pip
sudo apt install -y python3 python3-pip python3-venv

# Navigate to python directory
cd /mnt/d/Projects/AeroForge/python

# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Install PyTorch with CUDA support (if you have NVIDIA GPU)
# Otherwise, skip this and it will use CPU
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

## Step 5: Install PyTracking

```bash
# Still in the python directory with venv activated
cd ~
git clone https://github.com/visionml/pytracking.git
cd pytracking

# Initialize submodules
git submodule update --init --recursive

# Install pytracking
pip install -e .

# Download pre-trained models (DiMP50 - about 70MB)
cd pytracking
python download_models.py
# When prompted, download at least "dimp50"
```

## Step 6: Test Python Wrapper

```bash
# Go back to AeroForge python directory
cd /mnt/d/Projects/AeroForge/python

# Make sure venv is activated
source venv/bin/activate

# Test the wrapper
python3 pytracking_wrapper.py
```

You should see:
```
[INFO] PyTracking wrapper started
[INFO] PyTracking available: True
```

The script is waiting for input. Press Ctrl+C to exit.

**If it works, the Python side is ready! ✅**

## Step 7: Build AeroForge in WSL

```bash
cd /mnt/d/Projects/AeroForge

# Create build directory for Linux
mkdir -p build/linux-rel
cd build/linux-rel

# Configure CMake
cmake ../.. \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DAEROFORGE_WITH_IMGUI=ON \
    -DAEROFORGE_WITH_DJI=OFF \
    -DAEROFORGE_BUILD_TESTS=ON \
    -DAEROFORGE_BUILD_APPS=ON

# Build
cmake --build . -j$(nproc)
```

**Note:** This might take 5-10 minutes on first build.

## Step 8: Run the Tracker!

First, you'll need an X server to display the GUI from WSL.

### Install X Server on Windows

Download and install **VcXsrv** (free):
- Download from: https://sourceforge.net/projects/vcxsrv/
- Install it
- Run **XLaunch** from Start menu
- Choose settings:
  - Display: "Multiple windows"
  - Start: "Start no client"
  - Extra: Check "Disable access control"
  - Finish

### Set DISPLAY variable in WSL

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

### Run AeroForge!

```bash
cd /mnt/d/Projects/AeroForge

# Make sure Python venv is activated
source python/venv/bin/activate

# Run the tracker
./build/linux-rel/apps/track/aeroforge-track --config configs/track_default.yaml
```

The GUI should open! 🎉

## Usage

1. **Select a target**: Click and drag on the video feed to select an object
2. **Watch it track**: PyTracking will track the selected object
3. **Reset**: Press `R` to select a new target
4. **Exit**: Press `ESC` to quit

## Troubleshooting

### "Cannot open display"

Make sure:
1. VcXsrv is running (look for X icon in system tray)
2. DISPLAY variable is set:
   ```bash
   echo $DISPLAY
   # Should show something like: 172.x.x.x:0
   ```
3. Try disabling Windows firewall temporarily

### "PyTracking not available"

```bash
# Make sure venv is activated
source python/venv/bin/activate

# Test import
python3 -c "from pytracking.evaluation import Tracker; print('OK')"
```

Should print `OK`.

### "Subprocess not running"

Make sure:
1. Python venv is activated before running aeroforge-track
2. Wrapper script is executable:
   ```bash
   chmod +x python/pytracking_wrapper.py
   ```

### GUI is slow/laggy

This is normal for WSL + X server. Performance suggestions:
- Use lower camera resolution (edit config file)
- Use faster tracker (dimp instead of tomp)
- Run on native Linux if possible

## Alternative: Native Windows Build (Future)

For native Windows support without WSL, we would need to:
1. Implement Windows subprocess handling using `CreateProcess`
2. Use Windows named pipes instead of Unix pipes
3. Or use TCP sockets for cross-platform IPC

Let me know if you want me to implement native Windows support!

## Quick Reference

### Every time you want to run:

```bash
# 1. Open WSL (Ubuntu)
# 2. Navigate to project
cd /mnt/d/Projects/AeroForge

# 3. Activate Python environment
source python/venv/bin/activate

# 4. Set display (if not in .bashrc)
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

# 5. Run
./build/linux-rel/apps/track/aeroforge-track --config configs/track_default.yaml
```

### Add to .bashrc for convenience:

```bash
echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk "{print \$2}"):0' >> ~/.bashrc
```

## Performance Notes

- **WSL2** is much faster than WSL1. Check version:
  ```bash
  wsl -l -v  # Run in PowerShell
  ```
  Should say "VERSION 2"

- **GPU access**: WSL2 supports CUDA if you have NVIDIA GPU and install drivers
  - Install CUDA toolkit in WSL
  - PyTorch will automatically use GPU

## Next Steps

Once you have it working:
1. Try different trackers (edit `apps/track/aeroforge-track.cpp:960`)
2. Adjust camera settings in config file
3. Test with different objects and lighting conditions
