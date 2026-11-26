# PyTracking Integration for AeroForge

This directory contains the Python integration for using [pytracking](https://github.com/visionml/pytracking) trackers in AeroForge.

## Overview

The integration uses a subprocess architecture where:
1. The C++ application launches `pytracking_wrapper.py` as a subprocess
2. Communication happens via JSON messages over stdin/stdout
3. Images are encoded as base64 for transmission
4. The wrapper handles all pytracking initialization and tracking calls

## Setup

### Prerequisites

- Python 3.8 or later
- PyTorch 2.0+
- CUDA (optional, for GPU acceleration)

### Installation

#### Option 1: Automated Setup (Linux/macOS)

```bash
cd python
chmod +x setup_pytracking.sh
./setup_pytracking.sh
```

#### Option 2: Manual Setup

1. **Create a virtual environment (recommended):**

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. **Install dependencies:**

```bash
pip install -r requirements.txt
```

3. **Clone and install pytracking:**

```bash
cd ~
git clone https://github.com/visionml/pytracking.git
cd pytracking
git submodule update --init --recursive
pip install -e .
```

4. **Download pre-trained models:**

```bash
cd pytracking
python download_models.py
```

This will download models for DiMP, PrDiMP, ToMP, KeepTrack, and other trackers.

### Verifying Installation

Test the wrapper manually:

```bash
cd python
python3 pytracking_wrapper.py
```

The script will start and wait for JSON input on stdin. You can test it with:

```json
{"type": "reset"}
```

It should respond with:
```json
{"status": "ok"}
```

Type `{"type": "quit"}` to exit.

## Available Trackers

The wrapper supports various pytracking trackers:

- **dimp** (default) - Discriminative Model Prediction tracker
  - Fast and accurate
  - Good for real-time applications
  - Default parameters: `dimp50`

- **prdimp** - Probabilistic DiMP
  - Handles uncertainty better
  - Slightly slower than DiMP

- **tomp** - Transformer-based tracker
  - State-of-the-art performance
  - Requires more compute

- **keep_track** - Long-term tracker
  - Good for handling occlusions
  - More robust to target loss

- **super_dimp** - Enhanced DiMP
  - Better accuracy than DiMP
  - Slightly slower

## Protocol

The wrapper uses a line-based JSON protocol over stdin/stdout.

### Initialize Tracker

**Request:**
```json
{
  "type": "init",
  "frame": "<base64-encoded-image>",
  "bbox": [x, y, width, height],
  "tracker_name": "dimp"
}
```

**Response:**
```json
{
  "status": "ok",
  "tracker": "dimp",
  "param": "dimp50"
}
```

### Track Frame

**Request:**
```json
{
  "type": "track",
  "frame": "<base64-encoded-image>"
}
```

**Response:**
```json
{
  "status": "ok",
  "bbox": [x, y, width, height],
  "confidence": 0.95
}
```

### Reset Tracker

**Request:**
```json
{"type": "reset"}
```

**Response:**
```json
{"status": "ok"}
```

### Quit

**Request:**
```json
{"type": "quit"}
```

**Response:**
```json
{"status": "ok"}
```

## Performance Considerations

- **Base64 Encoding Overhead**: Images are base64-encoded for transmission, which adds ~33% size overhead. This is acceptable for 30 FPS but can be optimized later with shared memory.

- **Frame Rate**: The wrapper can handle 30+ FPS with DiMP on modern GPUs (RTX 3060+).

- **GPU Acceleration**: PyTracking will automatically use CUDA if available. Make sure PyTorch is installed with CUDA support:
  ```bash
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
  ```

## Troubleshooting

### "PyTracking not available" error

Make sure pytracking is installed and in your Python path:
```bash
pip show pytracking
```

If not found, reinstall following the setup instructions.

### Low frame rate

- Enable GPU acceleration (install CUDA-enabled PyTorch)
- Use faster trackers (dimp instead of tomp)
- Reduce image resolution if possible

### Tracker loses target frequently

- Try more robust trackers: `keep_track` or `prdimp`
- Check that the initial bounding box is accurate
- Ensure good lighting and avoid motion blur

## Integration with C++

The C++ code uses `PyTrackingInterface` class (see `include/aeroforge/pytracking_iface.hpp`) to communicate with this wrapper.

Example usage:

```cpp
#include "aeroforge/pytracking_iface.hpp"

// Create interface
af::PyTrackingInterface tracker("python/pytracking_wrapper.py");

// Initialize with first frame and bbox
cv::Rect bbox(100, 100, 50, 50);
tracker.init(frame, bbox, "dimp");

// Track subsequent frames
auto result = tracker.track(frame);
if (result.success) {
    cv::rectangle(frame, result.bbox, cv::Scalar(0, 255, 0), 2);
}
```

## License

The Python wrapper script is part of AeroForge and follows the same license.

PyTracking itself is licensed under GPL-3.0. See the [pytracking repository](https://github.com/visionml/pytracking) for details.
