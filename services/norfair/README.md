# Norfair Tracking Service

Python service that provides multi-object tracking for AeroForge using the [Norfair](https://github.com/tryolabs/norfair) library.

## Overview

This service runs as a separate process and communicates with the AeroForge C++ application via Unix domain sockets using MessagePack binary protocol. It provides robust multi-object tracking with configurable parameters.

## Quick Start

```bash
# Install dependencies and run service
./run.sh
```

The service will:
1. Create a Python virtual environment (if needed)
2. Install dependencies from `requirements.txt`
3. Start the socket server on `/tmp/aeroforge_norfair.sock`

## Configuration

Edit `config.yaml` to adjust tracking parameters:

```yaml
tracker:
  distance_function: "euclidean"  # or "iou"
  distance_threshold: 30          # Lower = stricter matching
  hit_counter_max: 15             # Frames to survive without detection
  initialization_delay: 0          # Frames before new track starts
```

### Key Parameters

- **distance_function**: How to measure similarity between detections and tracks
  - `euclidean`: Distance between centroids (faster, good for non-overlapping objects)
  - `iou`: Intersection over Union of bounding boxes (better for overlapping objects)

- **distance_threshold**: Maximum distance to match a detection to a track
  - Lower values = stricter matching (fewer false matches, more track fragmentation)
  - Higher values = more lenient (fewer lost tracks, more false matches)

- **hit_counter_max**: Number of frames a track survives without a matching detection
  - Higher values = tracks survive longer occlusions
  - Lower values = faster removal of disappeared objects

## Protocol

The service uses MessagePack for efficient binary serialization.

### Request Format
```python
{
  "version": 1,
  "cmd": "update",
  "dt": 0.033,  # seconds since last frame
  "detections": [
    {
      "id": 0,
      "bbox": [x, y, width, height],
      "centroid": [cx, cy],
      "score": 0.95
    }
  ]
}
```

### Response Format
```python
{
  "version": 1,
  "success": true,
  "detections": [
    {
      "id": 1,  # Norfair tracking ID (persistent across frames)
      "bbox": [x, y, width, height],
      "centroid": [cx, cy],
      "score": 0.95
    }
  ]
}
```

## How It Works

1. **Detection Conversion**: AeroForge detections (bbox + centroid) are converted to Norfair's point-based format using 5 points: centroid + 4 corners

2. **Tracking**: Norfair maintains a Kalman filter for each tracked object, predicting motion and matching new detections

3. **ID Assignment**: Each track receives a unique ID that persists across frames, enabling multi-object tracking

4. **Result Conversion**: Tracked points are converted back to AeroForge format (bbox + centroid)

## Troubleshooting

**Service won't start:**
- Check Python 3.7+ is installed: `python3 --version`
- Check permissions on socket path: `ls -l /tmp/aeroforge_norfair.sock`

**Connection refused from C++:**
- Ensure service is running: `ps aux | grep norfair_service`
- Check socket file exists: `ls -l /tmp/aeroforge_norfair.sock`
- Verify socket path matches in both configs

**Poor tracking performance:**
- Tune `distance_threshold` based on your object size and motion
- Increase `hit_counter_max` if objects get lost during brief occlusions
- Try `iou` distance function for overlapping objects
- Enable DEBUG logging to see tracking decisions

## Dependencies

- Python 3.7+
- norfair >= 2.2.0
- msgpack >= 1.0.0
- numpy >= 1.20.0
- pyyaml >= 6.0

## Architecture

```
┌─────────────────────┐
│ norfair_service.py  │  ← Main entry point, socket server
│                     │
│  ┌──────────────┐   │
│  │ protocol.py  │   │  ← MessagePack serialization
│  └──────────────┘   │
│                     │
│  ┌──────────────┐   │
│  │ tracker.py   │   │  ← Norfair wrapper & format conversion
│  └──────────────┘   │
└─────────────────────┘
```

## License

Same as AeroForge (MIT)
