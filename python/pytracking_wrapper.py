#!/usr/bin/env python3
"""
PyTracking Wrapper for AeroForge

This script provides a JSON-based IPC interface to pytracking trackers.
It communicates with the C++ application via stdin/stdout using JSON messages.

IMPORTANT: Only JSON should go to stdout. All other output goes to stderr.

Protocol:
- Input: JSON messages on stdin (one per line)
- Output: JSON responses on stdout (one per line)

Message types:
1. init: Initialize tracker with first frame and bounding box
   {
     "type": "init",
     "frame": "<base64-encoded-image>",
     "bbox": [x, y, width, height],
     "tracker_name": "dimp" (optional, default: "dimp")
   }
   Response: {"status": "ok"} or {"status": "error", "message": "..."}

2. track: Track object in new frame
   {
     "type": "track",
     "frame": "<base64-encoded-image>"
   }
   Response:
   {
     "status": "ok",
     "bbox": [x, y, width, height],
     "confidence": 0.95
   }

3. reset: Reset tracker
   {"type": "reset"}
   Response: {"status": "ok"}

4. quit: Shut down
   {"type": "quit"}
   Response: {"status": "ok"}
"""

# CRITICAL: Redirect stdout at BOTH Python and OS levels BEFORE any other imports
# This prevents PyTracking and its dependencies from corrupting our JSON stream
import sys
import os

# Duplicate file descriptor 1 (stdout) to preserve it for JSON output
_json_fd = os.dup(1)
# Use line-buffered stream to ensure timely delivery; we'll flush explicitly too
_json_out = os.fdopen(_json_fd, 'w', buffering=1)

# Protocol prefix to clearly mark JSON lines and avoid stdout noise
AF_JSON_PREFIX = "AFJSON "

# Redirect file descriptor 1 (stdout) to file descriptor 2 (stderr)
# This catches C-level output from PyTorch and other native libraries
os.dup2(2, 1)

# Also redirect Python's sys.stdout for Python-level print statements
sys.stdout = sys.stderr

# Suppress all warnings and info messages
import warnings
warnings.filterwarnings('ignore')

import json
import base64
import numpy as np
import cv2
from pathlib import Path
import logging

# Suppress ALL output except JSON
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['PYTHONWARNINGS'] = 'ignore'

# Disable matplotlib/visdom if they're used
os.environ['MPLBACKEND'] = 'Agg'

# Redirect logging to stderr to keep stdout clean for JSON
logging.basicConfig(level=logging.ERROR, stream=sys.stderr)

# Try to import pytracking
try:
    import importlib.util

    # Check if pytracking is installed as package
    pytracking_spec = importlib.util.find_spec("pytracking")
    if pytracking_spec is None:
        # If not installed as package, try to add from common locations
        possible_paths = [
            Path.home() / "pytracking",  # Windows: C:\Users\username\pytracking
            Path(__file__).parent.parent.parent / "pytracking",  # ../../../pytracking
            Path("/opt/pytracking"),  # Linux
        ]

        pytracking_found = False
        for path in possible_paths:
            if path.exists() and (path / "pytracking").exists():
                sys.path.insert(0, str(path))
                pytracking_found = True
                break

        if not pytracking_found:
            raise ImportError("PyTracking not found. Please ensure it's cloned to ~/pytracking")

    from pytracking.evaluation import Tracker
    PYTRACKING_AVAILABLE = True
    IMPORT_ERROR = None
except ImportError as e:
    PYTRACKING_AVAILABLE = False
    IMPORT_ERROR = str(e)
except Exception as e:
    PYTRACKING_AVAILABLE = False
    IMPORT_ERROR = f"Unexpected error: {str(e)}"


class PyTrackingWrapper:
    """Wrapper for pytracking trackers with JSON IPC interface."""

    def __init__(self):
        self.tracker = None
        self.tracker_name = "dimp"  # default tracker
        self.tracker_param = "dimp50"  # default parameters
        self.initialized = False

    def decode_frame(self, base64_data):
        """Decode base64-encoded image to numpy array (BGR format)."""
        try:
            # Decode base64 to bytes
            img_bytes = base64.b64decode(base64_data)

            # Convert bytes to numpy array
            nparr = np.frombuffer(img_bytes, np.uint8)

            # Decode image
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if img is None:
                raise ValueError("Failed to decode image")

            return img
        except Exception as e:
            raise ValueError(f"Image decode error: {e}")

    def init(self, frame_data, bbox, tracker_name=None):
        """
        Initialize tracker with first frame and bounding box.

        Args:
            frame_data: Base64-encoded image
            bbox: [x, y, width, height]
            tracker_name: Name of tracker (e.g., "dimp", "keep_track", "tomp")

        Returns:
            dict: Status response
        """
        if not PYTRACKING_AVAILABLE:
            return {
                "status": "error",
                "message": f"PyTracking not available: {IMPORT_ERROR}"
            }

        try:
            # Decode frame
            frame = self.decode_frame(frame_data)

            # Set tracker name if provided
            if tracker_name:
                self.tracker_name = tracker_name
                # Map tracker names to their default parameters
                param_map = {
                    "dimp": "dimp50",
                    "prdimp": "prdimp50",
                    "super_dimp": "super_dimp",
                    "keep_track": "default",
                    "tomp": "tomp50",
                }
                self.tracker_param = param_map.get(tracker_name, "default")

            # Suppress stdout during tracker creation (it prints info messages)
            import io
            import contextlib

            with contextlib.redirect_stdout(io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    # Create tracker instance
                    self.tracker = Tracker(self.tracker_name, self.tracker_param)

                    # Initialize tracker with first frame and bbox
                    # PyTracking uses [x, y, width, height] format
                    init_bbox = bbox  # Already in correct format

                    # Initialize - pytracking expects RGB, so convert
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    self.tracker.initialize(frame_rgb, {'init_bbox': init_bbox})

            self.initialized = True

            return {
                "status": "ok",
                "tracker": self.tracker_name,
                "param": self.tracker_param
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Initialization failed: {str(e)}"
            }

    def track(self, frame_data):
        """
        Track object in new frame.

        Args:
            frame_data: Base64-encoded image

        Returns:
            dict: Status and tracking results
        """
        if not self.initialized:
            return {
                "status": "error",
                "message": "Tracker not initialized. Call init first."
            }

        try:
            # Decode frame
            frame = self.decode_frame(frame_data)

            # Suppress output during tracking
            import io
            import contextlib

            with contextlib.redirect_stdout(io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    # Track - convert to RGB
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    output = self.tracker.track(frame_rgb)

            # Extract bounding box [x, y, width, height]
            bbox = output['target_bbox']

            # Get confidence if available
            confidence = output.get('confidence', output.get('max_score', 1.0))

            return {
                "status": "ok",
                "bbox": [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])],
                "confidence": float(confidence)
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Tracking failed: {str(e)}"
            }

    def reset(self):
        """Reset tracker state."""
        self.tracker = None
        self.initialized = False
        return {"status": "ok"}


def log_error(message):
    """Log error to stderr (won't interfere with JSON on stdout)."""
    print(f"[ERROR] {message}", file=sys.stderr, flush=True)


def log_info(message):
    """Log info to stderr (won't interfere with JSON on stdout)."""
    print(f"[INFO] {message}", file=sys.stderr, flush=True)


def main():
    """Main loop: read JSON from stdin, process, write JSON to stdout."""

    log_info("PyTracking wrapper started")
    log_info(f"PyTracking available: {PYTRACKING_AVAILABLE}")

    wrapper = PyTrackingWrapper()

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            try:
                # Parse JSON request
                request = json.loads(line)
                msg_type = request.get("type")

                # Handle request
                if msg_type == "init":
                    frame_data = request.get("frame")
                    bbox = request.get("bbox")
                    tracker_name = request.get("tracker_name", "dimp")

                    if not frame_data or not bbox:
                        response = {
                            "status": "error",
                            "message": "Missing frame or bbox"
                        }
                    else:
                        response = wrapper.init(frame_data, bbox, tracker_name)

                elif msg_type == "track":
                    frame_data = request.get("frame")

                    if not frame_data:
                        response = {
                            "status": "error",
                            "message": "Missing frame"
                        }
                    else:
                        response = wrapper.track(frame_data)

                elif msg_type == "reset":
                    response = wrapper.reset()

                elif msg_type == "quit":
                    response = {"status": "ok"}
                    print(AF_JSON_PREFIX + json.dumps(response), file=_json_out, flush=True)
                    log_info("Shutting down")
                    break

                else:
                    response = {
                        "status": "error",
                        "message": f"Unknown message type: {msg_type}"
                    }

                # Send response
                print(AF_JSON_PREFIX + json.dumps(response), file=_json_out, flush=True)

            except json.JSONDecodeError as e:
                error_response = {
                    "status": "error",
                    "message": f"JSON decode error: {str(e)}"
                }
                print(AF_JSON_PREFIX + json.dumps(error_response), file=_json_out, flush=True)

            except Exception as e:
                log_error(f"Unexpected error: {e}")
                error_response = {
                    "status": "error",
                    "message": f"Internal error: {str(e)}"
                }
                print(AF_JSON_PREFIX + json.dumps(error_response), file=_json_out, flush=True)

    except KeyboardInterrupt:
        log_info("Interrupted")

    except Exception as e:
        log_error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
