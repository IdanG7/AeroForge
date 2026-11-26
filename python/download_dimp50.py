#!/usr/bin/env python3
"""Download DiMP50 model for PyTracking"""

import os
from pathlib import Path

try:
    import gdown
except ImportError:
    print("ERROR: gdown not installed")
    print("Run: pip install gdown")
    exit(1)

# Set up paths
pytracking_dir = Path.home() / "pytracking"
networks_dir = pytracking_dir / "pytracking" / "networks"

print(f"PyTracking directory: {pytracking_dir}")
print(f"Networks directory: {networks_dir}")
print()

# Create networks directory
networks_dir.mkdir(parents=True, exist_ok=True)
print(f"Created: {networks_dir}")
print()

# Download DiMP50
model_path = networks_dir / "dimp50.pth"
url = "https://drive.google.com/uc?id=1qgachgqks2UGjKx-GdO1qylBDdB1f9KN"

print("Downloading DiMP50 model (~70MB)...")
print("This may take a few minutes depending on your internet speed...")
print()

try:
    gdown.download(url, str(model_path), quiet=False)
    print()
    print("="*50)
    print("SUCCESS! Model downloaded")
    print("="*50)
    print(f"Location: {model_path}")
    print(f"Size: {model_path.stat().st_size / (1024*1024):.1f} MB")
except Exception as e:
    print()
    print(f"ERROR: {e}")
    print()
    print("Manual download:")
    print("1. Go to: https://github.com/visionml/pytracking")
    print("2. Click 'Model Zoo' in README")
    print("3. Download DiMP50")
    print(f"4. Save to: {model_path}")
    exit(1)
