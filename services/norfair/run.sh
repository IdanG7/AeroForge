#!/bin/bash
# Startup script for Norfair tracking service

set -e  # Exit on error

cd "$(dirname "$0")"

echo "=== Norfair Tracking Service ==="

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv

    echo "Installing dependencies..."
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
else
    source venv/bin/activate
fi

echo "Starting Norfair service..."
python norfair_service.py --config config.yaml
