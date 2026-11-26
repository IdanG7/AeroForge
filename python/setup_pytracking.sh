#!/bin/bash
# Setup script for PyTracking integration with AeroForge

set -e

echo "=================================================="
echo "  AeroForge PyTracking Setup"
echo "=================================================="
echo ""

# Check Python version
PYTHON_CMD=""
if command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null; then
    PYTHON_CMD="python"
else
    echo "ERROR: Python 3 not found. Please install Python 3.8 or later."
    exit 1
fi

echo "Found Python: $PYTHON_CMD"
PYTHON_VERSION=$($PYTHON_CMD --version)
echo "Version: $PYTHON_VERSION"
echo ""

# Check if running in virtual environment
if [[ -z "${VIRTUAL_ENV}" ]]; then
    echo "WARNING: No virtual environment detected."
    echo "It's recommended to create a virtual environment:"
    echo "  python3 -m venv venv"
    echo "  source venv/bin/activate  # On Windows: venv\\Scripts\\activate"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install Python dependencies
echo "Installing Python dependencies..."
$PYTHON_CMD -m pip install --upgrade pip
$PYTHON_CMD -m pip install -r requirements.txt

echo ""
echo "Dependencies installed."
echo ""

# Clone pytracking if not already present
PYTRACKING_DIR="$HOME/pytracking"
if [ ! -d "$PYTRACKING_DIR" ]; then
    echo "PyTracking not found at $PYTRACKING_DIR"
    echo "Cloning pytracking repository..."
    git clone https://github.com/visionml/pytracking.git "$PYTRACKING_DIR"

    echo ""
    echo "Setting up pytracking..."
    cd "$PYTRACKING_DIR"

    # Initialize submodules
    git submodule update --init --recursive

    # Install pytracking
    $PYTHON_CMD -m pip install -e .

    # Download pre-trained models (optional)
    echo ""
    echo "=================================================="
    echo "Would you like to download pre-trained models?"
    echo "This will download DiMP50 (~70MB) and other models."
    echo "=================================================="
    read -p "Download models? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd pytracking
        $PYTHON_CMD download_models.py
    fi
else
    echo "PyTracking already installed at $PYTRACKING_DIR"
    echo "Updating pytracking..."
    cd "$PYTRACKING_DIR"
    git pull
    $PYTHON_CMD -m pip install -e .
fi

echo ""
echo "=================================================="
echo "  Setup Complete!"
echo "=================================================="
echo ""
echo "PyTracking is installed at: $PYTRACKING_DIR"
echo ""
echo "To test the wrapper, run:"
echo "  $PYTHON_CMD pytracking_wrapper.py"
echo ""
echo "The wrapper will wait for JSON input on stdin."
echo ""
