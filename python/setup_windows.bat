@echo off
REM Setup script for PyTracking on Windows

echo ===========================================
echo   AeroForge PyTracking Setup (Windows)
echo ===========================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python not found. Please install Python 3.8 or later from python.org
    echo https://www.python.org/downloads/
    pause
    exit /b 1
)

echo Found:
python --version
echo.

REM Check if in virtual environment
if defined VIRTUAL_ENV (
    echo Running in virtual environment: %VIRTUAL_ENV%
) else (
    echo WARNING: No virtual environment detected.
    echo It's recommended to create one:
    echo    python -m venv venv
    echo    venv\Scripts\activate
    echo.
    set /p continue="Continue anyway? (y/n): "
    if /i not "%continue%"=="y" exit /b 1
)

REM Install Python dependencies
echo Installing Python dependencies...
python -m pip install --upgrade pip
if %errorlevel% neq 0 (
    echo ERROR: Failed to upgrade pip
    pause
    exit /b 1
)

python -m pip install -r requirements.txt
if %errorlevel% neq 0 (
    echo ERROR: Failed to install requirements
    pause
    exit /b 1
)

echo.
echo Dependencies installed successfully.
echo.

REM Check if pytracking is already installed
python -c "import pytracking" >nul 2>&1
if %errorlevel% equ 0 (
    echo PyTracking is already installed.
    set /p update="Do you want to update it? (y/n): "
    if /i not "%update%"=="y" goto :done
)

REM Clone pytracking
set PYTRACKING_DIR=%USERPROFILE%\pytracking
if not exist "%PYTRACKING_DIR%" (
    echo.
    echo Cloning pytracking repository to %PYTRACKING_DIR%...
    git clone https://github.com/visionml/pytracking.git "%PYTRACKING_DIR%"
) else (
    echo.
    echo PyTracking directory already exists: %PYTRACKING_DIR%
    echo Updating...
    cd /d "%PYTRACKING_DIR%"
    git pull
)

REM Check if the directory exists now (whether we cloned or it existed)
if not exist "%PYTRACKING_DIR%" (
    echo ERROR: PyTracking directory not found. Clone may have failed.
    pause
    exit /b 1
)

REM Install pytracking
echo.
echo Installing pytracking...
cd /d "%PYTRACKING_DIR%"

REM Initialize submodules
git submodule update --init --recursive

REM Install pytracking in editable mode
python -m pip install -e .
if %errorlevel% neq 0 (
    echo ERROR: Failed to install pytracking
    pause
    exit /b 1
)

REM Download models
echo.
echo ===========================================
echo Would you like to download pre-trained models?
echo This will download DiMP50 (~70MB) and others.
echo ===========================================
set /p download_models="Download models? (y/n): "
if /i "%download_models%"=="y" (
    cd pytracking
    python download_models.py
)

:done
echo.
echo ===========================================
echo   Setup Complete!
echo ===========================================
echo.
echo PyTracking installed at: %PYTRACKING_DIR%
echo.
echo To test the wrapper:
echo   python python\pytracking_wrapper.py
echo.
echo The wrapper will wait for JSON input.
echo.
pause
