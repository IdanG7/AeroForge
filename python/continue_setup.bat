@echo off
REM Continue setup after successful clone

echo Continuing PyTracking installation...
echo.

REM Set the pytracking directory
set PYTRACKING_DIR=%USERPROFILE%\pytracking

REM Check it exists
if not exist "%PYTRACKING_DIR%" (
    echo ERROR: PyTracking directory not found at %PYTRACKING_DIR%
    pause
    exit /b 1
)

echo Found pytracking at: %PYTRACKING_DIR%
echo.

REM Install pytracking
echo Installing pytracking...
cd /d "%PYTRACKING_DIR%"

REM Initialize submodules
echo Initializing submodules...
git submodule update --init --recursive
if %errorlevel% neq 0 (
    echo WARNING: Submodule initialization had issues, but continuing...
)

REM Install pytracking in editable mode
echo.
echo Installing pytracking package...
python -m pip install -e .
if %errorlevel% neq 0 (
    echo ERROR: Failed to install pytracking
    pause
    exit /b 1
)

echo.
echo ===========================================
echo   PyTracking installed successfully!
echo ===========================================
echo.

REM Download models
echo Would you like to download pre-trained models?
echo This will download DiMP50 (~70MB) and others.
set /p download_models="Download models? (y/n): "
if /i "%download_models%"=="y" (
    cd pytracking
    python download_models.py
)

echo.
echo ===========================================
echo   Setup Complete!
echo ===========================================
echo.
echo PyTracking installed at: %PYTRACKING_DIR%
echo.
echo To test the wrapper:
echo   cd D:\Projects\AeroForge\python
echo   python pytracking_wrapper.py
echo.
pause
