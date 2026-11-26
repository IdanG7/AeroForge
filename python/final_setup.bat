@echo off
REM Final setup - simpler approach

echo ===========================================
echo   PyTracking Final Setup
echo ===========================================
echo.

set PYTRACKING_DIR=%USERPROFILE%\pytracking

REM Verify pytracking directory exists
if not exist "%PYTRACKING_DIR%" (
    echo ERROR: PyTracking not found at %PYTRACKING_DIR%
    pause
    exit /b 1
)

echo PyTracking directory: %PYTRACKING_DIR%
echo.

REM Install additional dependencies that pytracking needs
echo Installing PyTracking dependencies...
pip install tensorboard visdom tikzplotlib gdown pandas matplotlib
if %errorlevel% neq 0 (
    echo WARNING: Some dependencies failed to install, but continuing...
)

echo.
echo ===========================================
echo Testing PyTracking import...
echo ===========================================
echo.

REM Create a test script
echo import sys > test_pytracking.py
echo sys.path.insert(0, r'%PYTRACKING_DIR%') >> test_pytracking.py
echo try: >> test_pytracking.py
echo     from pytracking.evaluation import Tracker >> test_pytracking.py
echo     print('SUCCESS: PyTracking imported successfully!') >> test_pytracking.py
echo     print('Available trackers: dimp, prdimp, tomp, keep_track, etc.') >> test_pytracking.py
echo except Exception as e: >> test_pytracking.py
echo     print(f'ERROR: {e}') >> test_pytracking.py
echo     sys.exit(1) >> test_pytracking.py

python test_pytracking.py
set TEST_RESULT=%errorlevel%
del test_pytracking.py

if %TEST_RESULT% neq 0 (
    echo.
    echo ERROR: PyTracking import failed!
    echo.
    echo This might be because:
    echo   1. PyTorch is not installed
    echo   2. Missing dependencies
    echo.
    echo Try installing PyTorch:
    echo   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
    echo.
    pause
    exit /b 1
)

echo.
echo ===========================================
echo Download pre-trained models?
echo ===========================================
echo This will download DiMP50 (~70MB)
set /p download_models="Download models? (y/n): "

if /i "%download_models%"=="y" (
    echo.
    echo Downloading models...
    cd /d "%PYTRACKING_DIR%\pytracking"
    if exist "download_models.py" (
        python download_models.py
    ) else (
        echo WARNING: download_models.py not found. You may need to download models manually.
        echo See: https://github.com/visionml/pytracking
    )
)

echo.
echo ===========================================
echo   Setup Complete!
echo ===========================================
echo.
echo PyTracking is ready to use!
echo Location: %PYTRACKING_DIR%
echo.
echo The wrapper script will automatically add PyTracking to the path.
echo.
echo Next step: Test the wrapper
echo   cd D:\Projects\AeroForge\python
echo   python pytracking_wrapper.py
echo.
pause
