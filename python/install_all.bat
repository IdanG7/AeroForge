@echo off
REM Install EVERYTHING for pytracking - all dependencies at once

echo ===========================================
echo   Installing ALL PyTracking Dependencies
echo ===========================================
echo.

REM Install PyTorch with CUDA support first (this is big)
echo [1/3] Installing PyTorch with CUDA support...
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

REM Install all pytracking dependencies
echo.
echo [2/3] Installing PyTracking dependencies...
pip install opencv-python pillow numpy scipy matplotlib pandas tensorboard visdom tikzplotlib gdown jpeg4py cython ninja scikit-image

REM Install additional tracking-specific packages
echo.
echo [3/3] Installing tracking-specific packages...
pip install shapely pycocotools lmdb

echo.
echo ===========================================
echo   Testing PyTracking...
echo ===========================================
echo.

REM Test import
python -c "import sys; sys.path.insert(0, r'%USERPROFILE%\pytracking'); from pytracking.evaluation import Tracker; print('SUCCESS: PyTracking works!')"

if %errorlevel% neq 0 (
    echo.
    echo ERROR: PyTracking import still failed!
    echo Please check the error message above.
    pause
    exit /b 1
)

echo.
echo ===========================================
echo   All dependencies installed successfully!
echo ===========================================
echo.
echo Download models? (DiMP50 ~70MB)
set /p download="Type y to download: "

if /i "%download%"=="y" (
    cd /d %USERPROFILE%\pytracking\pytracking
    if exist "download_models.py" (
        python download_models.py
    )
)

echo.
echo ===========================================
echo   READY TO GO!
echo ===========================================
echo.
echo Test the wrapper:
echo   python pytracking_wrapper.py
echo.
pause
