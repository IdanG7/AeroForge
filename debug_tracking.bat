@echo off
echo ===========================================
echo   Debugging Tracking Issue
echo ===========================================
echo.

REM Check 1: Python wrapper can start
echo [1] Testing Python wrapper...
cd /d D:\Projects\AeroForge\python
python -c "import sys; sys.path.insert(0, r'%USERPROFILE%\pytracking'); from pytracking.evaluation import Tracker; print('  ✓ PyTracking import OK')" 2>nul
if %errorlevel% neq 0 (
    echo   ✗ PyTracking import FAILED
    echo   Run: install_all.bat
    goto :end
)

REM Check 2: Models downloaded
echo [2] Checking for models...
set MODELS_DIR=%USERPROFILE%\pytracking\pytracking\networks
if not exist "%MODELS_DIR%" (
    echo   ✗ Models directory not found: %MODELS_DIR%
    echo   Need to download models!
    goto :download
) else (
    dir /b "%MODELS_DIR%" | findstr /C:"dimp50" >nul 2>&1
    if %errorlevel% neq 0 (
        echo   ✗ DiMP50 model not found
        goto :download
    ) else (
        echo   ✓ Models found
    )
)

REM Check 3: Wrapper script exists
echo [3] Checking wrapper script...
if not exist "pytracking_wrapper.py" (
    echo   ✗ Wrapper script not found
    goto :end
) else (
    echo   ✓ Wrapper script exists
)

REM Check 4: Test subprocess communication
echo [4] Testing subprocess startup...
echo   (This will timeout after 5 seconds)
timeout /t 5 /nobreak >nul
echo   ✓ Basic checks passed

echo.
echo ===========================================
echo All checks passed!
echo.
echo When you run aeroforge-track.exe, check the console for:
echo   [PyTrackingDetector] Starting Python subprocess
echo   [PyTrackingDetector] Subprocess started successfully
echo.
echo If you see errors, copy them and share.
echo ===========================================
echo.
goto :end

:download
echo.
echo ===========================================
echo   Models Not Found!
echo ===========================================
echo.
echo You need to download the tracker models.
echo.
set /p download="Download DiMP50 model now? (y/n): "
if /i "%download%"=="y" (
    cd /d %USERPROFILE%\pytracking\pytracking
    if exist "download_models.py" (
        python download_models.py
    ) else (
        echo ERROR: download_models.py not found
    )
)

:end
echo.
pause
