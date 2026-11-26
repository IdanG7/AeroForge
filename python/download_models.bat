@echo off
REM Download PyTracking models

echo ===========================================
echo   Downloading PyTracking Models
echo ===========================================
echo.

set PYTRACKING_DIR=%USERPROFILE%\pytracking

REM Check if pytracking exists
if not exist "%PYTRACKING_DIR%" (
    echo ERROR: PyTracking not found at %PYTRACKING_DIR%
    pause
    exit /b 1
)

echo PyTracking directory: %PYTRACKING_DIR%
echo.

REM The models will be downloaded to: %PYTRACKING_DIR%\pytracking\networks
REM We'll download DiMP50 manually

echo Creating networks directory...
mkdir "%PYTRACKING_DIR%\pytracking\networks" 2>nul

echo.
echo Downloading DiMP50 model (~70MB)...
echo This may take a few minutes...
echo.

REM Use Python to download from Google Drive
python -c "import gdown; gdown.download('https://drive.google.com/uc?id=1qgachgqks2UGjKx-GdO1qylBDdB1f9KN', '%PYTRACKING_DIR%/pytracking/networks/dimp50.pth', quiet=False)"

if %errorlevel% neq 0 (
    echo.
    echo ERROR: Download failed!
    echo.
    echo Try manual download:
    echo 1. Go to: https://github.com/visionml/pytracking
    echo 2. Find "Model Zoo" section
    echo 3. Download DiMP50
    echo 4. Place in: %PYTRACKING_DIR%\pytracking\networks\
    echo.
    pause
    exit /b 1
)

echo.
echo Verifying download...
if exist "%PYTRACKING_DIR%\pytracking\networks\dimp50.pth" (
    echo.
    echo ===========================================
    echo   SUCCESS! Model downloaded
    echo ===========================================
    echo.
    echo Model location: %PYTRACKING_DIR%\pytracking\networks\dimp50.pth
    echo.
) else (
    echo.
    echo ERROR: Model file not found after download
    echo.
)

pause
