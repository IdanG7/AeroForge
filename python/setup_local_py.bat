@echo off
REM Copy local.py to pytracking

echo Copying local.py to pytracking...

set SOURCE=local.py
set DEST=%USERPROFILE%\pytracking\pytracking\evaluation\local.py

if not exist "%SOURCE%" (
    echo ERROR: local.py not found in current directory
    pause
    exit /b 1
)

copy /Y "%SOURCE%" "%DEST%"

if %errorlevel% equ 0 (
    echo.
    echo SUCCESS! local.py copied to:
    echo %DEST%
    echo.
    echo PyTracking is now configured!
    echo.
) else (
    echo.
    echo ERROR: Failed to copy file
    echo.
)

pause
