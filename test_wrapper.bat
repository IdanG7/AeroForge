@echo off
REM Quick test of the Python wrapper

cd /d D:\Projects\AeroForge\python

echo Testing Python wrapper...
echo.
echo You should see:
echo   [INFO] PyTracking wrapper started
echo   [INFO] PyTracking available: True
echo.
echo If you see that, press Ctrl+C to stop.
echo.

python pytracking_wrapper.py

pause
