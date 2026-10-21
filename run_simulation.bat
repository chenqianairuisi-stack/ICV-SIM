@echo off
cd /d "%~dp0"
python run_offline.py
if errorlevel 1 (
  echo Simulation failed. Check the messages above.
  pause
  exit /b 1
)
python tools\make_delivery.py
if errorlevel 1 (
  echo Animation export failed. The simulation data is still available.
  pause
  exit /b 1
)
start "" "%~dp0web\index.html"
pause
