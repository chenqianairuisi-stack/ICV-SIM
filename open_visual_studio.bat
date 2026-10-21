@echo off
cd /d "%~dp0"
if not exist build\vs2022\ICV_SIM.sln (
  echo First generate the VS 2022 solution: cmake --preset vs2022
  pause
  exit /b 1
)
start "" "%~dp0build\vs2022\ICV_SIM.sln"
