@echo off
cd /d "%~dp0"
start "" cmake-gui.exe --preset=vs2022
