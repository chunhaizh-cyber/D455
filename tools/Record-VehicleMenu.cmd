@echo off
chcp 65001 >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Capture-VehicleVideoMenu.ps1" %*
exit /b %errorlevel%
