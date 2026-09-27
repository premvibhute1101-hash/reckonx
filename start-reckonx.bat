@echo off
title ReckonX Navigation Launcher
cls
echo =========================================================================
echo                    ReckonX Navigation ^& Telematics System                 
echo =========================================================================
echo.
echo Starting ReckonX services...
echo.
echo  [1/2] Launching Fastify Backend Server (Port 3001 + SQLite DB)...
start "ReckonX Backend Server (Port 3001)" cmd /k "cd /d %~dp0backend && npm run dev"

echo  [2/2] Launching Frontend Web App ^& Mobile QR Code Generator (Port 5173)...
timeout /t 2 /nobreak >nul
start "ReckonX Frontend Web App (HTTPS)" cmd /k "cd /d %~dp0 && npm run dev"

echo.
echo =========================================================================
echo  ReckonX Services Launched Successfully!
echo =========================================================================
echo.
echo  - Backend API:  http://localhost:3001
echo  - Frontend App: https://localhost:5173
echo.
echo  - Mobile Physical Testing: Scan the QR code printed in the Frontend
echo    terminal window using your smartphone (Same WiFi network required).
echo =========================================================================
echo.
pause
