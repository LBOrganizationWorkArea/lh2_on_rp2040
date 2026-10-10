@echo off
setlocal
cd /d "%~dp0"
title LH2 bridge

rem --- Python environment (created once, in .venv next to this file) ---
if not exist ".venv\Scripts\python.exe" (
    echo Creazione ambiente Python in .venv ...
    py -3 -m venv .venv 2>nul || python -m venv .venv
    if not exist ".venv\Scripts\python.exe" (
        echo ERRORE: Python 3.10+ non trovato. Installalo da https://www.python.org e riprova.
        pause
        exit /b 1
    )
)
".venv\Scripts\python.exe" -c "import pymavlink, numpy, scipy, yaml, serial" 2>nul || (
    echo Installazione dipendenze ...
    ".venv\Scripts\python.exe" -m pip install --upgrade pip
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt || (pause & exit /b 1)
)

rem --- Web server for the editor (port 8765) + browser ---
start "LH2 editor HTTP" /min ".venv\Scripts\python.exe" -m http.server 8765 --bind 127.0.0.1 --directory "%~dp0."
timeout /t 2 >nul
start "" http://127.0.0.1:8765/bs_pose_editor.html

rem --- MAVLink UDP 14550 -> HTTP 8051 bridge (keep this window open) ---
".venv\Scripts\python.exe" utils\user_interface\display_real_time_windows.py %*
pause
