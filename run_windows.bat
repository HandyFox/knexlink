@echo off
REM Double-click this file to run Linkage Designer.
REM The first run sets up a private Python environment (a few minutes).
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo Setting up for the first time - this takes a few minutes...
    py -3 -m venv .venv 2>nul
    if not exist ".venv\Scripts\python.exe" python -m venv .venv
)
if not exist ".venv\Scripts\python.exe" (
    echo Could not create a Python environment. Is Python installed and on PATH?
    pause
    exit /b 1
)

if not exist ".venv\installed.txt" (
    ".venv\Scripts\python.exe" -m pip install --upgrade pip
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt || (
        echo Install failed - see the messages above.
        pause
        exit /b 1
    )
    echo done> ".venv\installed.txt"
)

".venv\Scripts\python.exe" main.py %*
if errorlevel 1 pause
