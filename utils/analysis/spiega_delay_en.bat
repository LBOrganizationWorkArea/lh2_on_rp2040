@echo off
rem Step-by-step explanation of the delay, in English: double-click (newest log in Documents\Mission Planner\logs) or drop a .bin on this file.
cd /d "%~dp0"
py -3.11 spiega_delay.py %* --lang en
pause
