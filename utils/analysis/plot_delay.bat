@echo off
rem Lighthouse delay plot: double-click (newest log in Documents\Mission Planner\logs) or drop a .bin on this file.
cd /d "%~dp0"
py -3.11 plot_ext_nav_delay.py %*
pause
