@echo off
rem Spiegazione passo per passo del ritardo: doppio clic (log piu recente in Documenti\Mission Planner\logs) o trascina un .bin su questo file.
cd /d "%~dp0"
py -3.11 spiega_delay.py %*
pause
