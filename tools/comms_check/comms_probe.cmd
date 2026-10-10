@echo off
rem Runs comms_probe.py with whatever Python this PC has (Anaconda first).
rem Usage from this folder:  .\comms_probe.cmd --ip 192.168.125.1 all
setlocal
set "PY=%USERPROFILE%\anaconda3\python.exe"
if exist "%PY%" goto run
where py >nul 2>nul
if %errorlevel%==0 set "PY=py" & goto run
set "PY=python"
:run
"%PY%" "%~dp0comms_probe.py" %*
endlocal
