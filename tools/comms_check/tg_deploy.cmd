@echo off
rem Runs tg_deploy.py with whatever Python this PC has (Anaconda first).
rem Usage from this folder:  .\tg_deploy.cmd --ip 10.8.8.56 status
setlocal
set "PY=%USERPROFILE%\anaconda3\python.exe"
if exist "%PY%" goto run
where py >nul 2>nul
if %errorlevel%==0 set "PY=py" & goto run
set "PY=python"
:run
"%PY%" "%~dp0tg_deploy.py" %*
endlocal
