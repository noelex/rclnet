@echo off
call "%RCLNET_SETUP%"
if errorlevel 1 exit /b 1
for /f "usebackq delims=" %%L in ("%RCLNET_OVERLAYS%") do (
    call "%%L"
    if errorlevel 1 exit /b 1
)
set "RMW_IMPLEMENTATION=%RCLNET_RMW%"
"%RCLNET_DOTNET%" "%~dp0Rcl.NET.TestWorker.dll" "%RCLNET_REQUEST%"
exit /b %errorlevel%
