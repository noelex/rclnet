@echo off
if not defined RCLNET_PIXI_MANIFEST goto direct
"%RCLNET_PIXI_EXE%" run --as-is --manifest-path "%RCLNET_PIXI_MANIFEST%" cmd.exe /d /c call "%~dp0worker.cmd"
exit /b %errorlevel%
:direct
call "%~dp0worker.cmd"
exit /b %errorlevel%
