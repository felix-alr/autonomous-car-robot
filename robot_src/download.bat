@echo off
setlocal enabledelayedexpansion

REM --- Check if mpremote is accessible ---
where mpremote >nul 2>nul
if errorlevel 1 (
    echo Error: "mpremote" command not found in PATH.
    echo Please install mpremote and make sure to add it to your system PATH, see general manual chapter 6.
    exit /b 1
)

REM Check if current directory name is "robot-src"
for %%d in ("%cd%") do set "CURDIR=%%~nxd"

if /I "%CURDIR%"=="robot_src" (
    echo Already in robot_src directory.
) else (
    if exist "robot_src" (
        echo Moving into robot_src directory...
        cd robot_src
    ) else (
        echo Error: No "robot_src" directory found in %cd%.
        exit /b 1
    )
)

REM Store base path after moving (or staying)
set "BASE=%cd%"

REM copy all files in robot-src
for %%f in (*.py) do (
    set "FULL=%%f"
    set "REL=!FULL:%BASE%\=!"
    mpremote cp !REL! :!REL!
)

REM copy pololu library
cd pololu_3pi_2040_robot
mpremote cp -r . :pololu_3pi_2040_robot

REM reset after flashing
mpremote reset

echo done.
endlocal
