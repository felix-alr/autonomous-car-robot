@echo off
setlocal enabledelayedexpansion

REM Check if current directory name is "robot-src"
for %%d in ("%cd%") do set "CURDIR=%%~nxd"

if /I "%CURDIR%"=="robot-src" (
    echo Already in robot-src directory.
) else (
    if exist "robot-src" (
        echo Moving into robot-src directory...
        cd robot-src
    ) else (
        echo Error: No "robot-src" directory found in %cd%.
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

echo done.
endlocal
