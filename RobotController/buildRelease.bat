@echo off

REM Check if "SIMULATION" keyword is provided as a parameter
if /I "%~1"=="SIMULATION" (
    echo Configuring for SIMULATION mode
    cmake -DSIMULATION=ON -B ./build -S .
) else (
    echo Configuring for REAL mode
    cmake -DSIMULATION=OFF -B ./build -S .
)

if /I "%~1"=="XBOX" (
    echo Configuring for XBox Controller
    cmake -DXBOX=ON -B ./build -S .
) else if /I "%~2"=="XBOX" (
    echo Configuring for XBox Controller
    cmake -DXBOX=ON -B ./build -S .
) else (
    echo Configuring for PS5 Controller
    cmake -DXBOX=OFF -B ./build -S .
)

REM Run the CMake build command
echo Building the project
cmake --build ./build --config Release --target RobotController
