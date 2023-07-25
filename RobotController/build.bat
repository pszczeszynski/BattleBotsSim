@echo off

REM Check if "SIMULATION" keyword is provided as a parameter
if /I "%~1"=="SIMULATION" (
    echo Configuring for SIMULATION mode
    cmake -DSIMULATION=ON -B ./build -S .
) else (
    echo Configuring for REAL mode
    cmake -DSIMULATION=OFF -B ./build -S .
)

REM Run the CMake build command
echo Building the project
cmake --build ./build --config Release --target RobotController
