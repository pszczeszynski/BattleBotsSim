#!/bin/bash

# Check if "SIMULATION" keyword is provided as a parameter
if [ "${1^^}" == "SIMULATION" ]; then
    echo "Configuring for SIMULATION mode"
    cmake -DSIMULATION=ON -B ./build -S .
else
    echo "Configuring for REAL mode"
    cmake -DSIMULATION=OFF -B ./build -S .
fi

cmake -DWINDOWS=OFF -B ./build -S .

# Run the CMake build command
echo "Building the project"
cmake --build ./build --config Release --target RobotController
