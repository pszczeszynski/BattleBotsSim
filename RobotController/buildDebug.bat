@echo off
set "cmakeParams="
set "cleanBuild="
setlocal EnableDelayedExpansion

set "SIMULATION=OFF"
set "VIDEO_FILES_PASSED=OFF"
set "FORCE_SIM_DATA_PASSED=OFF"

for %%a in (%*) do (
    echo "%%a"
    if "%%a"=="SIMULATION" set "SIMULATION=ON"
    if "%%a"=="SIM" set "SIMULATION=ON"
    if "%%a"=="VIDEO_FILES" (
        set "VIDEO_FILES_PASSED=ON"
        set "cmakeParams=!cmakeParams! -DVIDEO_FILES=ON"
    )
    if "%%a"=="VIDEO" (
        set "VIDEO_FILES_PASSED=ON"
        set "cmakeParams=!cmakeParams! -DVIDEO_FILES=ON"
    )
    if "%%a"=="FORCE_SIM_DATA" (
        if "!SIMULATION!" neq "ON" (
            echo "FORCE_SIM_DATA can only be used in SIMULATION mode"
            exit /b 1
        )
        set "FORCE_SIM_DATA_PASSED=ON"
        set "cmakeParams=!cmakeParams! -DFORCE_SIM_DATA=ON"
    )
    if "%%a"=="CLEAN" set "cleanBuild=YES"
)

if "!SIMULATION!"=="ON" (
    set "cmakeParams=!cmakeParams! -DSIMULATION=ON"
) else (
    set "cmakeParams=!cmakeParams! -USIMULATION"
)

if "!VIDEO_FILES_PASSED!"=="OFF" (
    set "cmakeParams=!cmakeParams! -UVIDEO_FILES"
)

if "!FORCE_SIM_DATA_PASSED!"=="OFF" (
    set "cmakeParams=!cmakeParams! -UFORCE_SIM_DATA"
)

if "!cleanBuild!"=="YES" (
    echo "Cleaning directories..."
    rmdir /s /q .\build 2>nul
)

echo "cmakeParams: !cmakeParams!"

REM Configure the cmake
echo cmake -B ./build -S . -DCMAKE_BUILD_TYPE=Debug !cmakeParams!
cmake -B ./build -S . -DCMAKE_BUILD_TYPE=Debug !cmakeParams!

REM Run the CMake build command
echo Building the project
cmake --build ./build --config Debug --target RobotController

endlocal
