@echo off
set "cmakeParams="
set "cleanBuild="
setlocal EnableDelayedExpansion

set "SIMULATION=OFF"

for %%a in (%*) do (
    echo "%%a"
    if "%%a"=="SIMULATION" set SIMULATION=ON
    if "%%a"=="SIM" set SIMULATION=ON
    if "%%a"=="VIDEO_FILES" set "cmakeParams=!cmakeParams! -DVIDEO_FILES=ON"
    if "%%a"=="VIDEO" set "cmakeParams=!cmakeParams! -DVIDEO_FILES=ON"
    if "%%a"=="FORCE_SIM_DATA" (
        if "!SIMULATION!" neq "ON" (
            echo "FORCE_SIM_DATA can only be used in SIMULATION mode"
            exit /b 1
        )
        set "cmakeParams=!cmakeParams! -DFORCE_SIM_DATA=ON"
    )
    if "%%a"=="CLEAN" set "cleanBuild=YES"
)

if "%SIMULATION%"=="ON" (
    set "cmakeParams=!cmakeParams! -DSIMULATION=ON"
)


if "%cleanBuild%"=="YES" (
	echo "Cleaning directories..."
	rm -fr ./build
)

REM Configure the cmake
echo cmake -B ./build -S . %cmakeParams% 
cmake -B ./build -S . %cmakeParams% 

REM Run the CMake build command
echo Building the project
cmake --build ./build --config Debug --target RobotController
