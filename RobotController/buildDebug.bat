@echo off
setlocal EnableDelayedExpansion
set "cmakeParams="
set "cleanBuild="

for %%a in (%*) do (
    echo "%%a"
    if "%%a"=="SIMULATION" set "cmakeParams=!cmakeParams! -DSIMULATION=ON"
    if "%%a"=="VIDEO_FILES" set "cmakeParams=!cmakeParams! -DVIDEO_FILES=ON"
    if "%%a"=="VIDEO" set "cmakeParams=!cmakeParams! -DVIDEO_FILES=ON"
    if "%%a"=="XBOX" set "cmakeParams=!cmakeParams! -DXBOX=ON"
	if "%%a"=="CLEAN" set "cleanBuild=YES"
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
