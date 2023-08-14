cmake_minimum_required(VERSION 3.16)
project(RobotController VERSION 0.1.0 LANGUAGES CXX)
# set(CMAKE_CUDA_ARCHITECTURES 86) # look up the cuda architecture for your gpu. For my RTX 3080 it's 8.6

# set C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set policy for find_package to use <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)

# # Set the directory you want to add to the PATH
# set(MY_DIRECTORY "${CMAKE_SOURCE_DIR}/libs/DS5W/")

# # Add the directory to the PATH using an environment variable
# set(ENV{PATH} "${MY_DIRECTORY};$ENV{PATH}")

# # Print the updated PATH for verification (optional)
# message("Updated PATH: $ENV{PATH}")

# add define for SIMULATION if SIMULATION is an arg
option(SIMULATION "Build for simulation" OFF)
if(SIMULATION)
    add_definitions(-DSIMULATION)
endif()

# add define for XBOX if XBOX is an arg
option(XBOX "Build for xbox controller" OFF)
if(XBOX)
    add_definitions(-DXBOX)
endif()

# TODO: make this not hardcoded
list(APPEND CMAKE_PREFIX_PATH "C:/Qt/6.5.1/msvc2019_64")

find_package( OpenCV REQUIRED )
# find_package( CUDA REQUIRED )
find_package(Qt6 REQUIRED COMPONENTS Widgets)
qt_standard_project_setup()

add_definitions(-D_MBCS)

set(JSON_INCLUDE_PATH "./libs/json/include")
include_directories( ${OpenCV_INCLUDE_DIRS} ${JSON_INCLUDE_PATH} )

# Define the sources and headers
set(SOURCES 
    Globals.cc
    RobotConfig.cc
    RobotController.cc
    MathUtils.cc
    Vision.cc
    CameraReceiver.cc
    ServerSocket.cc
    RobotStateParser.cc
    Clock.cc
    RobotOdometry.cc
    ValueBin.cc
    TrackingUtils.cc
    Timer.cc
    OpponentProfile.cc
    PathFinder.cc
    RobotClassifier.cc
    VisionPreprocessor.cc
    RobotControllerGUI.cc
    RobotLink.cc
    Input/Gamepad.cc
    Input/Input.cc
    UIWidgets/IMUWidget.cc
)

# Create the object files
# cuda_add_library(objects ${SOURCES})
# set_target_properties(objects PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# Create the executable
# cuda_add_executable(RobotController ${SOURCES})
add_executable(RobotController ${SOURCES})

target_link_libraries(RobotController opencv_world Qt6::Widgets Xinput ${CMAKE_SOURCE_DIR}/libs/DS5W/ds5w_x64.lib)


add_custom_command(TARGET RobotController POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/libs/DS5W/ds5w_x64.dll
        $<TARGET_FILE_DIR:RobotController>
)