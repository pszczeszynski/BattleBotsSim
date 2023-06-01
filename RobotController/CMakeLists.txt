cmake_minimum_required(VERSION 3.2)
project(RobotController VERSION 0.1.0 LANGUAGES CUDA CXX)
set(CMAKE_CUDA_ARCHITECTURES 86) # look up the cuda architecture for your gpu. For my RTX 3080 it's 8.6

# Set policy for find_package to use <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)

find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include paths
set(JSON_INCLUDE_PATH "./libs/json/include")


include_directories( ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${JSON_INCLUDE_PATH} )

# Define the sources and headers
set(SOURCES 
    RobotController.cc
    MathUtils.cc
    Vision.cu
    CameraReceiver.cc
    ServerSocket.cc
    RobotStateParser.cc
    Clock.cc
    RobotTracker.cc
    ValueBin.cc
    TrackingUtils.cc
    Timer.cc
    OpticalFlow.cc
    CornerTracker.cc
    Corner.cc
)

# Create the object files
# cuda_add_library(objects ${SOURCES})
# set_target_properties(objects PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# Create the executable
cuda_add_executable(RobotController ${SOURCES})

target_link_libraries(RobotController ${CUDA_LIBRARIES} opencv_world)
