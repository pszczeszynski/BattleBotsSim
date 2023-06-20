cmake_minimum_required(VERSION 3.2)
project(RobotController VERSION 0.1.0 LANGUAGES CUDA CXX)
set(CMAKE_CUDA_ARCHITECTURES 86) # look up the cuda architecture for your gpu. For my RTX 3080 it's 8.6

# Set policy for find_package to use <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)

find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add library paths
set(GLEW_LIB_PATH ${CMAKE_SOURCE_DIR}/libs/GLEW/lib/Release/x64/)
link_directories(${GLEW_LIB_PATH})
set(SFML_LIB_PATH ${CMAKE_SOURCE_DIR}/libs/SFML/lib/)
link_directories(${SFML_LIB_PATH})
set(SOIL_LIB_PATH ${CMAKE_SOURCE_DIR}/libs/SOIL/lib/windows/)
link_directories(${SOIL_LIB_PATH})

# Add include paths
set(JSON_INCLUDE_PATH "./libs/json/include")
set(GLEW_INCLUDE_PATH "./libs/GLEW/include")
set(GLM_INCLUDE_PATH  "./libs/GLM")
set(SFML_INCLUDE_PATH "./libs/SFML/include")
set(SOIL_INCLUDE_PATH "./libs/SOIL/src/SOIL2")

include_directories( ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${JSON_INCLUDE_PATH} ${GLEW_INCLUDE_PATH} ${GLM_INCLUDE_PATH} ${SFML_INCLUDE_PATH} ${SOIL_INCLUDE_PATH} )

# Define the sources and headers
set(SOURCES 
    RobotController.cc
    MathUtils.cc
    Vision.cc
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
    OpponentProfile.cc
    PathFinder.cc
    Graphics/GameLoop.cc
    Graphics/Camera.cc
    Graphics/Window/Clock.cc
    Graphics/Window/Window.cc
    Graphics/Model/Object.cc
    Graphics/Model/OpenGLModel.cc
)

# Create the object files
# cuda_add_library(objects ${SOURCES})
# set_target_properties(objects PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# Create the executable
cuda_add_executable(RobotController ${SOURCES})

target_link_libraries(RobotController ${CUDA_LIBRARIES} ${SFML_LIBRARIES} opencv_world glew32 opengl32 sfml-system sfml-graphics sfml-window soil2-debug)
