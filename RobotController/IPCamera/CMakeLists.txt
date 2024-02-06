cmake_minimum_required(VERSION 3.16)
project(Acquisition VERSION 1.0.0 LANGUAGES CXX)

# Set C++ 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Spinnaker SDK paths
set(SPINNAKER_INCLUDE_DIR "C:/Program Files/FLIR Systems/Spinnaker/include")
set(SPINNAKER_LIB_DIR "C:/Program Files/FLIR Systems/Spinnaker/lib64/vs2015")

# Find required packages
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${SPINNAKER_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# Link directories
link_directories(${SPINNAKER_LIB_DIR})

# Define the source file
set(SOURCES 
    Acquisition.cpp
    stdafx.cpp
)

# Create the executable
add_executable(Acquisition ${SOURCES})

# Spinnaker library to link with
set(SPINNAKER_LIBS Spinnakerd_v140)

# Link the necessary libraries
target_link_libraries(Acquisition ${OpenCV_LIBS} ${SPINNAKER_LIBS})
