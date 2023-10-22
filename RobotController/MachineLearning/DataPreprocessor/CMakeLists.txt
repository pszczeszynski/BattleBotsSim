cmake_minimum_required(VERSION 3.16)
project(DataPreprocessor VERSION 0.1.0 LANGUAGES CXX)

# Set C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set policy for find_package to use <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)

# Find required packages
find_package(OpenCV REQUIRED)

# Set include directories
set(JSON_INCLUDE_PATH "../../libs/json/include")

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS} ${JSON_INCLUDE_PATH})



# Define the sources
set(SOURCES 
    DataPreprocessor.cc
    ../../VisionPreprocessor.cc
    ../../RobotConfig.cc
)

# Create the executable
add_executable(DataPreprocessor ${SOURCES})

# Link the necessary libraries
target_link_libraries(DataPreprocessor ${OpenCV_LIBS})
