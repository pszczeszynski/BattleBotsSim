cmake_minimum_required(VERSION 3.16)
project(RobotController VERSION 0.1.0 LANGUAGES CXX)

# Set C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set policy for find_package to use <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)

# Add defines for options
option(SIMULATION "Build for simulation" OFF)
if(SIMULATION)
    add_definitions(-DSIMULATION)
endif()

option(VIDEO_FILES "Build for Videos" OFF)
if(VIDEO_FILES)
    add_definitions(-DVIDEO_FILES)
endif()

option(FORCE_SIM_DATA "Force simulation data" OFF)
if(FORCE_SIM_DATA)
    message(STATUS "FORCE_SIM_DATA is ON")
    add_definitions(-DFORCE_SIM_DATA)
endif()


# Find OpenCV with required components
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio tracking)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
message(STATUS "OpenCV_LIB_PATH: ${OpenCV_LIB_PATH}")

message(STATUS "Checking for: ${OpenCV_LIB_PATH}/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.lib")

# Construct the opencv_world library name based on OpenCV version
# set(OPENCV_WORLD_LIB_NAME "opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE} ")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(OPENCV_WORLD_LIB_NAME "opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d")
    set(OPENCV_WORLD_DLL "C:/opencv/install/x64/vc16/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll")
else()
    set(OPENCV_WORLD_LIB_NAME "opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")
    set(OPENCV_WORLD_DLL "C:/opencv/install/x64/vc16/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll")
endif()


set(OPENCV_WORLD_LIB "${OpenCV_LIB_PATH}/${OPENCV_WORLD_LIB_NAME}.lib")
if(EXISTS "${OPENCV_WORLD_LIB}")
    list(APPEND OpenCV_LIBS "${OPENCV_WORLD_LIB}")
    message(STATUS "Added ${OPENCV_WORLD_LIB} to OpenCV_LIBS")
else()
    message(FATAL_ERROR "${OPENCV_WORLD_LIB} not found")
endif()



# Define include paths
set(JSON_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/libs/json/include")
set(IMGUI_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/libs/IMGUI")
set(GLFW_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/libs/IMGUI/glfw/include")
set(SPINNAKER_INCLUDE_DIR "C:/Program Files/FLIR Systems/Spinnaker/include")
set(SPINNAKER_INCLUDE_DIR2 "C:/Program Files/Teledyne/Spinnaker/include")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${JSON_INCLUDE_PATH}
    ${IMGUI_INCLUDE_PATH}
    ${GLFW_INCLUDE_PATH}
    ${SPINNAKER_INCLUDE_DIR}
    ${SPINNAKER_INCLUDE_DIR2}
    "C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/shared"
)

# Define OS
add_definitions(-D_MBCS -DOS_WINDOWS)

# Define sources
set(SOURCES
    RobotController.cc
    DriverStationLog.cc
    UIWidgets/RobotControllerGUI.cc
    UIWidgets/ConfigWidget.cc
    UIWidgets/FieldWidget.cc
    UIWidgets/TrackingWidget.cc
    UIWidgets/RobotTelemetryWidget.cc
    libs/IMGUI/imgui.cpp
    libs/IMGUI/imgui_draw.cpp
    libs/IMGUI/imgui_widgets.cpp
    libs/IMGUI/imgui_tables.cpp
    libs/IMGUI/imgui_impl_opengl3.cpp
    libs/IMGUI/imgui_impl_glfw.cpp
    libs/IMGUI/imgui_demo.cpp
    RobotConfig.cc
    GuiUtils.cc
    MathUtils.cc
    Input/Gamepad.cc
    Globals.cc
    Odometry/BlobDetection/RobotClassifier.cc
    RobotLink.cc
    RobotOdometry.cc
    RobotStateParser.cc
    Odometry/OdometryBase.cc
    Odometry/BlobDetection/BlobDetection.cc
    Odometry/Human/HumanPosition.cc
    Odometry/BlobDetection/RobotClassifier.cc
    Odometry/Heuristic1/RobotTracker.cc
    Odometry/Heuristic1/HeuristicOdometry.cc
    Odometry/IMU/OdometryIMU.cc
    SelfRighter.cc
    ServerSocket.cc
    ThreadPool.cc
    Clock.cc
    VisionPreprocessor.cc
    CameraReceiver.cc
    UIWidgets/IMUWidget.cc
    UIWidgets/ImageWidget.cc
    UIWidgets/PlaybackWidget.cc
    UIWidgets/ClockWidget.cc
    UIWidgets/GraphWidget.cc
    UIWidgets/VariantsWidget.cc
    UIWidgets/UIUtilities.cc
    UIWidgets/ManualControlWidget.cc
    UIWidgets/ODataWidget.cc
    UIWidgets/CameraWidget.cc
    CVRotation.cc
    Odometry/Neural/CVPosition.cc
    Strategies/Kill.cc
    Strategies/AStarAttack.cc
    Strategies/Field.cc
    Strategies/ScoreRegion.cc
    Strategies/AStar.cc
    Strategies/Orbit.cc
    Strategies/RobotMovement.cc
    Strategies/DriveToAngleSimulation.cc
    Strategies/Line.cc
    Strategies/Node.cc
    Strategies/FilteredRobot.cc
    Weapons.cc
    hid/hid.cc
    Input/InputState.cc
    PurePursuit.cc
    Strategies/Extrapolate.cc
    SafeDrawing.cc
)

# Check for opencv_tracking module
find_package(OpenCV COMPONENTS tracking QUIET)
if(OpenCV_tracking_FOUND)
    message(STATUS "OpenCV tracking module found. Including OpenCVTracker.cc")
    list(APPEND SOURCES Odometry/OpenCVTracker/OpenCVTracker.cc)
    add_definitions(-D_OPENCV_TRACKING)
else()
    message(WARNING "OpenCV tracking module not found. Excluding OpenCVTracker.cc")
endif()

# Find OpenGL
find_package(OpenGL REQUIRED)

# Spinnaker libraries
set(SPINNAKER_LIBS "C:/Program Files/FLIR Systems/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")
set(SPINNAKER_LIBS2 "C:/Program Files/Teledyne/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")

# Create executable
add_executable(RobotController ${SOURCES})

# Link libraries
target_link_libraries(RobotController PRIVATE
    ${OpenCV_LIBS}
    Xinput
    "${CMAKE_SOURCE_DIR}/libs/DS5W/ds5w_x64.lib"
    "${CMAKE_SOURCE_DIR}/libs/IMGUI/glfw/lib-vc2010-64/glfw3.lib"
    ${OPENGL_LIBRARIES}
    hid
    setupapi
    # ${SPINNAKER_LIBS}
    ${SPINNAKER_LIBS2}
)

# Copy DLLs post-build
# Copy DLLs post-build
add_custom_command(TARGET RobotController POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/libs/DS5W/ds5w_x64.dll"
        $<TARGET_FILE_DIR:RobotController>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENCV_WORLD_DLL}"
        $<TARGET_FILE_DIR:RobotController>

  # This is needed for makig videos in certain formats. For now just not using those formats.
  # Reason is because openCV needs to be compiled properly to support this DLL and most of us don't have it.
  #  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  #      "${CMAKE_SOURCE_DIR}/libs/OpenH264/openh264-2.3.1-win64.dll"
  #      $<TARGET_FILE_DIR:RobotController>
)