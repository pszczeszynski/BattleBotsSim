cmake_minimum_required(VERSION 3.16)
project(RobotController VERSION 0.1.0 LANGUAGES CXX)

# set C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set policy for find_package to use <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)


# add define for SIMULATION if SIMULATION is an arg
option(SIMULATION "Build for simulation" OFF)
if(SIMULATION)
    add_definitions(-DSIMULATION)
endif()

# add define for VIDEO_FILES
option(VIDEO_FILES "Build for Videos" OFF)
if(VIDEO_FILES)
    add_definitions(-DVIDEO_FILES)
endif()

# add define for FORCE_SIM_DATA
option(FORCE_SIM_DATA "Force simulation data" OFF)
if(FORCE_SIM_DATA)
    # debug print
    message(STATUS "FORCE_SIM_DATA is ON")
    add_definitions(-DFORCE_SIM_DATA)
endif()


find_package( OpenCV REQUIRED COMPONENTS core)
add_definitions(-D_MBCS)

set(JSON_INCLUDE_PATH "./libs/json/include")
set(IMGUI_INCLUDE_PATH "./libs/IMGUI")
set(GLFW_INCLUDE_PATH "./libs/IMGUI/glfw/include")




set(SPINNAKER_INCLUDE_DIR "C:/Program Files/FLIR Systems/Spinnaker/include")
set(SPINNAKER_INCLUDE_DIR2 "C:/Program Files/Teledyne/Spinnaker/include")


include_directories( ${OpenCV_INCLUDE_DIRS} ${JSON_INCLUDE_PATH} ${IMGUI_INCLUDE_PATH} ${GLFW_INCLUDE_PATH} ${SPINNAKER_INCLUDE_DIR} ${SPINNAKER_INCLUDE_DIR2})

set(OS "WINDOWS")
add_definitions(-DOS_WINDOWS)
set(LIBS hid setupapi)

# Define the sources and headers
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




# Create the object files
# cuda_add_library(objects ${SOURCES})
# set_target_properties(objects PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# Create the executable
# cuda_add_executable(RobotController ${SOURCES})
add_executable(RobotController ${SOURCES})


find_package(OpenGL REQUIRED)

include_directories("C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/shared")

# Spinnaker library to link with
set(SPINNAKER_LIBS "C:/Program Files/FLIR Systems/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")
set(SPINNAKER_LIBS2 "C:/Program Files/Teledyne/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")

target_link_libraries(RobotController opencv_world Xinput ${CMAKE_SOURCE_DIR}/libs/DS5W/ds5w_x64.lib 
                      ${CMAKE_SOURCE_DIR}/libs/IMGUI/glfw/lib-vc2010-64/glfw3.lib ${OPENGL_LIBRARIES}
                      hid setupapi
                      ${SPINNAKER_LIBS2})


add_custom_command(TARGET RobotController POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/libs/DS5W/ds5w_x64.dll
        $<TARGET_FILE_DIR:RobotController>
)