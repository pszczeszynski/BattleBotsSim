#pragma once
#include "ServerSocket.h"
#include "CameraReceiver.h"
#include "RobotStateParser.h"
#include "Vision.h"
#include "Clock.h"
class RobotController
{
public:
    RobotController();
    void Run();

private:
    ServerSocket socket;

    // cameras
    CameraReceiver overheadCam;
    Clock clock;

    Vision vision;

    RobotControllerMessage loop(RobotState &state);
};