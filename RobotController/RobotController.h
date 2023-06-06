#pragma once
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include "Clock.h"

#define ENABLE_VISION
#ifdef ENABLE_VISION
#include "CameraReceiver.h"
#include "Vision.h"
#endif

class RobotController
{
public:
    RobotController();
    void Run();

private:
    ServerSocket socket;

#ifdef ENABLE_VISION
    CameraReceiver overheadCam;
    Vision vision;
#endif

    Clock clock;


    RobotControllerMessage loop(RobotState &state);
};