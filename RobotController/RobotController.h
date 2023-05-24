#pragma once
#include "ServerSocket.h"
#include "CameraReceiver.h"
#include "RobotStateParser.h"
#include "Vision.h"
#include "Graphics/Window/Clock.h"

class RobotController
{
public:
    RobotController();
    void Run();

private:
    ServerSocket socket;

    // cameras
    CameraReceiver cameraFL;
    CameraReceiver cameraFR;
    CameraReceiver cameraBL;
    CameraReceiver cameraBR;
    CameraReceiver cameraLL;
    CameraReceiver cameraLR;
    CameraReceiver cameraRL;
    CameraReceiver cameraRR;

    Vision vision;
    Clock clock;
    Clock waitToBackupClock;
    bool backingUp = false;

    RobotControllerMessage loop(RobotState &state);
};