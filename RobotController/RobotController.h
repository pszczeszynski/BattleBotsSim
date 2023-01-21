#pragma once
#ifdef LINUX_MODE
#include "ServerSocketLinux.h"
#include "CameraReceiverLinux.h"
#else
#include "ServerSocket.h"
#include "CameraReceiver.h"
#endif
#include "RobotStateParser.h"
#include "Vision.h"

class RobotController
{
public:
    RobotController();
    void Run();

private:
    ServerSocket socket;

    CameraReceiver cameraFL;
    CameraReceiver cameraFR;

    Vision vision;

    RobotControllerMessage loop(RobotState &state);
};