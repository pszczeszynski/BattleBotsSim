#pragma once
#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "Vision.h"
#include <QObject>
#include <QPixmap>
#include "RobotTracker.h"
#include "RobotLink.h"
#include "RobotControllerGUI.h"
#include "Gamepad.h"

class RobotController : public QObject
{
    Q_OBJECT
public:
    RobotController();
signals:
    // updates the image of the field in the GUI
    void RefreshFieldImageSignal();
public slots:
    void Run();
private:
    Clock clock;

    DriveCommand RobotLogic(RobotMessage &state);
    DriveCommand OrbitMode(RobotMessage &state);
    DriveCommand ManualMode(RobotMessage &state);
    DriveCommand DriveToPosition(const cv::Point2f& targetPos, bool chooseNewTarget);
    void UpdateRobotTrackers(VisionClassification classification);
    Vision vision;
    Gamepad gamepad;

#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    RobotLinkSim robotLink;
#else
    CameraReceiver overheadCamL_real;
    RobotLinkReal robotLink;
#endif
};
