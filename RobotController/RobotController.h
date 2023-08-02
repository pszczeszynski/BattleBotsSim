#pragma once
#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "Vision.h"
#include <QObject>
#include <QPixmap>
#include "RobotOdometry.h"
#include "RobotLink.h"
#include "RobotControllerGUI.h"
#include "Gamepad.h"
#include "../Communication/Communication.h"

class RobotController : public QObject
{
    Q_OBJECT
public:
    RobotController();
    static RobotController& GetInstance();

    RobotMessage& GetLatestMessage();
signals:
    // updates the image of the field in the GUI
    void RefreshFieldImageSignal();
public slots:
    void Run();
private:
    Clock clock;

    DriveCommand RobotLogic();
    DriveCommand OrbitMode();
    DriveCommand ManualMode();
    DriveCommand DriveToPosition(const cv::Point2f& targetPos, bool chooseNewTarget);
    void UpdateRobotTrackers(VisionClassification classification);
    Vision vision;
    Gamepad gamepad;

    RobotMessage state;

#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    RobotLinkSim robotLink;
#else
    CameraReceiver overheadCamL_real;
    RobotLinkReal robotLink;
#endif
};
