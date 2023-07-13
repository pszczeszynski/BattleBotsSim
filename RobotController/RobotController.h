#pragma once
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include "Clock.h"
#include "OpponentProfile.h"

#define ENABLE_VISION
#ifdef ENABLE_VISION
#include "CameraReceiver.h"
#include "Vision.h"
#endif
#include <QObject>
#include <QPixmap>
#include "RobotTracker.h"
#include "RobotLink.h"

class RobotController : public QObject
{
    Q_OBJECT
public:
    RobotController();
public slots:
    void Run();
private:
    ServerSocket socket;

    Clock clock;

    void show_opponent_charges(cv::Mat& drawing_image);

    DriveCommand loop(RobotMessage &state);
    DriveCommand driveToPosition(const cv::Point2f currPos, const double currAngle, const cv::Point2f& targetPos, bool chooseNewTarget);
    OpponentProfile p{};


    RobotTracker robotTracker;
    RobotTracker opponentTracker;

////////// VISION /////////
#ifdef ENABLE_VISION
    Vision vision;

////////// SIMULATION ////////
#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    RobotLinkSim robotLink;
    // CameraReceiverSim overheadCamR_sim;
#else
////////// REAL ////////
    CameraReceiver overheadCamL_real;
    RobotLinkReal robotLink;
#endif
#endif
/////////////////////////

};