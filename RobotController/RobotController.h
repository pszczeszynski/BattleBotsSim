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

#define SIMULATION

class RobotController
{
public:
    RobotController();
    void Run();
private:
    ServerSocket socket;


    Clock clock;

    void show_opponent_charges(cv::Mat& drawing_image);

    RobotControllerMessage loop(RobotState &state, cv::Mat& drawingImage);
    RobotControllerMessage driveToPosition(const cv::Point2f currPos, const double currAngle, const cv::Point2f& targetPos, const RobotState& state, cv::Mat& drawingImage, bool chooseNewTarget);
    OpponentProfile p{};

////////// VISION /////////
#ifdef ENABLE_VISION
    Vision vision;

////////// SIMULATION ////////
#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    // CameraReceiverSim overheadCamR_sim;
#else
////////// REAL ////////
    CameraReceiver overheadCamL_real;
    CameraReceiver overheadCamR_real;
#endif
#endif
/////////////////////////

};