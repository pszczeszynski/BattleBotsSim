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

class RobotController
{
public:
    RobotController();
    void Run();
    void DrawGraphic(const cv::Mat& frame);
private:
    ServerSocket socket;

#ifdef ENABLE_VISION
    CameraReceiver overheadCam;
    Vision vision;
#endif

    Clock clock;

    void show_opponent_charges(cv::Mat& drawing_image);


    RobotControllerMessage loop(RobotState &state, cv::Mat& drawingImage);
    RobotControllerMessage driveToPosition(const cv::Point2f currPos, const double currAngle, const cv::Point2f& targetPos, const RobotState& state, cv::Mat& drawingImage, bool chooseNewTarget);
    OpponentProfile p{};
};