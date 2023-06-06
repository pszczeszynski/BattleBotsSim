#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "MathUtils.h"
#include "RobotStateParser.h"
#include "CameraReceiver.h"
#include "RobotTracker.h"
#include "OpticalFlow.h"

class Vision
{
public:
    Vision(CameraReceiver &overheadCam);
    cv::Point2f Vision::findOpponent(cv::Mat&, cv::Mat&);
    void performOpticalFlow();
    void runPipeline();
private:
    CameraReceiver& overheadCam;
    cv::Mat previousFrame;

    std::vector<RobotTracker> robotTrackers = {};

    OpticalFlow opticalFlow;
    OpticalFlow opponentOpticalFlow;
    bool isInitialized = false;

    void updateRobotTrackers(std::vector<MotionBlob>& centers, cv::Mat& frame);
    void getOpponentRotation();
};
