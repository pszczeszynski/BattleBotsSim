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
#include "PathFinder.h"
#include "Graphics/GameLoop.h"

class Vision
{
public:
    Vision(CameraReceiver &overheadCam);
    void Vision::locateRobots(cv::Mat&, cv::Mat&);
    // void performOpticalFlow();
    void runPipeline();

    void convertToBirdsEyeView(cv::Mat& frame, cv::Mat& dst);

    const cv::Mat& GetBirdsEyeImage();

    cv::Point2f GetRobotPosition();
    double GetRobotAngle();
    cv::Point2f GetOpponentPosition();
    double GetOpponentAngle();
    
    double angle;
    cv::Point2f position;
    double opponent_angle;
    cv::Point2f opponent_position;
private:
    CameraReceiver& overheadCam;
    cv::Mat currFrame;
    cv::Mat previousFrame;

    std::vector<RobotTracker> robotTrackers = {};

    OpticalFlow opticalFlow;
    OpticalFlow opponentOpticalFlow;
    bool isInitialized = false;

    void updateRobotTrackers(std::vector<MotionBlob>& centers, cv::Mat& frame);
    void getOpponentRotation();


    std::thread* gameLoopThread;
    GameLoop* pGameLoop;

};
