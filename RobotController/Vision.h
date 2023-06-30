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
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>

class Vision
{
public:
    Vision(ICameraReceiver &overheadCamL, ICameraReceiver &overheadCamR);
    void Vision::locateRobots(cv::Mat&, cv::Mat&);
    // void performOpticalFlow();
    void runPipeline();

    void convertToBirdsEyeView(cv::Mat& frame, cv::Mat& dst);

    const cv::Mat& GetBirdsEyeImageL();
    const cv::Mat& GetBirdsEyeImageR();

    cv::Point2f GetRobotPosition();
    double GetRobotAngle();
    cv::Point2f GetOpponentPosition();
    double GetOpponentAngle();

    void DrawRobots();
    void DetectRotation(cv::Mat& canny);
    
    double angle;
    cv::Point2f position;
    double opponent_angle;
    cv::Point2f opponent_position;
private:
    ICameraReceiver& overheadCamL;
    ICameraReceiver& overheadCamR;
    cv::Mat currFrameL;
    cv::Mat previousFrameL;
    cv::Mat currFrameR;
    cv::Mat previousFrameR;

    std::vector<RobotTracker> robotTrackers = {};

    OpticalFlow opticalFlow;
    OpticalFlow opponentOpticalFlow;
    bool isInitialized = false;

    void updateRobotTrackers(std::vector<MotionBlob>& centers, cv::Mat& frame);
    void getOpponentRotation();
    void compute3dPointCloud(cv::Mat &leftCam, cv::Mat &rightCam, std::vector<cv::Point3f> &pointCloud,
                             std::vector<cv::Vec3b> &colors);
    cv::Point3f convert2dPointTo3d(int x, int y, short disparity);
    void computeDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disparity, cv::Mat &disparityNormalized);

    cv::Ptr<cv::cuda::StereoSGM> stereoSGMMain;
    cv::Ptr<cv::StereoSGBM> stereoSGBMMainCPU;

    std::thread* gameLoopThread;
    GameLoop* pGameLoop = nullptr;

};
