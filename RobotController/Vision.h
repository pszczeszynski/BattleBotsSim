#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "MathUtils.h"
#include "RobotStateParser.h"
#include <thread>
#include "Graphics/GameLoop.h"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>
#include "CameraReceiver.h"

class Vision
{
public:
    enum class RobotSide
    {
        Front,
        Back,
        Left,
        Right
    };

    /**
     * Every point cloud side chooses a robot candidate and the best one is used
    */
    struct OpponentCandidate
    {
        cv::Point3f pos;
        double score;
    };

    void runPipeline(cv::Point3f opponentPositionSim, cv::Point3f motionVector);

    void compute3dPointCloud(cv::Mat &left, cv::Mat &right,
                             std::vector<cv::Point3f> &pointCloud, std::vector<cv::Vec3b> &colors, Vision::RobotSide robotSide);
    OpponentCandidate findOpponent(std::vector<cv::Point3f> &pointCloud, std::vector<cv::Vec3b> &colors);
    void computeDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disparity);
    cv::Point3f convert2dPointTo3d(int x, int y, short disparity);

    Vision(CameraReceiver &cameraTL, CameraReceiver &cameraTR, CameraReceiver &cameraBL, CameraReceiver &cameraBR,
           CameraReceiver &cameraLL, CameraReceiver &cameraLR, CameraReceiver &cameraRL, CameraReceiver &cameraRR);

private:
    cv::Ptr<cv::cuda::StereoSGM> stereoSGMMain;
    GameLoop* pGameLoop = nullptr;
    std::thread *gameLoopThread;

    CameraReceiver& cameraTL;
    CameraReceiver& cameraTR;
    CameraReceiver& cameraBL;
    CameraReceiver& cameraBR;
    CameraReceiver& cameraLL;
    CameraReceiver& cameraLR;
    CameraReceiver& cameraRL;
    CameraReceiver& cameraRR;

    cv::Point3f rotatePointToRobotSide(cv::Point3f p, RobotSide robotSide);
};
