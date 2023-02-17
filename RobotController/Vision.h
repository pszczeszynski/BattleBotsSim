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

class Vision
{
public:
    void compute3dPointCloud(const cv::Mat &left, const cv::Mat &right, std::vector<Point> &pointCloud);
    void computeDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disparity);
    Point convert2dPointTo3d(int x, int y, short disparity);
    void visualizePointCloud(const std::vector<Point>& pointCloud);

    Vision();
    cv::Ptr<cv::StereoSGBM> sgbm;

    GameLoop* pGameLoop = nullptr;
    std::thread *pointCloudThread;
};
