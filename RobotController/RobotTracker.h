#pragma once
#include <opencv2/opencv.hpp>
#include "Clock.h"
class RobotTracker
{
public: 
    RobotTracker(cv::Point2f position);
    

    void update(cv::Point2f position, cv::Mat& frame);
    double getCostOfUpdating(cv::Point2f position);

    cv::Point2f getPosition();
    double getAngle();

    double getRotationBetweenMats(cv::Mat& img1, cv::Mat& img2);

private:

    cv::Point2f position;
    cv::Point2f lastPosition;
    cv::Point2f velocity;
    Clock lastUpdate;

    cv::Mat croppedFrameLast;
    double angle;

    bool isValid;
};
