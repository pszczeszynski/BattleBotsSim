#pragma once
#include <opencv2/opencv.hpp>
#include "Clock.h"
#include "OpponentProfile.h"

/**
 * @brief A motion blob is a connected component in the image
 * It is only exists in one frame and has no history.
*/
struct MotionBlob
{
public:
    cv::Rect rect;
    cv::Point2f center; // weighted center (!= rect.center)
    cv::Mat* frame;
};

class RobotTracker
{
public: 
    RobotTracker(cv::Point2f position);

    void update(MotionBlob& blob, cv::Mat& frame);
    double getCostOfUpdating(MotionBlob& blob);
    void invalidate();

    cv::Point2f getPosition();
    double getAngle();

    double getRotationBetweenMats(cv::Mat& img1, cv::Mat& img2, cv::Point2f center);

    cv::Point2f getExtrapolatedPos();

    cv::Point2f getVelocity();

    double angle; // TODO: make this private
    cv::Point2f position;

private:

    cv::Point2f lastPosition;
    cv::Point2f velocity;
    Clock lastUpdate;
    MotionBlob lastBlob;

    cv::Mat croppedFrameLast;

    bool isValid;
};
