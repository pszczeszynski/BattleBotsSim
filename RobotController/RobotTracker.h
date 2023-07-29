#pragma once
#include <opencv2/opencv.hpp>
#include "Clock.h"
#include "OpponentProfile.h"
#include "MathUtils.h"
#include "Globals.h"

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

    static RobotTracker& Robot();
    static RobotTracker& Opponent();

    void UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame, RobotIMUData& imuData);
    void UpdateVisionOnly(MotionBlob& blob, cv::Mat& frame);
    void UpdateIMUOnly(RobotIMUData& imuData);
    void UpdateSetPosAndVel(cv::Point2f position, cv::Point2f velocity);

    void invalidate();

    cv::Point2f getPosition();
    Angle getAngle();

    Angle angle; // TODO: make this private
    cv::Point2f position;
    bool isValid;

    cv::Point2f GetVelocity();

private:
    double lastIMUAngle;
    Angle CalcAnglePathTangent();
    bool visualAngleValid = false;

    cv::Point2f lastPositionWhenUpdatedAngle; // angle updated based on displacement

    cv::Point2f velocity;
    Clock lastUpdate;
};
