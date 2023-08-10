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

class RobotOdometry
{
public: 
    RobotOdometry(cv::Point2f position);

    static RobotOdometry& Robot();
    static RobotOdometry& Opponent();

    void UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame);
    double UpdateForceSetAngle(double newAngle);
    void UpdateIMUOnly();
    void UpdateVisionOnly(MotionBlob& blob, cv::Mat& frame);

    // used for calibration
    void UpdateForceSetPosAndVel(cv::Point2f position, cv::Point2f velocity);

    void Invalidate();

    cv::Point2f GetPosition();
    Angle GetAngle();


    cv::Point2f GetVelocity();
    double GetAngleVelocity();

    void InvertAngle();

private:
    void PostUpdate(cv::Point2f position, cv::Point2f velocity, Angle angle);

    double UpdateAndGetIMUAngle();
    cv::Point2f GetSmoothedVisualVelocity(MotionBlob& blob);
    double _lastIMUAngle;

    Angle _angle;
    cv::Point2f _position;
    bool _isValid;

    Angle CalcAnglePathTangent();
    bool _visualAngleValid = false;
    Clock _lastVisualAngleValidClock;

    cv::Point2f _lastPositionWhenUpdatedAngle; // angle updated based on displacement

    cv::Point2f _lastVelocity;
    double _angleVelocity;

    Clock _lastUpdateClock;
    Clock _lastVelocityCalcClock;


    #define VISUAL_VELOCITY_HISTORY_SIZE 10
    std::deque<cv::Point2f> _visualVelocities;
};
