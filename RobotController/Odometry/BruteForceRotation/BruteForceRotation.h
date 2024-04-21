#pragma once
#include "../OdometryBase.h"
#include "../../../Communication/Communication.h"
#include "../../CameraReceiver.h"
// opencv
#include "../../Mathutils.h"
#include <opencv2/opencv.hpp>

class RobotController;


class RotationCalculator
{
public:
    RotationCalculator();

    double ComputeRotation(cv::Mat currForeground, cv::Point2f robotPos);
    void SetForeground(cv::Mat newForeground, double angle);

private:
    cv::Mat _foreground;     // the foreground of this robot
    double _foregroundAngle; // the angle the foreground for this robot was saved
};

// BruteForceRotation calss
class BruteForceRotation : public OdometryBase
{
public:
    BruteForceRotation(ICameraReceiver *videoSource);
    void SetAngle(double newAngle, bool opponentRobot) override;
    void SetPosition(cv::Point2f newPos, bool opponenetRobot) override;
    // void SwitchRobots(void) override{}; // Dont do anything for SwitchRobots

    // bool Run(void) override; // Starts the thread(s) to decode data. Returns true if succesful

private:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override; // Run every time a new frame is available

    RotationCalculator _opponentRotationCalculator;

    cv::Point2f _opponentPos;

    std::mutex _updateMutex;
};
