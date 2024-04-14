#pragma once
#include "../OdometryBase.h"
#include "../../../Communication/Communication.h"
// opencv
#include <opencv2/opencv.hpp>

class RobotController;


class RotationCalculator
{
public:
    RotationCalculator();

    double ComputeRotation(cv::Mat currForeground, cv::Point2f robotPos);
    void SetForeground(cv::Mat newForeground, double angle);

private:
    cv::Mat foreground;     // the foreground of this robot
    double foregroundAngle; // the angle the foreground for this robot was saved
};

// BruteForceRotation calss
class BruteForceRotation : public OdometryBase
{
public:
    BruteForceRotation();
    void SetAngle(double newAngle, bool opponentRobot) override;
    void SwitchRobots(void) override{}; // Dont do anything for SwitchRobots

    bool Run(void) override; // Starts the thread(s) to decode data. Returns true if succesful

private:
    RotationCalculator _opponentRotationCalculator;
};
