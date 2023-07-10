/**
 * Used to descern between us and the opponent
 */

#pragma once

#include "RobotTracker.h"

struct RobotCalibrationData
{
    cv::Scalar meanColor;
    cv::Mat histogram;
    double diameter;
};

class RobotClassifier
{
public:
    static RobotClassifier* instance;
    RobotClassifier(RobotTracker& robotTracker, RobotTracker& opponentTracker);

    void Update(std::vector<MotionBlob>& blobs, cv::Mat& frame, cv::Mat& motionImage);

    void SwitchRobots();
    void RequestRecalibrate();
private:
    double ClassifyBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);
    void RecalibrateRobot(RobotCalibrationData& data, MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);
    cv::Scalar GetMeanColorOfBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);

    RobotTracker& opponentTracker;
    RobotTracker& robotTracker;

    RobotCalibrationData robotCalibrationData;
    RobotCalibrationData opponentCalibrationData;

    bool requestingRecalibrate = false;
};

