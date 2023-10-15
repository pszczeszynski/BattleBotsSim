/**
 * Used to descern between us and the opponent
 */

#pragma once

#include "RobotOdometry.h"
#include "VisionClassification.h"

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
    RobotClassifier();

    VisionClassification ClassifyBlobs(std::vector<MotionBlob>& blobs, cv::Mat& frame, cv::Mat& motionImage);

    void SwitchRobots();
private:
    double ClassifyBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);
    void RecalibrateRobot(RobotCalibrationData& data, MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);
    cv::Scalar GetMeanColorOfBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);

    RobotCalibrationData robotCalibrationData;
    RobotCalibrationData opponentCalibrationData;
};

