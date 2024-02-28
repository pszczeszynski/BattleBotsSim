/**
 * Used to descern between us and the opponent
 */

#pragma once

#include "../../VisionClassification.h"
#include "MotionBlob.h"
#include "../OdometryBase.h"

struct RobotCalibrationData
{
    cv::Scalar meanColor;
    cv::Mat histogram;
    double diameter;
};

class RobotClassifier
{
public:
    RobotClassifier();

    VisionClassification ClassifyBlobs(std::vector<MotionBlob> &blobs, cv::Mat &frame, cv::Mat &motionImage, OdometryData &robotData, OdometryData &opponentData);

private:
    double ClassifyBlob(MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage, OdometryData &robotData, OdometryData &opponentData);
    void RecalibrateRobot(RobotCalibrationData &data, MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage);
    cv::Scalar GetMeanColorOfBlob(MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage);

    RobotCalibrationData robotCalibrationData;
    RobotCalibrationData opponentCalibrationData;
};
