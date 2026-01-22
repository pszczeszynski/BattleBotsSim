/**
 * Used to descern between us and the opponent
 */

#pragma once

#include "../../VisionClassification.h"
#include "../OdometryBase.h"
#include "MotionBlob.h"


struct RobotCalibrationData {
  cv::Scalar meanColor;
  cv::Mat histogram;
  double diameter;
};

class RobotClassifier {
 public:
  RobotClassifier();

  VisionClassification ClassifyBlobs(std::vector<MotionBlob> &blobs,
                                     cv::Mat &frame, cv::Mat &motionImage,
                                     OdometryData &robotData,
                                     OdometryData &opponentData,
                                     bool neuralBlackedOut, double frameTime);

 private:
  double ClassifyBlob(MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage,
                      OdometryData &robotData, OdometryData &opponentData);
  void RecalibrateRobot(RobotCalibrationData &data, MotionBlob &blob,
                        cv::Mat &frame, cv::Mat &motionImage);

  RobotCalibrationData robotCalibrationData;
  RobotCalibrationData opponentCalibrationData;
};
