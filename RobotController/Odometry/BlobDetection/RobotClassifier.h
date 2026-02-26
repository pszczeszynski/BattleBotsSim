/**
 * Used to descern between us and the opponent
 */

#pragma once

#include "MotionBlob.h"
#include <opencv2/core.hpp>

// Forward declaration - defined in BlobDetection.h
struct VisionClassification;

// Lightweight prior information for blob classification
// Used instead of OdometryData to avoid unnecessary dependencies
struct TrackPrior {
  bool valid;
  cv::Point2f pos;
  cv::Point2f vel;
};

struct RobotCalibrationData {
  cv::Scalar meanColor;
  cv::Mat histogram;
  double diameter;
};

class RobotClassifier {
 public:
  RobotClassifier();

  VisionClassification ClassifyBlobs(std::vector<MotionBlob> &blobs,
                                     const cv::Mat &frame, const cv::Mat &motionImage,
                                     const TrackPrior &usPrior,
                                     const TrackPrior &themPrior, double frameTime);

 private:
  double ClassifyBlob(MotionBlob &blob, const cv::Mat &frame, const cv::Mat &motionImage,
                      const TrackPrior &usPrior, const TrackPrior &themPrior);
  void RecalibrateRobot(RobotCalibrationData &data, MotionBlob &blob,
                        const cv::Mat &frame, const cv::Mat &motionImage);

  RobotCalibrationData robotCalibrationData;
  RobotCalibrationData opponentCalibrationData;
};
