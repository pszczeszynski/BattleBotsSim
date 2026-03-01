#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include "CameraReceiver.h"
#include "Odometry/OdometryBase.h"


// MachineLearning/rotation_model_hoop.onnx
#define MODEL_PATH "C:/Dev/BattlebotsSim/RobotController/MachineLearning/rotationDetectorV5.onnx"

class CVRotation : public OdometryBase {
 public:
  explicit CVRotation(ICameraReceiver* videoSource);

  // Publish-only contract: CVRotation does NOT store/mutate OdometryData as
  // internal state. OdometryData is constructed only at publish time.
  void _ProcessNewFrame(const cv::Mat frame, double frameTime) override;

  static CVRotation* GetInstance();

  // Computes robot rotation (radians) from image + position prior.
  // Updates internal diagnostics (net vectors, disagreement, lastRotation).
  double ComputeRobotRotation(const cv::Mat& fieldImage, cv::Point2f robotPos,
                              double frameTime);

  double GetLastComputedRotation();
  double GetLastConfidence();

 private:
  static CVRotation* _instance;

  bool _CropImage(const cv::Mat& src, cv::Mat& dst, cv::Rect roi);

  cv::dnn::Net _net;

  // Diagnostic state for confidence computation
  cv::Point2f _netXY1{0, 0};
  cv::Point2f _netXY2{0, 0};
  double _lastDisagreementRad{0.0};
  double _lastRotation{0.0};
  double _lastUpdateTime{0.0};
};
