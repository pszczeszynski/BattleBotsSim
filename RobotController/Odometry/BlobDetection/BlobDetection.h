#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "../../CameraReceiver.h"
#include "../../VisionClassification.h"
#include "../OdometryBase.h"
#include "MotionBlob.h"
#include "RobotClassifier.h"

// BlobDetection Odometry
// Tracks objects based on the blob movement
class BlobDetection : public OdometryBase {
 public:
  BlobDetection(ICameraReceiver *videoSource);

  void SwitchRobots(void) override;
  void SetPosition(cv::Point2f newPos, bool opponentRobot) override;
  void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
  void SetAngle(AngleData angleData, bool opponentRobot) override;
  void GetDebugImage(cv::Mat &target, cv::Point offset = cv::Point(0, 0))
      override;  // Returns an image that is used for debugging purposes.

 private:
  // Run every time a new frame is available
  void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;

  VisionClassification DoBlobDetection(cv::Mat &currFrame, cv::Mat &prevFrame,
                                       double frameTime);
  void UpdateData(
      VisionClassification robotData,
      double timestamp);  // Updates the core data so others can poll it

  bool _IsValidBlob(MotionBlob &blobNew, OdometryData &prevData);
  void _GetSmoothedVisualVelocity(OdometryData &currData,
                                  OdometryData &prevData);
  void _SetData(MotionBlob *blob, OdometryData &currData,
                OdometryData &prevData, OdometryData &prevAngleData,
                double timestamp);
  void CalcAnglePathTangent(OdometryData &currData, OdometryData &prevAngleData,
                            double timestamp);

  RobotClassifier _robotClassifier;  // Takes the blobs and figures out whos who
  cv::Mat _previousImage;            // The previous image to do delta on
  double _prevFrameTime = 0;  // The previous time for velocity calcs and when we updated our previous image
  std::mutex _mutexDebugImage;
  cv::Mat _debugImage;
};
