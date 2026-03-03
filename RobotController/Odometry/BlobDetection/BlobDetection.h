#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

#include "../../CameraReceiver.h"
#include "../OdometryBase.h"
#include "MotionBlob.h"
#include "RobotClassifier.h"

// Result of blob classification - which blobs are robot vs opponent
struct VisionClassification {
  std::optional<MotionBlob> robot;
  std::optional<MotionBlob> opponent;
};

struct prevFrameData {
  cv::Mat previousImage;
  double previousFrameTime;
};

// Debug info for a blob - valid/invalid and reason
struct BlobDebugInfo {
  cv::Rect rect;
  cv::Point2f center;
  bool valid;
  std::string reason;
};

// BlobDetection Odometry
// Tracks objects based on the blob movement
//
// Publish-only contract: BlobDetection does NOT store or mutate OdometryData as
// internal state. OdometryData is only constructed at publish time (right
// before calling Publish()). The id field increments only when a publish event
// occurs.
class BlobDetection : public OdometryBase {
 public:
  BlobDetection(ICameraReceiver *videoSource);

  void SwitchRobots(void) override;
  void SetPosition(const PositionData& newPos, bool opponentRobot) override;
  void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
  void GetDebugImage(cv::Mat &target, cv::Point offset = cv::Point(0, 0))
      override;  // Returns an image that is used for debugging purposes.

 private:
  // Per-stream tracking state (one for us, one for opponent)
  // All smoothing/invalid counting/angle-tangent state lives here
  struct BlobTrackState {
    // Position and velocity state
    std::optional<cv::Point2f> last_position;
    std::optional<cv::Rect> last_rect;
    cv::Point2f last_velocity = cv::Point2f(0, 0);
    double last_position_time = 0;

    // Blob area tracking
    double last_blob_area = 0;

    // Velocity smoothing state
    double last_vel_time = 0;
  };

  // Run every time a new frame is available
  void _ProcessNewFrame(const cv::Mat currFrame, double frameTime) override;

  VisionClassification _DoBlobDetection(const cv::Mat &currFrame,
                                       const cv::Mat &prevFrame,
                                       double frameTime);

  // Process a single stream (us or opponent)
  void _ProcessStream(const MotionBlob& blob, BlobTrackState &state, bool isOpponent,
                      double timestamp);

  // Calculate smoothed velocity and update state
  // Uses prevPos (from state before update) and newPos to calculate velocity
  void _UpdateSmoothedVelocity(BlobTrackState &state, cv::Point2f newPos,
                               double timestamp,
                               const std::optional<cv::Point2f> &prevPos);

  // Construct and publish OdometryData from current state
  void _PublishFromState(BlobTrackState &state, bool isOpponent,
                         double timestamp);

  RobotClassifier _robotClassifier;  // Takes the blobs and figures out whos who
  // cv::Mat _previousImage;            // The previous image to do delta on
  // double _prevFrameTime = 0;  // The previous time for velocity calcs and when
                              // we updated our previous image
  prevFrameData _prevFrameData[100];
  int currIndex = 0;
  // int prevIndex = 0;
  
  std::mutex _mutexDebugImage;
  cv::Mat _debugImage;

  // Per-stream tracking state
  BlobTrackState _usState;
  BlobTrackState _themState;
};
