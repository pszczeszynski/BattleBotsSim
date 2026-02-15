#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <random>
#include <vector>

#include "../../CameraReceiver.h"
#include "../OdometryBase.h"

// Helper structure for rotation results
struct RotationResult {
  float angleDelta;  // Angle delta in radians
  float distance;    // Distance between the two points
};

// Tracked point with age
struct TrackPt {
  cv::Point2f pt;
  // Number of consecutive frames successfully tracked
  int age;
};

// LKFlowTracker Odometry
// Tracks objects using Lucas-Kanade optical flow
class LKFlowTracker : public OdometryBase {
 public:
  LKFlowTracker(ICameraReceiver* videoSource);

  void SwitchRobots(void) override;
  void SetPosition(cv::Point2f newPos, bool opponentRobot) override;
  void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
  void SetAngle(AngleData angleData, bool opponentRobot) override;
  void GetDebugImage(cv::Mat& target,
                     cv::Point offset = cv::Point(0, 0)) override;

 private:
  cv::Rect _GetROI() const;

  void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;
  void _StartCalled(void) override;

  // Core tracking functions
  bool _InitializePoints(cv::Mat& gray);
  bool _UpdateTracking(cv::Mat& prevGray, cv::Mat& currGray, double frameTime);
  void _ComputeRotationsFromPairs(const std::vector<cv::Point2f>& nextPts,
                                  const std::vector<uchar>& status,
                                  double& angleDelta,
                                  std::vector<std::pair<int, int>>& validPairs);
  cv::Point2f _ComputeTranslationFromPoints(
      const std::vector<cv::Point2f>& prevPts,
      const std::vector<cv::Point2f>& nextPts,
      const std::vector<uchar>& status);
  void _UpdateAngleFromRotations(const std::vector<RotationResult>& rotations,
                                 double& angleDelta);
  bool _RespawnPoints(const cv::Mat& gray, cv::Rect roi,
                      std::vector<TrackPt>& tracks, int targetCount);
  void _DrawDebugImage(cv::Size imageSize,
                       const std::vector<cv::Point2f>& nextPts,
                       const std::vector<std::pair<int, int>>& validPairs);
  void _DeduplicateTracks(std::vector<TrackPt>& tracks, float minDist);
  cv::Point2f _ComputeCenterFromPoints(const std::vector<cv::Point2f>& points);
  void _InterpolatePosTowardCenter(const std::vector<TrackPt>& tracks);
  std::vector<std::pair<int, int>> _GeneratePointPairs(int nPts);
  void _FilterPointsByROI(std::vector<TrackPt>& tracks);
  cv::Rect _ClipROIToBounds(cv::Rect roi, cv::Size bounds) const;
  // Last known image size for ROI clipping
  cv::Size _imageSize;
  cv::Mat _prevGray;
  std::vector<TrackPt> _tracks;
  Angle _angle;
  cv::Point2f _pos;

  bool _initialized;
  double _lastRespawnTime;  // Time of last respawn operation

  // Previous data for velocity calculations (local primitives, not
  // OdometryData)
  std::optional<double> _prevTime;

  // Debug image
  std::mutex _mutexDebugImage;
  cv::Mat _debugImage;

  // Random number generator for point pair selection
  std::mt19937 _rng;

  uint64_t _frameID = 0;
};
