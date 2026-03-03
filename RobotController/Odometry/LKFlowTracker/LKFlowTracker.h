#pragma once
#include <array>
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

// Per-target state for LK flow (us or opponent). Shared logic operates on this.
struct LKFlowTargetState {
  cv::Point2f pos{0, 0};
  Angle angle{Angle(0)};
  std::vector<TrackPt> tracks;
  std::optional<double> prevTime;
  double lastRespawnTime = 0.0;
  bool forceRespawnNextFrame = false;
  bool initialized = false;
};

// LKFlowTracker Odometry
// Tracks objects using Lucas-Kanade optical flow (both us and opponent)
class LKFlowTracker : public OdometryBase {
 public:
  LKFlowTracker(ICameraReceiver* videoSource);

  void SwitchRobots(void) override;
  void SetPosition(const PositionData& newPos, bool opponentRobot) override;
  void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
  void SetAngle(AngleData angleData, bool opponentRobot) override;
  void GetDebugImage(cv::Mat& target,
                     cv::Point offset = cv::Point(0, 0)) override;

 private:
  cv::Rect _GetROI(cv::Point2f pos, cv::Size imageSize) const;

  void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;
  void _StartCalled(void) override;
  Angle _MoveTowardsExternalPathTangent(Angle startAngle);

  // Core tracking: operates on one target's state. Returns true if update ok.
  bool _UpdateTracking(cv::Mat& prevGray, cv::Mat& currGray, double frameTime,
                       LKFlowTargetState& state, bool isOpponent);

  void _ComputeRotationsFromPairs(
      const std::vector<TrackPt>& tracks,
      const std::vector<cv::Point2f>& nextPts,
      const std::vector<uchar>& status,
      double& angleDelta,
      std::vector<std::pair<int, int>>& validPairs);
  cv::Point2f _ComputeTranslationFromPoints(
      const std::vector<TrackPt>& tracks,
      const std::vector<cv::Point2f>& prevPts,
      const std::vector<cv::Point2f>& nextPts,
      const std::vector<uchar>& status);
  void _UpdateAngleFromRotations(const std::vector<RotationResult>& rotations,
                                 double& angleDelta);
  bool _RespawnPoints(const cv::Mat& gray, cv::Rect roi,
                     std::vector<TrackPt>& tracks, int targetCount,
                     double frameTime, double& lastRespawnTime,
                     cv::Rect excludeRoi = cv::Rect(0, 0, 0, 0));
  void _DrawDebugImage(cv::Size imageSize,
                       const std::vector<cv::Point2f>& nextPts,
                       const std::vector<uchar>& status,
                       const std::vector<std::pair<int, int>>& validPairs,
                       cv::Rect roi, const cv::Scalar& color);
  void _DeduplicateTracks(std::vector<TrackPt>& tracks, float minDist);
  cv::Point2f _ComputeCenterFromPoints(const std::vector<TrackPt>& tracks);
  void _InterpolatePosTowardCenter(const std::vector<TrackPt>& tracks,
                                  cv::Point2f& pos);
  std::vector<std::pair<int, int>> _GeneratePointPairs(int nPts);
  void _FilterPointsByROI(std::vector<TrackPt>& tracks, cv::Rect roi);
  cv::Rect _ClipROIToBounds(cv::Rect roi, cv::Size bounds) const;

  // [0] = us, [1] = opponent
  std::array<LKFlowTargetState, 2> _targets;

  // Last known image size for ROI clipping
  cv::Size _imageSize;
  cv::Mat _prevGray;

  // Debug image
  std::mutex _mutexDebugImage;
  cv::Mat _debugImage;

  // Random number generator for point pair selection
  std::mt19937 _rng;

  uint64_t _frameID = 0;
};
