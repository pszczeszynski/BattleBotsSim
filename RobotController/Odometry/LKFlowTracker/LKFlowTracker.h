#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <random>
#include <vector>

#include "../../CameraReceiver.h"
#include "../../Clock.h"
#include "../OdometryBase.h"


// Helper structure for rotation results
struct RotationResult {
  float angleDelta;  // Angle delta in radians
  float distance;    // Distance between the two points
};

// Tracked point with age
struct TrackPt {
  cv::Point2f pt;
  int age;  // Number of consecutive frames successfully tracked
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

  // Set the ROI for point spawning (x, y, width, height)
  void SetROI(cv::Rect roi);

 private:
  void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;
  void _StartCalled(void) override;

  // Core tracking functions
  bool _InitializePoints(cv::Mat& gray, cv::Rect roi);
  bool _UpdateTracking(cv::Mat& prevGray, cv::Mat& currGray, double frameTime);
  void _ComputeRotationsFromPairs(const std::vector<cv::Point2f>& nextPts,
                                  const std::vector<uchar>& status,
                                  double& angleDelta,
                                  std::vector<std::pair<int, int>>& validPairs);
  void _UpdateAngleFromRotations(const std::vector<RotationResult>& rotations,
                                 double& angleDelta);
  bool _RespawnPoints(const cv::Mat& gray, cv::Rect roi,
                      std::vector<TrackPt>& tracks, int targetCount);

  // Helper functions
  cv::Point2f _ComputeCenterFromPoints(const std::vector<cv::Point2f>& points);
  std::vector<std::pair<int, int>> _GeneratePointPairs(int nPts);
  void _FilterPointsByROI(std::vector<TrackPt>& tracks);
  cv::Rect _ClipROIToBounds(cv::Rect roi, cv::Size bounds);

  // Configuration constants
  static constexpr int LK_MAX_CORNERS = 200;
  static constexpr double LK_QUALITY_LEVEL = 0.001;
  static constexpr double LK_MIN_DISTANCE = 7.0;
  static const cv::Size LK_WIN_SIZE;
  static constexpr int LK_MAX_LEVEL = 3;
  static constexpr int LK_NUM_PAIRS = 200;
  static constexpr int LK_MIN_TRACK_FRAMES = 2;
  static constexpr double RESPAWN_INTERVAL = 0.3;  // seconds - respawn interval

  // Internal state
  cv::Rect _roi;
  cv::Size _imageSize;  // Last known image size for ROI clipping
  cv::Mat _prevGray;
  std::vector<TrackPt> _tracks;  // Replaces _prevPts + _pointTrackCounts
  cv::Point2f _prevCenter;
  Angle _angle;
  int _targetPointCount;  // Target/baseline number of points to maintain
  bool _initialized;
  double _lastRespawnTime;  // Time of last respawn operation

  // Previous data for velocity calculations (local primitives, not OdometryData)
  std::optional<cv::Point2f> _prevPosition;
  std::optional<double> _prevTime;

  // Debug image
  std::mutex _mutexDebugImage;
  cv::Mat _debugImage;

  // Random number generator for point pair selection
  std::mt19937 _rng;
};
