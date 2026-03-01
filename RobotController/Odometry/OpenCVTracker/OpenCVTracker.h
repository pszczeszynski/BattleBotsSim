#pragma once

#include <mutex>
#include <optional>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "../OdometryBase.h"

struct TrackSlot {
  cv::Ptr<cv::Tracker> tracker;
  cv::Rect bbox;
  std::optional<cv::Rect> pendingInitBBox;
  bool initialized = false;
  double lastInitTime = 0.0;
  cv::Point2f lastCenter{};
  double lastTime = 0.0;
  bool hasLast = false;
  int invalidCount = 0;
  std::optional<AngleData> angle;
};

class OpenCVTracker : public OdometryBase {
 public:
  OpenCVTracker(ICameraReceiver* videoSource);

  void SwitchRobots(void) override;
  void SetPosition(const PositionData& newPos, bool opponentRobot) override;
  void SetAngle(AngleData angleData, bool opponentRobot) override;
  void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0)) override;

 private:
  void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;
  void _ProcessSlot(TrackSlot& slot, bool isOpponent, double frameTime);

  TrackSlot _robotSlot;
  TrackSlot _opponentSlot;
  cv::Mat _previousImage;

  std::mutex _slotsMutex;
  std::mutex _mutexDebugImage;
  cv::Rect _debugRobotBbox;
  cv::Rect _debugOpponentBbox;
  bool _debugRobotInitialized = false;
  bool _debugOpponentInitialized = false;

  static constexpr bool kDebugOpenCVTracker = false;
};
