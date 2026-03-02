#pragma once

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
  cv::Point2f lastCenter{};
  double lastTime = 0.0;
  bool hasLast = false;
  int invalidCount = 0;
};

class OpenCVTracker : public OdometryBase {
 public:
  OpenCVTracker(ICameraReceiver* videoSource);

  void SwitchRobots(void) override;
  void SetPosition(const PositionData& newPos, bool opponentRobot) override;

 private:
  void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;
  void _ProcessSlot(TrackSlot& slot, bool isOpponent, double frameTime);

  TrackSlot _robotSlot;
  TrackSlot _opponentSlot;
  cv::TrackerDaSiamRPN::Params _trackerParams;
  cv::Mat _previousImage;
};
