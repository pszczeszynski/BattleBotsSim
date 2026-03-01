#include "OpenCVTracker.h"

#include <iostream>
#include <utility>

#include "../../Globals.h"
#include "opencv2/tracking.hpp"
#include "opencv2/video/tracking.hpp"

namespace {
constexpr double kReinitPeriod = 0.5;
}  // namespace

constexpr double MAX_VELOCITY_TIME_GAP = 0.5;
constexpr int INVALID_RECOVERY_THRESHOLD = 1;
constexpr int ROI_SIZE = 50;

static cv::Rect ClampBBoxToImage(cv::Point2f center) {
  cv::Rect r;
  r.width = ROI_SIZE;
  r.height = ROI_SIZE;
  r.x = static_cast<int>(center.x - r.width / 2.0f);
  r.y = static_cast<int>(center.y - r.height / 2.0f);
  if (r.x < 0) r.x = 0;
  if (r.y < 0) r.y = 0;
  if (r.x + r.width > WIDTH) r.width = WIDTH - r.x;
  if (r.y + r.height > HEIGHT) r.height = HEIGHT - r.y;
  return r;
}

OpenCVTracker::OpenCVTracker(ICameraReceiver* videoSource)
    : OdometryBase(videoSource) {}

void OpenCVTracker::SwitchRobots(void) {
  std::swap(_robotSlot, _opponentSlot);
  // Prevent velocity spikes after swap: neither slot has valid last position
  _robotSlot.hasLast = false;
  _opponentSlot.hasLast = false;
}

void OpenCVTracker::SetPosition(const PositionData& newPos,
                                bool opponentRobot) {
  if (newPos.algorithm == OdometryAlg::OpenCV) {
    return;
  }

  TrackSlot& slot = opponentRobot ? _opponentSlot : _robotSlot;
  slot.pendingInitBBox = ClampBBoxToImage(newPos.position);
  slot.hasLast = false;
  slot.lastTime = 0.0;
  slot.invalidCount = 0;
}

void OpenCVTracker::SetAngle(AngleData angleData, bool opponentRobot) {
}

void OpenCVTracker::_ProcessSlot(TrackSlot& slot, bool isOpponent,
                                 double frameTime) {
  // Pending init: (re)create tracker, init with bbox, no publish
  if (slot.pendingInitBBox.has_value()) {
    cv::Rect bboxLocal = slot.pendingInitBBox.value();
    slot.pendingInitBBox.reset();
    slot.tracker = cv::TrackerCSRT::create();
    slot.tracker->init(_previousImage, bboxLocal);
    slot.bbox = bboxLocal;
    slot.initialized = true;
    slot.hasLast = false;
    slot.lastTime = 0.0;
    slot.invalidCount = 0;
    return;
  }

  if (!slot.initialized) {
    return;
  }

  cv::Rect bboxLocal = slot.bbox;
  bool valid = slot.tracker->update(_previousImage, bboxLocal);

  if (!valid) {
    slot.invalidCount++;
    if (slot.invalidCount >= INVALID_RECOVERY_THRESHOLD) {
      slot.initialized = false;
      slot.hasLast = false;
      slot.lastTime = 0.0;
      slot.invalidCount = 0;
    }

    return;
  }

  slot.invalidCount = 0;
  cv::Point2f center{bboxLocal.x + bboxLocal.width / 2.0f,
                     bboxLocal.y + bboxLocal.height / 2.0f};

  cv::Point2f velocity{0, 0};
  if (slot.hasLast) {
    double deltaTime = frameTime - slot.lastTime;
    if (deltaTime > 0 && deltaTime <= MAX_VELOCITY_TIME_GAP) {
      velocity = (center - slot.lastCenter) / static_cast<float>(deltaTime);
    }
  }

  slot.bbox = bboxLocal;
  slot.lastCenter = center;
  slot.lastTime = frameTime;
  slot.hasLast = true;

  OdometryData sample{};
  sample.pos = PositionData{center, velocity, bboxLocal, frameTime};
  OdometryBase::Publish(sample, isOpponent, OdometryAlg::OpenCV);
}

void OpenCVTracker::_ProcessNewFrame(cv::Mat currFrame, double frameTime) {
  if (currFrame.channels() == 1) {
    cv::cvtColor(currFrame, _previousImage, cv::COLOR_GRAY2RGB);
  } else if (currFrame.channels() == 3) {
    currFrame.copyTo(_previousImage);
  } else if (currFrame.channels() == 4) {
    cv::cvtColor(currFrame, _previousImage, cv::COLOR_RGBA2RGB);
  } else {
    std::cerr << "OpenCVTracker: unsupported frame channels: " << currFrame.channels() << std::endl;
    return;
  }

  _ProcessSlot(_robotSlot, false, frameTime);
  _ProcessSlot(_opponentSlot, true, frameTime);

  std::lock_guard<std::mutex> lock(_mutexDebugImage);
  _debugRobotBbox = _robotSlot.bbox;
  _debugOpponentBbox = _opponentSlot.bbox;
  _debugRobotInitialized = _robotSlot.initialized;
  _debugOpponentInitialized = _opponentSlot.initialized;
}

void OpenCVTracker::GetDebugImage(cv::Mat& target, cv::Point offset) {
  OdometryBase::GetDebugImage(target, offset);

  cv::Rect robotBbox, opponentBbox;
  bool robotInit, opponentInit;
  {
    std::lock_guard<std::mutex> lock(_mutexDebugImage);
    robotBbox = _debugRobotBbox;
    opponentBbox = _debugOpponentBbox;
    robotInit = _debugRobotInitialized;
    opponentInit = _debugOpponentInitialized;
  }

  const cv::Scalar kRobotColor(0, 255, 0);    // Green (BGR)
  const cv::Scalar kOpponentColor(0, 0, 255);  // Red (BGR)
  if (robotInit && robotBbox.area() > 0) cv::rectangle(target, robotBbox, kRobotColor, 2);
  if (opponentInit && opponentBbox.area() > 0) cv::rectangle(target, opponentBbox, kOpponentColor, 2);
}
