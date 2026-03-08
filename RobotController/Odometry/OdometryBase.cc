#include "OdometryBase.h"

#include <cmath>
#include <opencv2/core.hpp>

#include "../Globals.h"
#include "../UIWidgets/GraphWidget.h"
#include "OdometryData.h"

namespace {
GraphWidget g_blobVelocityUsGraph("Blob Velocity Us", 0.0f, 300.0f, "px/s");
GraphWidget g_blobVelocityOpponentGraph("Blob Velocity Opponent", 0.0f, 300.0f,
                                        "px/s");
GraphWidget g_lkFlowVelocityUsGraph("LKFlow Velocity Us", 0.0f, 300.0f, "px/s");
GraphWidget g_lkFlowVelocityOpponentGraph("LKFlow Velocity Opponent", 0.0f,
                                          300.0f, "px/s");
}  // namespace

OdometryBase::OdometryBase(ICameraReceiver *videoSource)
    : _videoSource(videoSource) {
  _data[0] = OdometryData{};
  _data[1] = OdometryData{};
};

OdometryBase::OdometryBase(void) {};

bool OdometryBase::IsRunning(void) { return _running.load(); }

// Start the thread
// Returns false if already running
bool OdometryBase::Run(void) {
  if (_running.load()) {
    // Already running, you need to stop existing thread first
    return false;
  }

  if (_videoSource == nullptr) {
    // No video source, cannot run
    return false;
  }

  // Start the new thread
  processingThread = std::thread([&]() {
    // Mark we are running
    _running.store(true);

    // Initialize anything that needs initilialization
    _StartCalled();

    frameID = -1;

    while (_running.load() && !_stopWhenAble.load()) {
      // Get the new frame video frame
      // Iats going to be pre-processed already (e.g. birdseyeview) and
      // black-and-white
      cv::Mat currFrame;
      double frameTime = -1.0f;
      frameID = _videoSource->GetFrame(currFrame, frameID, &frameTime,
                                       false);  // Blocking read with timeout

      // Process the new frame
      if ((frameID > -1) && !currFrame.empty()) {
        _ProcessNewFrame(currFrame, frameTime);
      }
    }

    // Call any stop routines
    _StopCalled();

    // Exiting thread
    _running.store(false);
    _stopWhenAble.store(false);
  });

  return true;
}

// Stop the main thread
bool OdometryBase::Stop(void) {
  // If not running, return true
  if (!_running.load()) {
    return true;
  }

  // Try to stop the thread
  _stopWhenAble.store(true);

  Clock clockWaiting;

  // Wait with a timeout to have it end
  while (_stopWhenAble.load() &&
         (clockWaiting.getElapsedTime() < ODOMETRY_STOP_TIMEOUT)) {
    // Sleep for 1ms
    Sleep(1);
  }

  // See if it ended. The thread may have died thus also check if its joinable.
  if (!_stopWhenAble.load() || processingThread.joinable()) {
    processingThread.join();

    // Reset these in case the thread died and didnt reset it
    _running.store(false);
    _stopWhenAble.store(false);

    return true;
  }

  // It didnt end!
  return false;
}

// Check if new data is available
bool OdometryBase::HasNewerDataById(int oldId, bool getOpponent) {
  // Mutex not required since we're just comparing an int
  int slot = getOpponent ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  return _data[slot].id > oldId;
}

//  Retrieve the actual data
OdometryData OdometryBase::GetData(bool getOpponent) {
  // Get acccess to private data
  std::unique_lock<std::mutex> locker(_updateMutex);

  // Return it (via copy operator)
  int slot = getOpponent ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  return _data[slot];
}

// First-order low-pass velocity filter. Returns filtered velocity, updates state.
static cv::Point2f ApplyVelocityFilter(cv::Point2f raw, double t, int slot,
                                        double tau,
                                        std::array<cv::Point2f, 2>& lastVel,
                                        std::array<double, 2>& lastTime) {
  const double dt = lastTime[slot] >= 0 ? t - lastTime[slot] : 0;
  cv::Point2f out = raw;
  if (dt > 0 && tau > 0) {
    const double alpha = 1.0 - std::exp(-dt / tau);
    out.x = static_cast<float>(alpha * raw.x +
                               (1.0 - alpha) * lastVel[slot].x);
    out.y = static_cast<float>(alpha * raw.y +
                               (1.0 - alpha) * lastVel[slot].y);
  }
  lastVel[slot] = out;
  lastTime[slot] = t;
  return out;
}

// Centralized publish function
void OdometryBase::Publish(OdometryData sample, bool isOpponent,
                           OdometryAlg alg,
                           std::optional<double> velocityFilterTimeConstant) {
  if (sample.pos.has_value()) sample.pos.value().algorithm = alg;
  if (sample.angle.has_value()) sample.angle.value().algorithm = alg;

  int slot = isOpponent ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  cv::Point2f velocityToStore =
      sample.pos.has_value() ? sample.pos.value().velocity : cv::Point2f(0, 0);

  if (velocityFilterTimeConstant.has_value() && sample.pos.has_value()) {
    velocityToStore = ApplyVelocityFilter(
        sample.pos.value().velocity, sample.pos.value().time, slot,
        velocityFilterTimeConstant.value(), _lastFilteredVelocity,
        _lastFilterTime);
    sample.pos.value().velocity = velocityToStore;
  }

  std::lock_guard<std::mutex> lk(_updateMutex);
  auto &out = _data[slot];
  int oldId = out.id;
  out = std::move(sample);
  out.id = oldId + 1;

  // GraphWidgets show filtered velocity magnitude (px/s)
  const float velMag = static_cast<float>(cv::norm(velocityToStore));
  if (alg == OdometryAlg::Blob) {
    (isOpponent ? g_blobVelocityOpponentGraph : g_blobVelocityUsGraph)
        .AddData(velMag);
  } else if (alg == OdometryAlg::LKFlow) {
    (isOpponent ? g_lkFlowVelocityOpponentGraph : g_lkFlowVelocityUsGraph)
        .AddData(velMag);
  }
}

// Returns an image use for debugging. Empty by default.
// Target can be CV_8UC3 (color) or CV_8UC1; drawing uses BGR when target is
// color.
void OdometryBase::GetDebugImage(cv::Mat &target, cv::Point offset) {
  if (target.empty()) {
    target = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
  }

  target =
      cv::Mat::zeros(target.size(), target.type());  // Clear the target image

  // X-coordinates for left and right columns
  const int leftX = 10 + offset.x;  // Left column for Robot Data

  // Draw robot data (top-left)
  int yLeft = 20 + offset.y;
  cv::putText(target, "Robot Data:", cv::Point(leftX, yLeft),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  _data[(int)RobotSlot::Us].GetDebugImage(target,
                                          cv::Point(leftX + 10, yLeft + 14));

  yLeft = 20 + offset.y;
  // Draw opponent data next to it
  cv::putText(target, "Opponent Data:", cv::Point(leftX + 190, yLeft),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  _data[(int)RobotSlot::Opponent].GetDebugImage(
      target, cv::Point(leftX + 10 + 190, yLeft + 14));
}