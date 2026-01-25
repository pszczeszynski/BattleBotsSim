#include "OdometryBase.h"

#include <opencv2/core.hpp>

// clock widget
#include "../Odometry/Heuristic1/RobotTracker.h"
#include "../UIWidgets/ClockWidget.h"

// ***********************************************
// ************ Odometry Base ********************
// ***********************************************

OdometryBase::OdometryBase(ICameraReceiver *videoSource)
    : _videoSource(videoSource), _data{OdometryData(0), OdometryData(0)} {};

OdometryBase::OdometryBase(void) : _data{OdometryData(0), OdometryData(0)} {};

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

void OdometryBase::SwitchRobots(void) {
  // Switch who's who
  std::unique_lock<std::mutex> locker(_updateMutex);
  std::swap(_data[(int)RobotSlot::Us], _data[(int)RobotSlot::Opponent]);
}

// Set postion recommends newPos to be the center of the robot. The algorithm is
// free to adjust as required (e.g. find closest tracking rectangle and use its
// center)
void OdometryBase::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  int slot = opponentRobot ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  OdometryData &odoData = _data[slot];

  double currTime = Clock::programClock.getElapsedTime();
  odoData.pos = PositionData(newPos, cv::Point2f(0, 0), currTime);
}

// Force position forces the center of robot to be newpos regardlesss of any
// other considerations
void OdometryBase::ForcePosition(cv::Point2f newPos, bool opponentRobot) {
  SetPosition(newPos, opponentRobot);
}

void OdometryBase::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  int slot = opponentRobot ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  OdometryData &odoData = _data[slot];
  if (odoData.pos.has_value()) {
    odoData.pos.value().velocity = newVel;
  }
}

// Sets angle and zeroes out angular velocity
void OdometryBase::SetAngle(AngleData angleData, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  int slot = opponentRobot ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  OdometryData &odoData = _data[slot];
  odoData.angle = angleData;
}

// Centralized publish function
void OdometryBase::Publish(OdometryData sample, bool isOpponent) {
  std::lock_guard<std::mutex> lk(_updateMutex);
  int slot = isOpponent ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  auto& out = _data[slot];
  int oldId = out.id;  // Save the old id before overwriting
  out = std::move(sample);
  out.id = oldId + 1;  // Increment from the old id
}

// Returns an image use for debugging. Empty by default
// The image should be greyscale and of type CV_8UC1
void OdometryBase::GetDebugImage(cv::Mat &target, cv::Point offset) {
  // This should not happen, but in case it does, we will create an empty image
  if (target.empty()) {
    target = cv::Mat(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0));
  }

  target =
      cv::Mat::zeros(target.size(), target.type());  // Clear the target image

  // X-coordinates for left and right columns
  const int leftX = 10 + offset.x;  // Left column for Robot Data

  // Draw robot data (top-left)
  int yLeft = 20 + offset.y;  // Start at top
  printText("Robot Data:", target, yLeft, leftX);
  _data[(int)RobotSlot::Us].GetDebugImage(target, cv::Point(leftX + 10, yLeft + 14));

  yLeft = 20 + offset.y;  // Reset to top

  // Draw opponent data next to it
  printText("Opponent Data:", target, yLeft, leftX + 190);
  _data[(int)RobotSlot::Opponent].GetDebugImage(target,
                                  cv::Point(leftX + 10 + 190, yLeft + 14));
}