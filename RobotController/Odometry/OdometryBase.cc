#include "OdometryBase.h"

#include <opencv2/core.hpp>

// clock widget
#include "../Odometry/Heuristic1/RobotTracker.h"
#include "../UIWidgets/ClockWidget.h"

// ***********************************************
// ************ Odometry Base ********************
// ***********************************************

OdometryBase::OdometryBase(ICameraReceiver *videoSource)
    : _videoSource(videoSource) {
  _currDataRobot.isUs = true;      // Make sure this is set
  _currDataOpponent.isUs = false;  // Make sure this is set
};

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
bool OdometryBase::NewDataValid(int oldId, bool getOpponent) {
  // Mutex not required since we're just comparing an int
  if (getOpponent) {
    return _currDataOpponent.id > oldId;
  }

  return _currDataRobot.id > oldId;
}

//  Retrieve the actual data
OdometryData OdometryBase::GetData(bool getOpponent) {
  // Get acccess to private data
  std::unique_lock<std::mutex> locker(_updateMutex);

  // Return it (via copy operator)
  return (getOpponent) ? _currDataOpponent : _currDataRobot;
}

void OdometryBase::SwitchRobots(void) {
  // Switch who's who
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData temp_Robot = _currDataRobot;
  _currDataRobot = _currDataOpponent;
  _currDataOpponent = temp_Robot;

  _currDataRobot.isUs = true;
  _currDataOpponent.isUs = false;
}

// Set postion recommends newPos to be the center of the robot. The algorithm is
// free to adjust as required (e.g. find closest tracking rectangle and use its
// center)
void OdometryBase::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

  odoData.robotPosition = newPos;
  odoData.robotPosValid = true;
}

// Force position forces the center of robot to be newpos regardlesss of any
// other considerations
void OdometryBase::ForcePosition(cv::Point2f newPos, bool opponentRobot) {
  SetPosition(newPos, opponentRobot);
}

void OdometryBase::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
  odoData.robotVelocity = newVel;
}

// Sets angle and zeroes out angular velocity
void OdometryBase::SetAngle(Angle newAngle, bool opponentRobot,
                            double angleFrameTime, double newAngleVelocity,
                            bool valid) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
  odoData.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
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
  _currDataRobot.GetDebugImage(target, cv::Point(leftX + 10, yLeft + 14));

  yLeft = 20 + offset.y;  // Reset to top

  // Draw opponent data next to it
  printText("Opponent Data:", target, yLeft, leftX + 190);
  _currDataOpponent.GetDebugImage(target,
                                  cv::Point(leftX + 10 + 190, yLeft + 14));
}