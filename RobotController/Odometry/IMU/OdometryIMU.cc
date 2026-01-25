#include "OdometryIMU.h"

#include "../../CVRotation.h"
#include "../../RobotConfig.h"
#include "../../RobotController.h"
#include "../../RobotOdometry.h"


OdometryIMU::OdometryIMU() : OdometryBase(nullptr), _lastImuAngle(0) {}

// Start the thread
// Returns false if already running
bool OdometryIMU::Run() {
  if (_running) {
    std::cerr << "WARNING: Attempt to run OdometryIMU after it is already running. Will not start new thread." << std::endl;
    return false;
  }

  // Start the new thread
  processingThread = std::thread([&]() {
    // Mark we are running
    _running.store(true);

    long frameID = -1;

    while (_running.load() && !_stopWhenAble.load()) {
      // Get the new IMU data
      double frameTime = -1.0f;

      IMUData newMessage;
      // Blocking read until new frame available
      long frameIDnew = RobotController::GetInstance().GetIMUFrame(
          newMessage, frameID, &frameTime);

      // If a new frame was obtained do this
      if (frameIDnew >= 0) {
        frameID = frameIDnew;

        // Update the data
        _UpdateData(newMessage, frameTime);
      }
    }

    // Exiting thread
    _running = false;
    _stopWhenAble = false;
  });

  return true;
}

/**
 * Gets the angle to add to the current angle to get the internal imu angle
 */
float OdometryIMU::GetOffset() {
  std::unique_lock<std::mutex> locker(_updateMutex);
  return _lastGlobalOffset;
}

void OdometryIMU::_UpdateData(IMUData& imuData, double timestamp) {
  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  OdometryData globalOdometryData = odometry.Robot(timestamp);
  CVRotation& cvRotation = odometry.GetNeuralRotOdometry();

  OdometryData cvRotData = cvRotation.GetData(false);
  cvRotData.ExtrapolateBoundedTo(timestamp);
  double neuralRotConfidence = cvRotation.GetLastConfidence();

  // Get unique access
  std::unique_lock<std::mutex> locker(_updateMutex);

  _currDataRobot = OdometryData(_currDataRobot.id + 1);
  // reset the last imu angle if we changed radio channels
  if (RADIO_CHANNEL != _lastRadioChannel) {
    _lastImuAngle = imuData.rotation;
  }

  if (globalOdometryData.angle.has_value()) {
    _lastGlobalOffset =
        angle_wrap(imuData.rotation - globalOdometryData.angle.value().angle);
  } else {
    _lastGlobalOffset = 0;
  }

  // increment the robot angle (corrects for any fixed dc offset)
  Angle newAngle = _lastAngle + Angle(imuData.rotation - _lastImuAngle);

  // fuse towards the neural rotation if confidence is high
  if (_currDataRobot.angle.has_value() &&
      neuralRotConfidence > ANGLE_FUSE_CONF_THRESH) {
    AngleData currAngleData = _currDataRobot.angle.value();
    double deltaTime = timestamp - currAngleData.time;
    double interpolateAmount = (std::min)(1.0, deltaTime * ANGLE_FUSE_SPEED);
    newAngle =
        InterpolateAngles(newAngle, currAngleData.angle, interpolateAmount);
  }

  // set the new angle + velocity
  _currDataRobot.angle =
      AngleData(newAngle, imuData.rotationVelocity, timestamp);

  _lastImuAngle = imuData.rotation;
  _lastRadioChannel = RADIO_CHANNEL;
  _lastAngle = _currDataRobot.angle.value().angle;
}

void OdometryIMU::SetAngle(AngleData angleData, bool opponentRobot) {
  // Only do this for our robot
  if (opponentRobot) {
    return;
  }

  std::unique_lock<std::mutex> locker(_updateMutex);
  _lastAngle = angleData.angle;
  _currDataRobot.angle = angleData;
}