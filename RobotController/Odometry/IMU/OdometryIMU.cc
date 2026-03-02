#include "OdometryIMU.h"

#include "../../CVRotation.h"
#include "../../RobotConfig.h"
#include "../../RobotController.h"
#include "../../RobotOdometry.h"

OdometryIMU::OdometryIMU() : OdometryBase(nullptr), _lastImuAngle(0) {}

// Start the thread
// Returns false if already running
bool OdometryIMU::Run() {
  bool expected = false;
  if (!_running.compare_exchange_strong(expected, true)) {
    std::cerr
        << "WARNING: Attempt to run OdometryIMU after it is already running.\n";
    return false;
  }

  processingThread = std::thread([&]() {
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

void OdometryIMU::_UpdateData(IMUData& imuData, double timestamp) {
  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  CVRotation& cvRotation = odometry.GetNeuralRotOdometry();

  // Reset continuity if radio channel changed or resync needed
  bool radioChannelChanged = (RADIO_CHANNEL != _lastRadioChannel);
  if (radioChannelChanged || _needImuResync) {
    _lastImuAngle = imuData.rotation;
  }

  // Integrate IMU delta onto last fused angle (stateful)
  Angle integrated = _lastAngle + Angle(imuData.rotation - _lastImuAngle);

  // Fuse towards neural rotation if confident + available

  OdometryData cvRotData = cvRotation.GetData(false);
  if (cvRotData.angle.has_value() && cvRotData.angle.value().GetAge() < 0.05) {
    cvRotData.ExtrapolateBoundedTo(timestamp);
    double neuralRotConfidence = cvRotation.GetLastConfidence();

    if (neuralRotConfidence > ANGLE_FUSE_CONF_THRESH &&
        cvRotData.angle.has_value()) {
      AngleData neuralAngle = cvRotData.angle.value();
      double deltaTime = timestamp - neuralAngle.time;
      double interpolateAmount = (std::min)(1.0, deltaTime * ANGLE_FUSE_SPEED);
      integrated =
          InterpolateAngles(integrated, neuralAngle.angle, interpolateAmount);
    }
  }

  // Build publish sample (angle only)
  OdometryData sample{};
  sample.angle = AngleData(integrated, imuData.rotationVelocity, timestamp);

  // Publish (this increments id internally)
  OdometryBase::Publish(sample, /*isOpponent=*/false, OdometryAlg::IMU);

  _lastImuAngle = imuData.rotation;
  _lastRadioChannel = RADIO_CHANNEL;
  _lastAngle = integrated;
  _needImuResync = false;  // clear resync flag after handling
}

void OdometryIMU::SetAngle(AngleData angleData, bool opponentRobot) {
  if (opponentRobot) return;

  if (angleData.algorithm == OdometryAlg::IMU) {
    return;
  }

  {
    std::unique_lock<std::mutex> locker(_angleMutex);
    _lastAngle = angleData.angle;
    _needImuResync = true;  // invalidate continuity so next packet resyncs IMU
  }
}