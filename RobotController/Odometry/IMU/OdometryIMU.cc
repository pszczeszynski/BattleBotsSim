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

  // Grab all needed state in one lock
  double lastImuAngleLocal;
  Angle lastAngleLocal;
  int lastRadioChannelLocal;
  bool needImuResyncLocal;
  {
    std::unique_lock<std::mutex> locker(_updateMutex);
    lastImuAngleLocal = _lastImuAngle;
    lastAngleLocal = _lastAngle;
    lastRadioChannelLocal = _lastRadioChannel;
    needImuResyncLocal = _needImuResync;
  }

  // If we don't have a global angle, we can't compute offset reliably.
  // Publish-only model: do not publish dummy/invalid samples.
  if (!globalOdometryData.angle.has_value()) {
    std::unique_lock<std::mutex> locker(_updateMutex);
    _lastRadioChannel = RADIO_CHANNEL;
    _lastImuAngle = imuData.rotation;  // keep continuity
    // keep _lastAngle as-is
    _lastGlobalOffset = 0;
    _needImuResync = false;  // clear resync flag since we reset IMU angle
    return;
  }

  // Reset continuity if radio channel changed or resync needed
  bool radioChannelChanged = (RADIO_CHANNEL != lastRadioChannelLocal);
  if (radioChannelChanged || needImuResyncLocal) {
    lastImuAngleLocal = imuData.rotation;
  }

  // Compute offset (raw imu - global robot) using the last published/global
  // angle.
  double newOffset =
      angle_wrap(imuData.rotation - globalOdometryData.angle.value().angle);

  // Integrate IMU delta onto last fused angle (stateful)
  Angle integrated =
      lastAngleLocal + Angle(imuData.rotation - lastImuAngleLocal);

  // Fuse towards neural rotation if confident + available
  if (neuralRotConfidence > ANGLE_FUSE_CONF_THRESH &&
      cvRotData.angle.has_value()) {
    AngleData neuralAngle = cvRotData.angle.value();
    double deltaTime = timestamp - neuralAngle.time;
    double interpolateAmount = (std::min)(1.0, deltaTime * ANGLE_FUSE_SPEED);
    integrated =
        InterpolateAngles(integrated, neuralAngle.angle, interpolateAmount);
  }

  // Build publish sample (angle only)
  OdometryData sample;
  sample.Clear();
  sample.angle = AngleData(integrated, imuData.rotationVelocity, timestamp);

  // Publish (this increments id internally)
  OdometryBase::Publish(sample, /*isOpponent=*/false);

  // Commit all state changes in one lock
  {
    std::unique_lock<std::mutex> locker(_updateMutex);
    _lastGlobalOffset = newOffset;
    _lastImuAngle = imuData.rotation;
    _lastRadioChannel = RADIO_CHANNEL;
    _lastAngle = integrated;
    _needImuResync = false;  // clear resync flag after handling
  }
}

void OdometryIMU::SetAngle(AngleData angleData, bool opponentRobot) {
  if (opponentRobot) return;

  {
    std::unique_lock<std::mutex> locker(_updateMutex);
    _lastAngle = angleData.angle;
    _needImuResync = true;  // invalidate continuity so next packet resyncs IMU
  }
}