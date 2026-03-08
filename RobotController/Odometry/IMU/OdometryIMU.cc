#include "OdometryIMU.h"

#include "../../RobotConfig.h"

namespace {
GraphWidget g_imuGraph("IMU rotation", -180.0f, 180.0f, "°");
GraphWidget g_imuPublishInterval("IMU publish interval", 0.0f, 300.0f, "ms");
Clock publishTimer{};
}

OdometryIMU::OdometryIMU() : OdometryBase(nullptr) {}

void OdometryIMU::PushIMUData(const IMUData &data, double time) {
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    _lastIMUData = data;
  }
  _UpdateData(data, time);
}

IMUData OdometryIMU::GetIMUData() {
  std::lock_guard<std::mutex> lock(_dataMutex);
  return _lastIMUData;
}

void OdometryIMU::_UpdateData(const IMUData &imuData, double timestamp) {
  std::lock_guard<std::mutex> lock(_stateMutex);

  if (!_lastTimestamp.has_value()) {
    _lastTimestamp = timestamp;
    _lastImuAngle = imuData.rotation;
    _lastRadioChannel = RADIO_CHANNEL;
    return;
  }

  bool radioChannelChanged = (RADIO_CHANNEL != _lastRadioChannel);

  Angle integrated = _lastAngle;
  if (!radioChannelChanged) {
    if (INTEGRATE_GYRO_VEL) {
      double deltaTime = timestamp - _lastTimestamp.value();
      if (deltaTime > 0.0 && deltaTime < 0.05) {
        integrated = integrated + Angle(imuData.rotationVelocity * deltaTime);
      }
    } else {
      integrated = integrated + Angle(imuData.rotation - _lastImuAngle);
    }
  }

  OdometryData sample{};
  double programTime = Clock::programClock.getElapsedTime();
  sample.angle = AngleData(integrated, imuData.rotationVelocity, programTime);

  OdometryBase::Publish(sample, /*isOpponent=*/false, OdometryAlg::IMU);
  publishTimer.markEnd();
  g_imuPublishInterval.AddData(publishTimer.getElapsedTime() * 1000.0f);
  g_imuGraph.AddData(angle_wrap(imuData.rotation) * 180.0f / M_PI);
  publishTimer.markStart();

  _lastAngle = integrated;
  _lastTimestamp = timestamp;
  _lastImuAngle = imuData.rotation;
  _lastRadioChannel = RADIO_CHANNEL;
}

void OdometryIMU::SetAngle(AngleData angleData, bool opponentRobot) {
  if (opponentRobot) return;
  if (angleData.algorithm == OdometryAlg::IMU) return;

  std::lock_guard<std::mutex> lock(_stateMutex);
  _lastAngle = angleData.angle;
  _lastTimestamp.reset();
}
