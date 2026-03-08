#pragma once

#include <optional>

#include "../../../Common/Communication.h"
#include "../OdometryBase.h"

class OdometryIMU : public OdometryBase {
 public:
  OdometryIMU();
  void SetAngle(AngleData angleData, bool opponentRobot) override;

  bool Run() override { _running.store(true); return true; }

  void PushIMUData(const IMUData &data, double time);

  IMUData GetIMUData();

 private:
  void _UpdateData(const IMUData &imuData, double timestamp);

  // Integration state (all protected by _stateMutex)
  std::mutex _stateMutex;
  Angle _lastAngle;
  std::optional<double> _lastTimestamp;
  double _lastImuAngle = 0;
  int _lastRadioChannel = 0;

  std::mutex _dataMutex;
  IMUData _lastIMUData{};
};
