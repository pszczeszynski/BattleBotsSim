#pragma once
#include "../../../Common/Communication.h"
#include "../OdometryBase.h"


class RobotController;

// OdometryIMU calss
class OdometryIMU : public OdometryBase {
 public:
  OdometryIMU();
  void SetAngle(AngleData angleData, bool opponentRobot) override;

  bool Run() override;  // Starts the thread(s) to decode data. Returns true if
                        // succesful

 private:
  void _UpdateData(IMUData &imuData, double timestamp);
  double _lastImuAngle = 0;
  Angle _lastAngle;
  int _lastRadioChannel = 0;
  bool _needImuResync = false;  // flag to reset IMU continuity on next packet

  std::mutex _angleMutex;
};
