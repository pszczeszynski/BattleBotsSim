#pragma once
#include "../OdometryBase.h"
#include "../../../Common/Communication.h"

class RobotController;

// OdometryIMU calss
class OdometryIMU : public OdometryBase
{
  public:
    OdometryIMU();
    void SetAngle(Angle newAngle, bool opponentRobot, double angleFrameTime,
                  double newAngleVelocity, bool valid) override;

    bool Run() override; // Starts the thread(s) to decode data. Returns true if
                         // succesful

    float GetOffset();

  private:
    void _UpdateData(IMUData &imuData, double timestamp);
    double _lastGlobalOffset = 0; // our raw imu angle - the global robot angle
                                  // at the last received packet time
    double _lastImuAngle = 0;
    Angle _lastAngle;
    int _lastRadioChannel = 0;
};
