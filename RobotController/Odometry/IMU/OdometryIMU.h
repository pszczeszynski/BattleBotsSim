#pragma once
#include "../OdometryBase.h"
#include "../../../Common/Communication.h"

class RobotController;

// OdometryIMU calss
class OdometryIMU : public OdometryBase
{
public:
    OdometryIMU();
    void SetAngle(double newAngle, bool opponentRobot) override;
    void SwitchRobots(void) override{}; // Dont do anything for SwitchRobots

    bool Run(void) override; // Starts the thread(s) to decode data. Returns true if succesful

    float GetOffset();

private:
    void _UpdateData(IMUData &imuData, double timestamp);
    double _lastImuAngle = 0;
    Angle _lastAngle;
    int _lastRadioChannel = 0;
};
