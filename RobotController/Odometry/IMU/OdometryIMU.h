#pragma once
#include "../OdometryBase.h"
#include "../../../Communication/Communication.h"

class RobotController;

// OdometryIMU calss
class OdometryIMU : public OdometryBase
{
public:
    OdometryIMU();
    void SetAngle(double newAngle, bool opponentRobot) override;
    void SwitchRobots(void) override {}; // Dont do anything for SwitchRobots

    bool Run( void ) override; // Starts the thread(s) to decode data. Returns true if succesful

private:
    void _UpdateData(IMUData& imuData, double timestamp);
    float _lastImuAngle = 0;
    int _lastRadioChannel = 0;
    float _angleOffset = 0;

};
