#pragma once
#include "../OdometryBase.h"
#include "../../RobotController.h"

// OdometryIMU calss
class OdometryIMU : public OdometryBase
{
    OdometryIMU(RobotController &controller);

    bool Run( void ) override; // Starts the thread(s) to decode data. Returns true if succesful

private:
    RobotController& _controller;
    void _UpdateData(IMUData& imuData, double time);

};
