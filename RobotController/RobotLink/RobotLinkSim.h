#pragma once

#include "IRobotLink.h"
#include "../ServerSocket.h"

class RobotLinkSim : public IRobotLink
{
public:
    RobotLinkSim();
    virtual void Drive(DriverStationMessage& command) override;
    virtual std::vector<RobotMessage> _ReceiveImpl() override;

    void ResetSimulation();
private:
    void setup();
    ServerSocket serverSocket;
    RobotMessage _lastMessage;
};
