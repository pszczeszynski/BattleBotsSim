#pragma once

#include "RobotStateParser.h"
#include "windows.h"
#include "ServerSocket.h"
#include "../Communication/Communication.h"
#include "../Communication/GenericReceiver.h"
#include <fstream>
#include <functional>
#include "Clock.h"

// interface
class IRobotLink
{
public:
    virtual void Drive(DriveCommand& command) = 0; // sends data to robot
    virtual RobotMessage Receive() = 0; // gets data from robot
private:
};

/**
 * This drives the simulated robot
*/
class RobotLinkSim : public IRobotLink
{
public:
    RobotLinkSim();
    virtual void Drive(DriveCommand& command) override;
    virtual RobotMessage Receive() override;

private:
    // socket stuff
    void setup();
    ServerSocket serverSocket;
};

#define TRANSMITTER_COM_PORT TEXT("COM7")

class RobotLinkReal : public IRobotLink
{
public:
    RobotLinkReal();
    virtual void Drive(DriveCommand &command) override;
    virtual RobotMessage Receive() override;
    ~RobotLinkReal();

private:
    HANDLE comPort;
    DCB dcbSerialParams;
    Clock sendingClock;
    GenericReceiver<RobotMessage> receiver;
};
