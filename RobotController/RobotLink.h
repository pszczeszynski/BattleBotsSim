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
    virtual RobotMessage Receive();
    Clock receiveClock; // for tracking the receive rate information (so public)
    Clock sendClock; // for tracking the send rate information (so public)
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

class RobotLinkReal : public IRobotLink
{
public:
    RobotLinkReal();
    virtual void Drive(DriveCommand &command) override;
    virtual RobotMessage Receive() override;
    ~RobotLinkReal();

private:
    void _WriteSerialMessage(const char *message, int messageLength);
    void _InitComPort();

    HANDLE _comPort;
    DCB _dcbSerialParams;
    Clock _sendingClock;
    GenericReceiver<RobotMessage> _receiver;
};
