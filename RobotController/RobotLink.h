#pragma once

#include "RobotStateParser.h"
#include "Windows.h"
#include "ServerSocket.h"

// driver station -> robot
struct DriveCommand
{
    double movement;
    double turn;
};

// robot -> driver station
struct RobotMessage
{
    Point accel;
    Point velocity;
    double rotation;
};

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

/**
 * This drives the real robot
*/
class RobotLinkReal : public IRobotLink
{
public:
    RobotLinkReal();
    virtual void Drive(DriveCommand& command) override;
    virtual RobotMessage Receive() override;
    ~RobotLinkReal();
private:
    // radio stuff
    HANDLE hPort;
};  