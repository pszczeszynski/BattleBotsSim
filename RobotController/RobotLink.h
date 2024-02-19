#pragma once

#include "RobotStateParser.h"
#include "windows.h"
#include "ServerSocket.h"
#include "../Communication/Communication.h"
#include "../Communication/GenericReceiver.h"
#include <fstream>
#include <functional>
#include "Clock.h"
#include <deque>
#include <thread>
#include <mutex>
#include "hid/hid.h"
#include <chrono>
#include <deque>

// interface
class IRobotLink
{
public:
    virtual void Drive(DriveCommand& command) = 0; // sends data to robot
    virtual RobotMessage Receive();
    RobotMessage GetLastIMUMessage();
    RobotMessage GetLastCANMessage();
    const std::deque<RobotMessage>& GetMessageHistory();

protected:
    // implemented by the subclass
    virtual std::vector<RobotMessage> _ReceiveImpl() = 0;

    RobotMessage _lastIMUMessage;
    RobotMessage _lastCANMessage;


    Clock _receiveClock; // for tracking the receive rate information (so public)
    Clock _sendClock; // for tracking the send rate information (so public)
};

/**
 * This drives the simulated robot
*/
class RobotLinkSim : public IRobotLink
{
public:
    RobotLinkSim();
    virtual void Drive(DriveCommand& command) override;
    virtual std::vector<RobotMessage> _ReceiveImpl() override;

private:
    // socket stuff
    void setup();
    ServerSocket serverSocket;
    RobotMessage _lastMessage;
};

#ifndef SIMULATION
class RobotLinkReal : public IRobotLink
{
public:
    RobotLinkReal();
    virtual void Drive(DriveCommand &command) override;
    virtual std::vector<RobotMessage> _ReceiveImpl() override;

    ~RobotLinkReal();

private:
    std::thread _radioThread;

    std::deque<RobotMessage> _unconsumedMessages;
    std::mutex _unconsumedMessagesMutex;

    // these fields are mutex protected
    std::mutex _sendMessageMutex;
    DriveCommand _messageToSend;
    bool _requestSend;


    std::mutex _comPortMutex;
};

#endif