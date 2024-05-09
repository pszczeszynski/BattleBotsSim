#pragma once

#include "RobotStateParser.h"
#include "windows.h"
#include "ServerSocket.h"
#include "../Common/Communication.h"
#include "../Common/Communication.h"
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
    virtual void Drive(DriverStationMessage& command) = 0; // sends data to robot
    virtual RobotMessage Receive();
    RobotMessage GetLastIMUMessage();
    RobotMessage GetLastCANMessage();
    RobotMessage GetLastRadioMessage();
    const std::deque<RobotMessage>& GetMessageHistory();
    bool IsTransmitterConnected();

protected:
    // implemented by the subclass
    virtual std::vector<RobotMessage> _ReceiveImpl() = 0;

    RobotMessage _lastIMUMessage;
    std::mutex _lastIMUMessageMutex;
    RobotMessage _lastCANMessage;
    std::mutex _lastCANMessageMutex;
    RobotMessage _lastRadioMessage;
    std::mutex _lastRadioMessageMutex;

    Clock _receiveClock; // for tracking the receive rate information (so public)
    Clock _sendClock; // for tracking the send rate information (so public)
    bool _transmitterConnected = false;
};

/**
 * This drives the simulated robot
*/
class RobotLinkSim : public IRobotLink
{
public:
    RobotLinkSim();
    virtual void Drive(DriverStationMessage& command) override;
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
    virtual void Drive(DriverStationMessage &command) override;
    virtual std::vector<RobotMessage> _ReceiveImpl() override;

    ~RobotLinkReal();

private:
    int ChooseBestChannel(DriverStationMessage& command);

    std::thread _radioThread;

    std::deque<RobotMessage> _unconsumedMessages;
    std::mutex _unconsumedMessagesMutex;

    // these fields are mutex protected
    std::mutex _sendMessageMutex;
    DriverStationMessage _messageToSend;
    bool _requestSend;


    std::mutex _comPortMutex;

    Clock _lastRadioSwitchClock;
};

#endif