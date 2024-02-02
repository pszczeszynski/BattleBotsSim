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
    virtual RobotMessage _ReceiveImpl() = 0;

    RobotMessage _lastIMUMessage;
    RobotMessage _lastCANMessage;

    // store the last 1000 messages
    const int MESSAGE_HISTORY_SIZE = 1000;
    std::deque<RobotMessage> _messageHistory;

    Clock _receiveClock; // for tracking the receive rate information (so public)
    Clock _sendClock; // for tracking the send rate information (so public)
    std::vector<RobotMessage> _mainThreadUnconsumedMessages;

};

/**
 * This drives the simulated robot
*/
class RobotLinkSim : public IRobotLink
{
public:
    RobotLinkSim();
    virtual void Drive(DriveCommand& command) override;
    virtual RobotMessage _ReceiveImpl() override;

private:
    // socket stuff
    void setup();
    ServerSocket serverSocket;
    RobotMessage _lastMessage;
};

class RobotLinkReal : public IRobotLink
{
public:
    RobotLinkReal();
    virtual void Drive(DriveCommand &command) override;
    virtual RobotMessage _ReceiveImpl() override;

    ~RobotLinkReal();

private:
    Clock _sendingClock;
    std::thread _radioThread;
    RobotMessage _lastMessage;

    std::vector<RobotMessage> _unconsumedMessages;

    unsigned long _newestMessageID;
    unsigned long _lastConsumedMessageID;
    std::mutex _lastMessageMutex;

    // these fields are mutex protected
    std::mutex _sendMessageMutex;
    DriveCommand _messageToSend;
    bool _requestSend;


    std::mutex _comPortMutex;
};
