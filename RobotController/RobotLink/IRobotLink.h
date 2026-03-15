#pragma once

#include "../ServerSocket.h"
#include "../../Common/Communication.h"
#include "../DriverStationLog.h"
#include "../Clock.h"
#include <deque>
#include <thread>
#include <mutex>
#include <functional>
#include <chrono>
#include <atomic>
#include <vector>

class IRobotLink
{
public:
    virtual void Drive(DriverStationMessage& command) = 0;
    virtual RobotMessage Receive();
    RobotMessage GetLastCANMessage();
    RobotMessage GetLastRadioMessage();
    RobotMessage GetLastBoardTelemetryMessage();
    RobotMessage GetLastIMUDebugMessage();
    bool IsTransmitterConnected();
    bool IsSecondaryTransmitterConnected();

    using IMUCallback = std::function<void(const IMUData&, double)>;
    void RegisterIMUCallback(IMUCallback callback);

protected:
    virtual std::vector<RobotMessage> _ReceiveImpl() = 0;

    RobotMessage _lastIMUMessage;
    std::mutex _lastIMUMessageMutex;
    RobotMessage _lastCANMessage;
    std::mutex _lastCANMessageMutex;
    RobotMessage _lastRadioMessage;
    std::mutex _lastRadioMessageMutex;
    RobotMessage _lastBoardTelemetryMessage;
    std::mutex _lastBoardTelemetryMessageMutex;
    RobotMessage _lastIMUDebugMessage;
    std::mutex _lastIMUDebugMessageMutex;

    std::deque<double> _messageReceiveTimes;
    std::vector<double> _lastMessageReceiveTimes;

    Clock _receiveClock;
    Clock _sendClock;
    bool _transmitterConnected = false;
    bool _secondaryTransmitterConnected = false;
    
    IMUCallback _imuCallback;
};
