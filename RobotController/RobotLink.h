#pragma once

#include "RobotStateParser.h"
#include "windows.h"
#include "ServerSocket.h"
#include "../Common/Communication.h"
#include "DriverStationLog.h"
#include <fstream>
#include <functional>
#include "Clock.h"
#include <deque>
#include <thread>
#include <mutex>
#include "hid/hid.h"
#include <chrono>
#include <deque>
#include <atomic>
#include <set>

// interface
class IRobotLink
{
public:
    virtual void Drive(DriverStationMessage& command) = 0; // sends data to robot
    virtual RobotMessage Receive();
    RobotMessage GetLastIMUMessage();
    RobotMessage GetLastCANMessage();
    RobotMessage GetLastRadioMessage();
    RobotMessage GetLastBoardTelemetryMessage();
    RobotMessage GetLastIMUDebugMessage();
    const std::deque<RobotMessage>& GetMessageHistory();
    bool IsTransmitterConnected();
    bool IsSecondaryTransmitterConnected();

protected:
    // implemented by the subclass
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

    Clock _receiveClock; // for tracking the receive rate information (so public)
    Clock _sendClock; // for tracking the send rate information (so public)
    bool _transmitterConnected = false;
    bool _secondaryTransmitterConnected = false;
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

    void ResetSimulation();
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
    void RegisterLogger(DriverStationLog* logger);

    ~RobotLinkReal();

private:
    void ChooseBestChannel(DriverStationMessage& command);

    void TryConnection(void);
    void RadioThreadFunction(void);
    void RadioThreadSendFunction(RawHID *dev, bool *newMessage,
                                 DriverStationMessage *message,
                                 std::mutex *messageMutex, bool delay);
    void RadioThreadRecvFunction(RawHID *dev, std::mutex *messageMutex,
                                 std::deque<RobotMessage> *messageQueue,
                                 RobotMessage *radioStats,
                                 Clock *lastReceivedTimer,
                                 std::mutex *radioStatsMutex);

    std::atomic<bool> _radio_reinit;
    RawHID _radios[2];

    std::thread _radioThread;
    Clock _radioThreadTimer;
    int _primary_radio_index;
    int _secondary_radio_index;

    std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;


    std::deque<RobotMessage> _unconsumedMessages;
    std::mutex _unconsumedMessagesMutex;

    // these fields are mutex protected
    std::mutex _sendMessageMutex;
    std::mutex _secondarySendMessageMutex;
    DriverStationMessage _messageToSend;
    DriverStationMessage _secondaryMessageToSend;
    bool _requestSend;
    bool _secondaryRequestSend;

    RobotMessage _primaryRadioStats;
    RobotMessage _secondaryRadioStats;

    Clock _primaryReceivedTimer;
    Clock _secondaryReceivedTimer;

    std::mutex _primaryRadioStatsMutex;
    std::mutex _secondaryRadioStatsMutex;

    std::set<uint32_t> _outstandingPackets;
    uint32_t _outstandingPacketCouner = 0;
    std::mutex _outstandingPacketMutex;

    std::mutex _comPortMutex;

    Clock _lastRadioSwitchClock;
    DriverStationLog *_logger;
};

#endif