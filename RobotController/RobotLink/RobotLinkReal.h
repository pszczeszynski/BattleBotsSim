#pragma once

#include "IRobotLink.h"
#define NOMINMAX
#include "../hid/hid.h"
#include <set>

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
