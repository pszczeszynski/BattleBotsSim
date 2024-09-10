#include "RobotLink.h"
#include "Clock.h"
#include "../Common/Communication.h"
#include "Globals.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "UIWidgets/ClockWidget.h"
#include "UIWidgets/GraphWidget.h"
#include "UIWidgets/GraphWidget.h"
#include "Strategies/DriveToAngleSimulation.h"
#include "Odometry/OdometryBase.h"
#include "RobotController.h"

#define USB_RETRY_TIME 50
#define HID_BUFFER_SIZE 64
#define SEND_TIMEOUT_MS 1 // after this time, give up sending and go back to receiving
#define RECEIVE_TIMEOUT_MS 100 //Was 5 but doubly defined to 100 later in code, assuming 100 is the intended duration

/**
 * Doesn't do anything except call the timer markStart
 * Must be called for any robotlink
 */
RobotMessage IRobotLink::Receive()
{
    static GraphWidget recvIntervalTime("Received Packet Interval Time", 0, 50, "ms");

    // get all new messages
    std::vector<RobotMessage> newMessages = _ReceiveImpl();

    if (newMessages.size() == 0)
    {
        return RobotMessage{RobotMessageType::INVALID};
    }

    // go through each message, save the latest IMU and CAN message
    bool hadValidMessage = false;

    // go through all the new messages
    for (RobotMessage &msg : newMessages)
    {
        // save the latest message of each type
        if (msg.type == RobotMessageType::IMU_DATA)
        {
            _lastIMUMessageMutex.lock();
            _lastIMUMessage = msg;
            _lastIMUMessageMutex.unlock();
            hadValidMessage = true;
        }
        else if (msg.type == RobotMessageType::CAN_DATA)
        {
            _lastCANMessageMutex.lock();
            _lastCANMessage = msg;
            _lastCANMessageMutex.unlock();
            hadValidMessage = true;
        }
        else if (msg.type == RobotMessageType::RADIO_DATA)
        {
            _lastRadioMessageMutex.lock();
            _lastRadioMessage = msg;
            _lastRadioMessageMutex.unlock();
            hadValidMessage = true;
        }
        else if (msg.type == RobotMessageType::BOARD_TELEMETRY_DATA)
        {
            _lastBoardTelemetryMessageMutex.lock();
            _lastBoardTelemetryMessage = msg;
            _lastBoardTelemetryMessageMutex.unlock();
            hadValidMessage = true;
        }
    }

    // if didn't get a single valid message
    if (!hadValidMessage)
    {
        // print error, return invalid
        std::cerr << "ERROR: invalid message type" << std::endl;
        return RobotMessage{RobotMessageType::INVALID};
    }

    // copy over new messages to the message history
    for (RobotMessage &msg : newMessages)
    {
        // display delay
        recvIntervalTime.AddData(_receiveClock.getElapsedTime() * 1000);
        // restart the last receive clock
        _receiveClock.markStart();
    }

    // return the last message
    return newMessages.back();
}

RobotMessage IRobotLink::GetLastIMUMessage()
{
    RobotMessage ret;
    _lastIMUMessageMutex.lock();
    ret = _lastIMUMessage;
    _lastIMUMessageMutex.unlock();
    return ret;
}

RobotMessage IRobotLink::GetLastCANMessage()
{
    RobotMessage ret;
    _lastCANMessageMutex.lock();
    ret = _lastCANMessage;
    _lastCANMessageMutex.unlock();
    return ret;
}

RobotMessage IRobotLink::GetLastRadioMessage()
{
    RobotMessage ret;

    if (_receiveClock.getElapsedTime() * 1000 > 100)
    {
        // set to 0's
        memset(&ret, 0, sizeof(RobotMessage));
        ret.type = RobotMessageType::RADIO_DATA;
        ret.radioData.averageDelayMS = -1;
        return ret;
    }

    _lastRadioMessageMutex.lock();
    ret = _lastRadioMessage;
    _lastRadioMessageMutex.unlock();
    return ret;
}

RobotMessage IRobotLink::GetLastBoardTelemetryMessage()
{
    RobotMessage ret;
    _lastBoardTelemetryMessageMutex.lock();
    ret = _lastBoardTelemetryMessage;
    _lastBoardTelemetryMessageMutex.unlock();
    return ret;
}

bool IRobotLink::IsTransmitterConnected()
{
    return _transmitterConnected;
}

bool IRobotLink::IsSecondaryTransmitterConnected()
{
    return _secondaryTransmitterConnected;
}


#define TRANSMITTER_COM_PORT TEXT("COM8")

#define COM_READ_TIMEOUT_MS 100
#define COM_WRITE_TIMEOUT_MS 100

char lastChar = '\0';
int messageCount = 0;

#ifndef SIMULATION

#define RADIO_DEVICES 5

// will wait for up to 5ms before quitting if only one device found
#define PRIMARY_TIMEOUT 0.05
// tries to connect to devices and determine which ones are TX devices
void RobotLinkReal::TryConnection(void)
{
    _transmitterConnected = false;
    _secondaryTransmitterConnected = false;
    DriverStationMessage ping_request;
    ping_request.type = LOCAL_PING_REQUEST;
    ping_request.valid = true;
    _primary_radio_index = -1;
    _secondary_radio_index = -1;
    char pingbuf[HID_BUFFER_SIZE] = {0};
    char recvbuf[HID_BUFFER_SIZE] = {0};
    memcpy(pingbuf, &ping_request, sizeof(DriverStationMessage));

    Clock c_retryTime;

    while(true)
    {
        // try to open devices
        int devices_opened = rawhid_open(RADIO_DEVICES, 0x16C0, 0x0486, 0xFFAB, 0x0200);
        c_retryTime.markStart();
        if (devices_opened >= 1)
        {

#ifdef PINGPONG
            for(int i = 0; i < devices_opened; i++) {
                rawhid_send(i, pingbuf, HID_BUFFER_SIZE, SEND_TIMEOUT_MS);
            }
            while(c_retryTime.getElapsedTime() < PRIMARY_TIMEOUT)
            {
                for(int i = 0; i < devices_opened; i++) {
                    int num = rawhid_recv(i, recvbuf, HID_BUFFER_SIZE, 2);
                    if (num >= sizeof(RobotMessage))
                    {
                        // reinterpret the buffer as a RobotMessage
                        RobotMessage msg = *reinterpret_cast<RobotMessage *>(recvbuf);
                        if (msg.type == LOCAL_PING_RESPONSE) 
                        {
                            if (!_transmitterConnected)
                            {
                                _transmitterConnected = true;
                                _primary_radio_index = i;
                            }
                            else if (!_secondaryTransmitterConnected) 
                            {
                                _secondaryTransmitterConnected = true;
                                _secondary_radio_index = i;
                                return;
                            }
                        }
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (_transmitterConnected) {
                return;
            }
#else
            if (devices_opened == 1) {
                _transmitterConnected = true;
                _primary_radio_index = 0;
            }
            else
            {
                _transmitterConnected = true;
                _primary_radio_index = 0;

                _secondaryTransmitterConnected = true;
                _secondary_radio_index = 1;
            }
            return;
#endif
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(USB_RETRY_TIME));
    }
}

#ifdef PINGPONG
// attempt to find a secondary radio every 5 seconds if only primary radio is connected
#define SECONDARY_RETRY 5
#else
#define SECONDARY_RETRY 0.5
#endif

void RobotLinkReal::RadioThreadSendFunction(RawHID *dev, bool *newMessage, DriverStationMessage *message, std::mutex *messageMutex, bool delay)
{
    int num;
    char buf[HID_BUFFER_SIZE];

    auto start = std::chrono::high_resolution_clock::now();

    //static GraphWidget statusCode("Radio status code", -5, 100, "");
    
    while(true)
    {
        if(!_radio_reinit && dev->IsOpen())
        {
            if(!dev->IsSendPending() && *newMessage)
            {
                if (messageMutex->try_lock())
                {
                    memcpy(buf, message, sizeof(DriverStationMessage));
                    messageMutex->unlock();
                    //if (delay) std::this_thread::sleep_for(std::chrono::microseconds(100));
                    if (delay) {
                        auto elapsed = std::chrono::high_resolution_clock::now() - _startTime;
                        uint32_t start = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
                        while (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() - start < 300) {
                            elapsed = std::chrono::high_resolution_clock::now() - _startTime;
                        }
                    }
                    if (!dev->SendAsync(buf, HID_BUFFER_SIZE)) _radio_reinit = true;
                    *newMessage = false;
                }
            }
            else if (dev->IsSendPending())
            {
                num = dev->CheckSendAsync();
                if (num < 0)
                {
                    _radio_reinit = true;
                }
            }
        }
        //std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void RobotLinkReal::RadioThreadRecvFunction(RawHID *dev, std::mutex *messageMutex, std::deque<RobotMessage> *messageQueue)
{
    static GraphWidget packetRoundTrip("Packet Round-trip Time", 0, 5, "ms", 1000);
    int num;
    char buf[HID_BUFFER_SIZE];

    Clock recvTimeout;
    
    while (true)
    {
        // only run if radios are properly working
        if(!_radio_reinit && dev->IsOpen())
        {
            if(!dev->IsRecvPending())
            {
                if (!dev->RecvAsync(HID_BUFFER_SIZE)) _radio_reinit = true;
                recvTimeout.markStart();
            }
            else
            {
                num = dev->CheckRecvAsync(HID_BUFFER_SIZE, buf);
                if (num < 0) _radio_reinit = true;
                if (num >= sizeof(RobotMessage))
                {
                    printf("received packet\n");
                    // reinterpret the buffer as a RobotMessage
                    RobotMessage msg = *reinterpret_cast<RobotMessage *>(buf);

                    if (msg.type != RobotMessageType::INVALID)
                    {
                        // copy over data
                        
                        _outstandingPacketMutex.lock();
                        if(_outstandingPackets.find(msg.timestamp) != _outstandingPackets.end())
                        {
                            _outstandingPackets.erase(msg.timestamp);
                            messageMutex->lock();
                            messageQueue->push_back(msg);
                            messageMutex->unlock();
                            auto elapsed = std::chrono::high_resolution_clock::now() - _startTime;
                            packetRoundTrip.AddData(float(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() - msg.timestamp)/1000);

                        }
                        _outstandingPacketMutex.unlock();
                    }
                    else
                    {
                        std::cerr << "ERROR: invalid message type" << std::endl;
                    }
                }
                else if (recvTimeout.getElapsedTime() > 1)
                {
                    dev->ResetRecv();
                    recvTimeout.markStart();
                    printf("reset timeout\n");
                }
            }
        }
        //std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void RobotLinkReal::RadioThreadFunction(void)
{
    int devices_opened = 0;

    _transmitterConnected = false;
    _secondaryTransmitterConnected = false;
    _radio_reinit = false;

    Clock secondaryRetryTimer;

    std::thread primayRadioSendThread(&RobotLinkReal::RadioThreadSendFunction, this, &_radios[0], &_requestSend, &_messageToSend, &_sendMessageMutex, false);
    std::thread primaryRadioRecvThread(&RobotLinkReal::RadioThreadRecvFunction, this, &_radios[0], &_unconsumedMessagesMutex, &_unconsumedMessages);

    std::thread secondaryRadioSendThread(&RobotLinkReal::RadioThreadSendFunction, this, &_radios[1], &_secondaryRequestSend, &_secondaryMessageToSend, &_secondarySendMessageMutex, true);
    std::thread secondaryRadioRecvThread(&RobotLinkReal::RadioThreadRecvFunction, this, &_radios[1], &_unconsumedMessagesMutex, &_unconsumedMessages);
    while (true)
    {
        while(devices_opened < 1)
        {
            devices_opened = RawHID_Open(_radios, 2, 0x16C0, 0x0486, 0xFFAB, 0x0200);
            std::this_thread::sleep_for(std::chrono::milliseconds(USB_RETRY_TIME));
        }
        _transmitterConnected = true;
        _secondaryTransmitterConnected = (bool)(devices_opened > 1);
        _radio_reinit = false;
        printf("Radio Reinit\n");
        if(!_secondaryTransmitterConnected)
        {
            secondaryRetryTimer.markStart();
        }
        else
        {
            secondaryRetryTimer.markEnd();
        }
        while (!_radio_reinit)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (secondaryRetryTimer.isRunning() && secondaryRetryTimer.getElapsedTime() > 5)
            {
                break;
            }
        }
        //printf("Radio Reinit needed\n");
        _radios[0].Close();
        _radios[1].Close();
        _transmitterConnected = false;
        _secondaryTransmitterConnected = false;
        devices_opened = 0;
    }
}

RobotLinkReal::RobotLinkReal()
{
    // init last can message
    memset(&_lastCANMessage, 0, sizeof(_lastCANMessage));
    _startTime = std::chrono::high_resolution_clock::now();
    _radioThread = std::thread(&RobotLinkReal::RadioThreadFunction, this);
}

/**
 * Chooses the best of 3 channels to send on
 * Monitors for dropouts and switches channels if necessary
 */
int RobotLinkReal::ChooseBestChannel(DriverStationMessage& msg)
{
    if (!_transmitterConnected)
    {
        return RADIO_CHANNEL;
    }

    // draw WARNING if switched
    if (_lastRadioSwitchClock.getElapsedTime() * 1000 < SWITCH_COOLDOWN_MS)
    {
        cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();
        cv::putText(drawingImage, "Switched to Radio " + std::to_string(RADIO_CHANNEL),
                    cv::Point(WIDTH / 2 - 200, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    // return to teensy_channel_1 if self righter is activated since only the center board can control it
    if (msg.type == DriverStationMessageType::DRIVE_COMMAND && abs(msg.driveCommand.selfRighterPower) > 0)
    {
        RADIO_CHANNEL = TEENSY_RADIO_1;
        _lastRadioSwitchClock.markStart();
    }

    // return the current channel if auto switch not enabled or if we have switched too recently
    if (!AUTO_SWITCH_CHANNEL || _lastRadioSwitchClock.getElapsedTime() * 1000 < SWITCH_COOLDOWN_MS)
    {
        return RADIO_CHANNEL;
    }

    // get the latest radio data
    RadioData data = GetLastRadioMessage().radioData;

    // check if the average delay is too high
    if (data.averageDelayMS > MAX_AVERAGE_DELAY_MS ||
        _receiveClock.getElapsedTime() * 1000 > MAX_AVERAGE_DELAY_MS)
    {
        // restart the cooldown clock
        _lastRadioSwitchClock.markStart();

        // switch to the next channel
        if (RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            return TEENSY_RADIO_2;
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            return TEENSY_RADIO_3;
        }
        else
        {
            return TEENSY_RADIO_1;
        }
    }

    // keep the current channel
    return RADIO_CHANNEL;
}

void RobotLinkReal::Drive(DriverStationMessage &command)
{
    static ClockWidget clockWidget{"Send drive command"};
    static ClockWidget interSendTime("Inter Send Time");
    static GraphWidget _radioPacketLoss("Radio Packet Loss", 0, 20, "", 100);
    static Clock packetLossUpdateRate;

    // set the radio channel
    RADIO_CHANNEL = ChooseBestChannel(command);
    command.radioChannel = RADIO_CHANNEL;

    // if we have sent a packet too recently, return
    if (_sendClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS)
    {
        return;
    }
    static GraphWidget stickX("Applied Movement", -1, 1, "");
    static GraphWidget stickY("Applied Turn", -1, 1, "");

    if (command.type == DRIVE_COMMAND)
    {
        stickX.AddData(command.driveCommand.movement / (MASTER_MOVE_SCALE_PERCENT / 100.0));
        stickY.AddData(command.driveCommand.turn / (MASTER_TURN_SCALE_PERCENT / 100.00));
    }
    else if (command.type == AUTO_DRIVE)
    {
        stickX.AddData(command.autoDrive.movement / (MASTER_MOVE_SCALE_PERCENT / 100.0));
        stickY.AddData(command.autoDrive.targetAngle);
    }
    else if (command.type == INVALID_DS)
    {
        std::cerr << "ERROR: invalid driver station message type" << std::endl;
        return;
    }

    clockWidget.markStart();

    auto elapsed = std::chrono::high_resolution_clock::now() - _startTime;
    command.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

    // set valid to true
    command.valid = true;

    try
    {
        // acquire sending mutex
        _sendMessageMutex.lock();
        // set message to send
        _messageToSend = command;
        // set requestSend to true
        _requestSend = true;
        // release sending mutex
        _sendMessageMutex.unlock();

        // acquire sending mutex
        _secondarySendMessageMutex.lock();
        // set message to send
        _secondaryMessageToSend = command;
        _secondaryMessageToSend.radioChannel = SECONDARY_RADIO_CHANNEL;
        // set requestSend to true
        _secondaryRequestSend = true;
        // release sending mutex
        _secondarySendMessageMutex.unlock();

        _sendClock.markStart();
        interSendTime.markEnd();
        interSendTime.markStart();

        _outstandingPacketMutex.lock();
        _outstandingPackets.insert(command.timestamp);
        _outstandingPacketMutex.unlock();
    }
    catch (std::exception &e)
    {
        std::cerr << "Error sending message: " << e.what() << std::endl;
    }

    if(!packetLossUpdateRate.isRunning() || packetLossUpdateRate.getElapsedTime() > 0.1)
    {
        _outstandingPacketMutex.lock();
        _radioPacketLoss.AddData(_outstandingPackets.size() - _outstandingPacketCouner);
        _outstandingPacketCouner = _outstandingPackets.size();
        _outstandingPacketMutex.unlock();
        packetLossUpdateRate.markStart();
    }

    clockWidget.markEnd();
}


std::vector<RobotMessage> RobotLinkReal::_ReceiveImpl()
{
    std::vector<RobotMessage> ret = {};

    // lock mutex
    _unconsumedMessagesMutex.lock();
    // copy over unconsumed messages to new messages
    std::copy(_unconsumedMessages.begin(), _unconsumedMessages.end(), std::back_inserter(ret));
    // reset unconsumed messages since we have copied them over
    _unconsumedMessages.clear();

    // unlock mutex
    _unconsumedMessagesMutex.unlock();

    // return the new messages
    return ret;
}

RobotLinkReal::~RobotLinkReal()
{
}
#endif

/////////////////// SIMULATION //////////////////////

RobotLinkSim::RobotLinkSim()
    : serverSocket("11115")
{
}

void RobotLinkSim::Drive(DriverStationMessage &msg)
{
    DriveCommand command;

    if (msg.type == DRIVE_COMMAND)
    {
        command = msg.driveCommand;
    }
    else if (msg.type == AUTO_DRIVE)
    {
        OdometryData odometry = RobotController::GetInstance().odometry.Robot();
        float imu_rotation = odometry.robotAngle;
        float imu_rotation_velocity = odometry.robotAngleVelocity;

        AutoDrive lastAutoCommand = msg.autoDrive;

        // drive to the target angle
        command = DriveToAngle(imu_rotation,
                               imu_rotation_velocity,
                               lastAutoCommand.targetAngle,
                               lastAutoCommand.ANGLE_EXTRAPOLATE_MS,
                               lastAutoCommand.TURN_THRESH_1_DEG,
                               lastAutoCommand.TURN_THRESH_2_DEG,
                               lastAutoCommand.MAX_TURN_POWER_PERCENT,
                               lastAutoCommand.MIN_TURN_POWER_PERCENT,
                               lastAutoCommand.SCALE_DOWN_MOVEMENT_PERCENT);
        
        command.movement = lastAutoCommand.movement;
    }

    UnityDriveCommand message = {command.movement, command.turn, (double)command.frontWeaponPower, (double)command.backWeaponPower};
    serverSocket.reply_to_last_sender(RobotStateParser::serialize(message));
}

#define ACCELEROMETER_TO_PX_SCALER 1

std::vector<RobotMessage> RobotLinkSim::_ReceiveImpl()
{
    static Clock lastCanDataClock;

    RobotMessage ret{RobotMessageType::INVALID};
    // zero out ret
    memset(&ret, 0, sizeof(ret));

    std::string received = "";
    while (received == "")
    {
        received = serverSocket.receive();
    }
    UnityRobotState message = RobotStateParser::parse(received);
    // set the global variable
    opponentRotationSim = angle_wrap(message.opponent_orientation * TO_RAD + M_PI);
    robotPosSim = cv::Point2f((float) message.robot_position.x, (float) message.robot_position.z);
    opponentPosSim = cv::Point2f{(float) message.opponent_position.x, (float) message.opponent_position.z};


    cv::Point2f mins = {-10, -10};
    cv::Point2f maxs = {10, 10};

    robotPosSim -= mins;
    robotPosSim.x /= maxs.x - mins.x;
    robotPosSim.y /= maxs.y - mins.y;

    robotPosSim.x = 1.0 - robotPosSim.x;
    robotPosSim *= WIDTH;

    opponentPosSim -= mins;
    opponentPosSim.x /= maxs.x - mins.x;
    opponentPosSim.y /= maxs.y - mins.y;
    opponentPosSim.x = 1.0 - opponentPosSim.x;
    opponentPosSim *= WIDTH;

    // if we have already had a can message in last 1/2 second
    if (lastCanDataClock.getElapsedTime() < 0.5)
    {
        ret.type = RobotMessageType::IMU_DATA;
        ret.imuData.rotation = Angle(message.robot_orientation + M_PI);
        ret.imuData.rotationVelocity = message.robot_rotation_velocity;
    }
    else
    {
        ret.type = RobotMessageType::CAN_DATA;
        ret.canData.motorERPM[2] = (int)abs(message.spinner_1_RPM * RPM_TO_ERPM / ERPM_FIELD_SCALAR) * 9;
        ret.canData.motorERPM[3] = (int)abs(message.spinner_2_RPM * RPM_TO_ERPM / ERPM_FIELD_SCALAR) * 9;
        lastCanDataClock.markStart();
    }

    // return the message
    return {ret};
}
