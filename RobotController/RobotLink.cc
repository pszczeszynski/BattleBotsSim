#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"
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
    static GraphWidget radioPacketLoss("Radio Packet Loss", 0, 50, "ms");

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
        radioPacketLoss.AddData(_receiveClock.getElapsedTime() * 1000);
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

bool IRobotLink::IsTransmitterConnected()
{
    return _transmitterConnected;
}

#define TRANSMITTER_COM_PORT TEXT("COM8")

#define COM_READ_TIMEOUT_MS 100
#define COM_WRITE_TIMEOUT_MS 100

char lastChar = '\0';
int messageCount = 0;


ClockWidget receiveThreadLoopTime("Receive thread loop time");

#ifndef SIMULATION
RobotLinkReal::RobotLinkReal()
{
    _radioThread = std::thread([this]()
                               {

            Clock c_rawhid; // Clock for not spamming rawhid error
            
            while (true)
            {
                _transmitterConnected = false;
                _lastRadioMessageMutex.lock();
                // set to 0's
                memset(&_lastRadioMessage, 0, sizeof(RobotMessage));
                _lastRadioMessage.radioData.averageDelayMS = -1;
                _lastRadioMessageMutex.unlock();

                int i, devices_opened, num;
                char c;
                char buf[HID_BUFFER_SIZE];

                // C-based example is 16C0:0480:FFAB:0200
                devices_opened = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
                // opened sucessfully

                // while not opened
                while (devices_opened <= 0)
                {
                    // Arduino-based example is 16C0:0486:FFAB:0200
                    devices_opened = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
                    if (devices_opened <= 0)
                    {
                        // Report error but don't spam it
                        if( !c_rawhid.isRunning() || (c_rawhid.getElapsedTime() > 1.0))
                        {
                            c_rawhid.markStart();
                            std::cerr << "no rawhid device found" << std::endl;
                        }
                        
                        std::this_thread::sleep_for(std::chrono::milliseconds(USB_RETRY_TIME));
                    }
                }

                printf("found rawhid device with handle %d\n", devices_opened);
                _transmitterConnected = true;

                // keep reading until error or device goes offline
                while (true)
                {
                    receiveThreadLoopTime.markStart();

                    // check if any Raw HID packet has arrived
                    num = rawhid_recv(0, buf, HID_BUFFER_SIZE, RECEIVE_TIMEOUT_MS);

                    if (num < 0)
                    {
                        printf("\nerror reading, device went offline\n");
                        rawhid_close(0);
                        receiveThreadLoopTime.markEnd();
                        break;
                    }

                    // if there is enough data for a RobotMessage
                    if (num >= sizeof(RobotMessage))
                    {
                        // reinterpret the buffer as a RobotMessage
                        RobotMessage msg = *reinterpret_cast<RobotMessage *>(buf);

                        if (msg.type != RobotMessageType::INVALID)
                        {
                            // copy over data
                            _unconsumedMessagesMutex.lock();
                            _unconsumedMessages.push_back(msg);
                            _unconsumedMessagesMutex.unlock();
                        }
                        else
                        {
                            std::cerr << "ERROR: invalid message type" << std::endl;
                        }
                    }


                    // if the main thread wants to send a message
                    // try lock
                    int status = 0;

                    _sendMessageMutex.lock();
                    if (_requestSend)
                    {
                        static ClockWidget sendTime("Send time");
                        static GraphWidget statusCode("Radio status code", -5, 100, "");
                        sendTime.markStart();

                        // copy over the message to send
                        memcpy(buf, &_messageToSend, sizeof(DriverStationMessage));

                        // send the message
                        status = rawhid_send(0, buf, HID_BUFFER_SIZE, SEND_TIMEOUT_MS);

                        statusCode.AddData(status);
                        sendTime.markEnd();
                        // reset requestSend
                        _requestSend = false;
                    }
    
                    _sendMessageMutex.unlock();

                    if (status < 0)
                    {
                        std::cerr << "Error sending message" << std::endl;
                        rawhid_close(0);
                        break;
                    }
                } 
            } });
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

        _sendClock.markStart();
    }
    catch (std::exception &e)
    {
        std::cerr << "Error sending message: " << e.what() << std::endl;
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
