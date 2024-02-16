#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"
#include "Globals.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "UIWidgets/ClockWidget.h"
#include "UIWidgets/GraphWidget.h"
#include "UIWidgets/GraphWidget.h"

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
        // save the last IMU and CAN message
        if (msg.type == RobotMessageType::IMU_DATA)
        {
            _lastIMUMessage = msg;
            hadValidMessage = true;
        }
        else if (msg.type == RobotMessageType::CAN_DATA)
        {
            _lastCANMessage = msg;
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
        _receiveClock.markStart();
    }

    // return the last message
    return newMessages.back();
}

RobotMessage IRobotLink::GetLastIMUMessage()
{
    return _lastIMUMessage;
}

RobotMessage IRobotLink::GetLastCANMessage()
{
    return _lastCANMessage;
}

#define TRANSMITTER_COM_PORT TEXT("COM8")

#define COM_READ_TIMEOUT_MS 100
#define COM_WRITE_TIMEOUT_MS 100

char lastChar = '\0';
Clock intermessageClock;
int messageCount = 0;


ClockWidget receiveThreadLoopTime("Receive thread loop time");

RobotLinkReal::RobotLinkReal()
{
    _sendingClock.markStart();

    _radioThread = std::thread([this]()
                               {

            Clock c_rawhid; // Clock for not spamming rawhid error
            
            while (true)
            {
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
                        memcpy(buf, &_messageToSend, sizeof(DriveCommand));

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

void RobotLinkReal::Drive(DriveCommand &command)
{
    static ClockWidget clockWidget{"Send drive command"};
    command.movement *= -1.0;
    command.turn *= -1.0;
    double temp = command.movement;
    command.movement = command.turn;
    command.turn = temp;

    // force command to be between -1 and 1
    command.movement = std::max(-1.0, std::min(1.0, command.movement));
    command.turn = std::max(-1.0, std::min(1.0, command.turn));
    // set valid to true
    command.valid = true;

    // if we have sent a packet too recently, return
    if (_sendingClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS)
    {
        return;
    }
    _sendingClock.markStart();

    clockWidget.markStart();

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

/////////////////// SIMULATION //////////////////////

RobotLinkSim::RobotLinkSim()
    : serverSocket("11115")
{
}

void RobotLinkSim::Drive(DriveCommand &command)
{
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
        ret.imuData.rotation = message.robot_orientation;
        ret.imuData.rotationVelocity = message.robot_rotation_velocity;
    }
    else
    {
        std::cout << "CAN DATA RECEIVED" << std::endl;
        ret.type = RobotMessageType::CAN_DATA;
        ret.canData.motorERPM[2] = (int)abs(message.spinner_1_RPM * RPM_TO_ERPM / ERPM_FIELD_SCALAR);
        ret.canData.motorERPM[3] = (int)abs(message.spinner_2_RPM * RPM_TO_ERPM / ERPM_FIELD_SCALAR);

        std::cout << "RPM: " << message.spinner_1_RPM << " " << message.spinner_2_RPM << std::endl;
        lastCanDataClock.markStart();
    }

    // return the message
    return {ret};
}
