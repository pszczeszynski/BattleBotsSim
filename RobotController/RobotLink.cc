#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"
#include "Globals.h"
#include "MathUtils.h"

RobotLinkReal::RobotLinkReal() : receiver(200, [this](char &c)
                                          {
    DWORD dwBytesRead = 0;
    DWORD dwErrors = 0;
    COMSTAT comStat;

    // Get and clear current errors on the com port.
    if (!ClearCommError(comPort, &dwErrors, &comStat))
    {
        // attempt to reinitialize com port
        InitComPort();
        std::cerr << "Error clearing COM port" << std::endl;
        return false;
    }

    // If there is nothing to read, return false early.
    if (comStat.cbInQue == 0)
    {
        return false;
    }

    // check if there is data to read
    if(!ReadFile(comPort, &c, 1, &dwBytesRead, NULL))
    {
        std::cerr << "Error reading from COM port" << std::endl;
    }
    return dwBytesRead > 0; })
{
    InitComPort();
    sendingClock.markStart();
}

void RobotLinkReal::InitComPort()
{
    static int numTries = 0;
    const int NUM_TRIES_BEFORE_PRINTING_ERROR = 500;

    comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (comPort == INVALID_HANDLE_VALUE)
    {
        // increase numTries
        numTries++;
        numTries %= NUM_TRIES_BEFORE_PRINTING_ERROR;

        // if we have tried NUM_TRIES_BEFORE_PRINTING_ERROR times, print error
        if (numTries % NUM_TRIES_BEFORE_PRINTING_ERROR == 1)
        {
            std::cerr << "Error opening port: " << TRANSMITTER_COM_PORT << std::endl;
        }

        // break out of function
        return;
    }
    else
    {
        numTries = 0;
        std::cout << "Transmitter COM Port opened successfully" << std::endl;
    }

    dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    dcbSerialParams.BaudRate = CBR_115200; // set the baud rate
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    if (!SetCommTimeouts(comPort, &timeouts))
    {
        std::cerr << "Error setting COM port timeouts" << std::endl;
    }

    if (!SetCommState(comPort, &dcbSerialParams))
    {
        std::cerr << "Error setting serial port state" << std::endl;
    }
}

#define DRIVE_SCALE 1.0

#define MIN_INTER_SEND_TIME_MS 5
void RobotLinkReal::Drive(DriveCommand &command)
{
    if (sendingClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS)
    {
        return;
    }

    if (comPort == INVALID_HANDLE_VALUE)
    {
        // attempt to reopen
        InitComPort();
        return;
    }

    sendingClock.markStart();

    command.movement = std::max(-1.0, std::min(1.0, command.movement));
    command.turn = std::max(-1.0, std::min(1.0, command.turn));

    // scale down
    command.movement *= DRIVE_SCALE;
    command.turn *= DRIVE_SCALE;

    command.valid = true;

    // write start MESSAGE_START_CHAR
    DWORD dwBytesWritten = 0;
    char start = MESSAGE_START_CHAR;
    WriteFile(comPort, &start, sizeof(start), &dwBytesWritten, NULL);
    // write command
    WriteFile(comPort, &command, sizeof(command), &dwBytesWritten, NULL);

    std::cout << "sending command: " << command.movement << ", " << command.turn << std::endl;

    // write start MESSAGE_END_CHAR
    char end = MESSAGE_END_CHAR;
    WriteFile(comPort, &end, sizeof(end), &dwBytesWritten, NULL);
}


int receivedPackets;
Clock lastReceivedTime;

#define RECEIVE_TIMEOUT_MS 100

RobotMessage RobotLinkReal::Receive()
{
    if (comPort == INVALID_HANDLE_VALUE)
    {
        // attempt to reopen
        InitComPort();
        return RobotMessage{0};
    }

    receiver.update();

    // Clock receiveWaitTimer;
    // receiveWaitTimer.markStart();

    // // wait until we have a valid message
    // while (!receiver.isLatestDataValid() &&
    //         receiveWaitTimer.getElapsedTime() * 1000 < RECEIVE_TIMEOUT_MS)
    // {
    //     receiver.update();
    // }
    RobotMessage retrievedStruct = receiver.getLatestData();

    if (receiver.isLatestDataValid())
    {
        receivedPackets++;
    }

    if (receivedPackets > 1000)
    {
        // std::cout << "received per second: " << receivedPackets / lastReceivedTime.getElapsedTime() << std::endl;
        receivedPackets = 0;
        lastReceivedTime.markStart();
    }


    return retrievedStruct;
}

RobotLinkReal::~RobotLinkReal()
{
    CloseHandle(comPort);
}

/////////////////// SIMULATION //////////////////////

RobotLinkSim::RobotLinkSim()
    : serverSocket("11115")
{
}

void RobotLinkSim::Drive(DriveCommand &command)
{
    UnityDriveCommand message = {command.movement, command.turn};
    serverSocket.reply_to_last_sender(RobotStateParser::serialize(message));
}

#define ACCELEROMETER_TO_PX_SCALER 0.05854 * WIDTH

RobotMessage RobotLinkSim::Receive()
{
    std::string received = "";
    while (received == "")
    {
        received = serverSocket.receive();
    }
    UnityRobotState message = RobotStateParser::parse(received);

    RobotMessage ret{0};
    // unity uses different coordinate system, swap it to be same as imu
    ret.velocity = Point{message.robot_velocity.x, message.robot_velocity.y, -message.robot_velocity.z};
    ret.velocity.x *= ACCELEROMETER_TO_PX_SCALER;
    ret.velocity.y *= ACCELEROMETER_TO_PX_SCALER;
    ret.velocity.z *= ACCELEROMETER_TO_PX_SCALER;
    ret.rotation = message.robot_orientation;
    ret.rotationVelocity = message.robot_rotation_velocity;

    // return the message
    return ret;
}