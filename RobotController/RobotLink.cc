#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"

RobotLinkReal::RobotLinkReal() : receiver(200, [this](char &c)
                                          {
    DWORD dwBytesRead = 0;
    DWORD dwErrors = 0;
    COMSTAT comStat;

    // Get and clear current errors on the com port.
    if (!ClearCommError(comPort, &dwErrors, &comStat))
    {
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
    sendingClock.markStart();
    comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (comPort == INVALID_HANDLE_VALUE)
    {
        std::cerr << "Error opening port: " << TRANSMITTER_COM_PORT << std::endl;
        return;
    }
    else
    {
        std::cout << "Transmitter COM Port opened successfully" << std::endl;
    }

    dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    dcbSerialParams.BaudRate = CBR_9600; // set the baud rate
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

#define MIN_INTER_SEND_TIME_MS 10
int i = 0;
void RobotLinkReal::Drive(DriveCommand &command)
{
    if (sendingClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS)
    {
        return;
    }
    sendingClock.markStart();

    command.movement = i;
    command.valid = true;

    // write start MESSAGE_START_CHAR
    DWORD dwBytesWritten = 0;
    char start = MESSAGE_START_CHAR;
    WriteFile(comPort, &start, sizeof(start), &dwBytesWritten, NULL);
    // write command
    WriteFile(comPort, &command, sizeof(command), &dwBytesWritten, NULL);
    // write start MESSAGE_END_CHAR
    char end = MESSAGE_END_CHAR;
    WriteFile(comPort, &end, sizeof(end), &dwBytesWritten, NULL);

    std::cout << "sending" << i << std::endl;
    i++;
}

RobotMessage RobotLinkReal::Receive()
{
    if (comPort == INVALID_HANDLE_VALUE)
    {
        std::cout << "closed" << std::endl;
        return RobotMessage{0};
    }

    receiver.update();

    if (!receiver.isLatestDataValid())
    {
        return RobotMessage{0};
    }

    RobotMessage retrievedStruct = receiver.getLatestData();

    // print accel
    std::cout << "accel: " << retrievedStruct.accel.x << ", " << retrievedStruct.accel.y << ", " << retrievedStruct.accel.z << std::endl;

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
    RobotControllerMessage message = {command.movement, command.turn};
    serverSocket.reply_to_last_sender(RobotStateParser::serialize(message));
}

RobotMessage RobotLinkSim::Receive()
{
    std::string received = "";
    while (received == "")
    {
        received = serverSocket.receive();
    }
    RobotState message = RobotStateParser::parse(received);

    RobotMessage ret{0};
    ret.accel = Point{message.robot_velocity.x, message.robot_velocity.y, message.robot_velocity.z};
    ret.rotation = message.robot_orientation;

    // return the message
    return ret;
}