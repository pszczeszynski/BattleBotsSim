#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"
#include "Globals.h"
#include "MathUtils.h"

/**
 * Doesn't do anything except call the timer markStart
 * Must be called for any robotlink
*/
RobotMessage IRobotLink::Receive()
{
    RobotMessage ret {RobotMessageType::INVALID};

    // mark that we have received a packet
    receiveClock.markStart();

    return ret;
}


#define TRANSMITTER_COM_PORT TEXT("COM9")

#define COM_READ_TIMEOUT_MS 100
#define COM_WRITE_TIMEOUT_MS 100

RobotLinkReal::RobotLinkReal() : _receiver(200, [this](char &c)
                                          {
    DWORD dwBytesRead = 0;
    DWORD dwErrors = 0;
    COMSTAT comStat;

    // Get and clear current errors on the com port.
    if (!ClearCommError(_comPort, &dwErrors, &comStat))
    {
        // attempt to reinitialize com port
        _InitComPort();
        std::cerr << "Error clearing COM port" << std::endl;
        return false;
    }

    // If there is nothing to read, return false early.
    if (comStat.cbInQue == 0)
    {
        return false;
    }

    // check if there is data to read
    if(!ReadFile(_comPort, &c, 1, &dwBytesRead, NULL))
    {
        std::cerr << "Error reading from COM port" << std::endl;
    }
    return dwBytesRead > 0; })
{
    _InitComPort();
    _sendingClock.markStart();
}

void RobotLinkReal::_InitComPort()
{
    static int numTries = 0;
    const int NUM_TRIES_BEFORE_PRINTING_ERROR = 5000;

    // close it if it is open
    CloseHandle(_comPort);

    _comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (_comPort == INVALID_HANDLE_VALUE)
    {
        // increase numTries
        numTries++;
        numTries %= NUM_TRIES_BEFORE_PRINTING_ERROR;

        // if we have tried NUM_TRIES_BEFORE_PRINTING_ERROR times, print error
        if (numTries % NUM_TRIES_BEFORE_PRINTING_ERROR == 1)
        {
            std::cerr << "ERROR: can't open comport" << std::endl;
        }

        // break out of function
        return;
    }
    else
    {
        numTries = 0;
        std::cout << "Transmitter COM Port opened successfully" << std::endl;
    }

    _dcbSerialParams = {0};
    _dcbSerialParams.DCBlength = sizeof(_dcbSerialParams);
    _dcbSerialParams.BaudRate = CBR_115200; // set the baud rate
    _dcbSerialParams.ByteSize = 8;
    _dcbSerialParams.StopBits = ONESTOPBIT;
    _dcbSerialParams.Parity = NOPARITY;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = COM_READ_TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = COM_WRITE_TIMEOUT_MS;

    if (!SetCommTimeouts(_comPort, &timeouts))
    {
        std::cerr << "Error setting COM port timeouts" << std::endl;
    }

    if (!SetCommState(_comPort, &_dcbSerialParams))
    {
        std::cerr << "Error setting serial port state" << std::endl;
    }
}

/**
 * Writes a message to the serial port.
 * @param message The message to write.
 * @param messageLength The length of the message to write.
 * @throws std::runtime_error if the message could not be written.
*/
void RobotLinkReal::_WriteSerialMessage(const char *message, int messageLength)
{
    DWORD dwBytesWritten = 0;
    if (!WriteFile(_comPort, message, messageLength, &dwBytesWritten, NULL))
    {
        throw std::runtime_error("Failed to write message");
    }
}

#define MIN_INTER_SEND_TIME_MS 4
void RobotLinkReal::Drive(DriveCommand &command)
{
    command.selfRighterPower = command.movement;
    command.movement = 0;
    command.turn = 0;

    // std::cout << "command.movement: " << command.movement << std::endl;
    command.movement *= -0.3;
    command.turn *= -0.3;
    // command.turn *= -1;
    double temp = command.movement;
    command.movement = command.turn;
    command.turn = temp;


    // if we have sent a packet too recently, return
    if (_sendingClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS)
    {
        return;
    }

    // if com port is invalid, attempt to reopen
    if (_comPort == INVALID_HANDLE_VALUE)
    {
        // attempt to reopen
        _InitComPort();
        return;
    }

    // if there are errors on the com port, attempt to reopen
    DWORD dwErrors = 0;
    COMSTAT comStat;
    if (!ClearCommError(_comPort, &dwErrors, &comStat))
    {
        // Handle the error with ClearCommError itself
        DWORD dwError = GetLastError();
        std::cerr << "ERROR: can't getting errors from COM port" << std::endl;
    }
    else
    {
        if (dwErrors & CE_BREAK)
            std::cout << "ERROR: The hardware detected a break condition.\n";
        if (dwErrors & CE_FRAME)
            std::cout << "ERROR: The hardware detected a framing error.\n";
        if (dwErrors & CE_OVERRUN)
            std::cout << "ERROR: A character-buffer overrun has occurred.\n";
        if (dwErrors & CE_RXOVER)
            std::cout << "ERROR: An input buffer overflow has occurred.\n";
        if (dwErrors & CE_RXPARITY)
            std::cout << "ERROR: The hardware detected a parity error.\n";
    }

    if (dwErrors != 0)
    {
        // attempt to reopen
        _InitComPort();
        return;
    }

    _sendingClock.markStart();
    // force command to be between -1 and 1
    command.movement = std::max(-1.0, std::min(1.0, command.movement));
    command.turn = std::max(-1.0, std::min(1.0, command.turn));
    // set valid to true
    command.valid = true;

    // now write the message
    try
    {
        // Write MESSAGE_START_CHAR
        char start = MESSAGE_START_CHAR;
        _WriteSerialMessage((char *)&start, sizeof(start));
        // Write command
        _WriteSerialMessage((char *)&command, sizeof(command));
        // Write MESSAGE_END_CHAR
        char end = MESSAGE_END_CHAR;
        _WriteSerialMessage((char *)&end, sizeof(end));

        sendClock.markStart();
    }
    catch (std::exception &e)
    {
        std::cerr << "Failed to write command: " << e.what() << std::endl;
        SAFE_DRAW
        cv::putText(drawingImage, "Failed COM WRITE!", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        END_SAFE_DRAW
        _InitComPort();
        return;
    }
}

#define RECEIVE_TIMEOUT_MS 100

RobotMessage RobotLinkReal::Receive()
{
    if (_comPort == INVALID_HANDLE_VALUE)
    {
        SAFE_DRAW
        cv::putText(drawingImage, "Failed COM!", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        END_SAFE_DRAW

        // attempt to reopen
        _InitComPort();
        return RobotMessage{RobotMessageType::INVALID};
    }

    _receiver.update();

    RobotMessage retrievedStruct = _receiver.getLatestData();

    // if latest data is valid
    if (_receiver.isLatestDataValid())
    {
        // call super method to update clock
        IRobotLink::Receive();
    }
    else
    {
        return RobotMessage{RobotMessageType::INVALID};
    }

    return retrievedStruct;
}

RobotLinkReal::~RobotLinkReal()
{
    CloseHandle(_comPort);
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

#define ACCELEROMETER_TO_PX_SCALER 1

RobotMessage RobotLinkSim::Receive()
{
    std::string received = "";
    while (received == "")
    {
        received = serverSocket.receive();
    }
    UnityRobotState message = RobotStateParser::parse(received);

    RobotMessage ret{RobotMessageType::IMU_DATA};
    ret.imuData.rotation = message.robot_orientation;
    ret.imuData.rotationVelocity = message.robot_rotation_velocity;

    // call super method to update clock
    IRobotLink::Receive();

    // return the message
    return ret;
}