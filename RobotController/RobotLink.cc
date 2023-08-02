#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"
#include "Globals.h"
#include "MathUtils.h"

#define TRANSMITTER_COM_PORT TEXT("COM6")

RobotLinkReal::RobotLinkReal() : _receiver(200, [this](char &c)
                                          {
    DWORD dwBytesRead = 0;
    DWORD dwErrors = 0;
    COMSTAT comStat;

    // Get and clear current errors on the com port.
    if (!ClearCommError(_comPort, &dwErrors, &comStat))
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
    if(!ReadFile(_comPort, &c, 1, &dwBytesRead, NULL))
    {
        std::cerr << "Error reading from COM port" << std::endl;
    }
    return dwBytesRead > 0; })
{
    InitComPort();
    _sendingClock.markStart();
}

void RobotLinkReal::InitComPort()
{
    static int numTries = 0;
    const int NUM_TRIES_BEFORE_PRINTING_ERROR = 500;

    _comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (_comPort == INVALID_HANDLE_VALUE)
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

    _dcbSerialParams = {0};
    _dcbSerialParams.DCBlength = sizeof(_dcbSerialParams);
    _dcbSerialParams.BaudRate = CBR_115200; // set the baud rate
    _dcbSerialParams.ByteSize = 8;
    _dcbSerialParams.StopBits = ONESTOPBIT;
    _dcbSerialParams.Parity = NOPARITY;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    if (!SetCommTimeouts(_comPort, &timeouts))
    {
        std::cerr << "Error setting COM port timeouts" << std::endl;
    }

    if (!SetCommState(_comPort, &_dcbSerialParams))
    {
        std::cerr << "Error setting serial port state" << std::endl;
    }
}

#define DRIVE_SCALE 1.0

#define MIN_INTER_SEND_TIME_MS 5
void RobotLinkReal::Drive(DriveCommand &command)
{
    if (_sendingClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS)
    {
        return;
    }

    if (_comPort == INVALID_HANDLE_VALUE)
    {
        // attempt to reopen
        InitComPort();
        return;
    }

    _sendingClock.markStart();

    command.movement = std::max(-1.0, std::min(1.0, command.movement));
    command.turn = std::max(-1.0, std::min(1.0, command.turn));

    // scale down
    command.movement *= DRIVE_SCALE;
    command.turn *= DRIVE_SCALE;

    command.valid = true;

    // write start MESSAGE_START_CHAR
    DWORD dwBytesWritten = 0;
    char start = MESSAGE_START_CHAR;
    WriteFile(_comPort, &start, sizeof(start), &dwBytesWritten, NULL);
    // write command
    WriteFile(_comPort, &command, sizeof(command), &dwBytesWritten, NULL);

    // write start MESSAGE_END_CHAR
    char end = MESSAGE_END_CHAR;
    WriteFile(_comPort, &end, sizeof(end), &dwBytesWritten, NULL);
}



#define RECEIVE_TIMEOUT_MS 100

RobotMessage RobotLinkReal::Receive()
{
    if (_comPort == INVALID_HANDLE_VALUE)
    {
        SAFE_DRAW
        cv::putText(drawingImage, "Failed COM!", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
        END_SAFE_DRAW

        // attempt to reopen
        InitComPort();
        return RobotMessage{0};
    }

    _receiver.update();

    RobotMessage retrievedStruct = _receiver.getLatestData();

    // if latest data is valid
    if (_receiver.isLatestDataValid())
    {
        // mark that we have received a packet
        _receivedPackets++;
        _lastReceivedTimer.markStart();
        _receiveFPS = _receivedPackets / _fpsTimer.getElapsedTime();
    }

    // if we have not received a packet in a while
    if (_fpsTimer.getElapsedTime() > 1)
    {
        _receivedPackets = 0;
        _fpsTimer.markStart();
    }

    SAFE_DRAW
    cv::putText(drawingImage, "Packets/Sec: " + std::to_string(_receiveFPS),
        cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.9), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    END_SAFE_DRAW

    if (_lastReceivedTimer.getElapsedTime() * 1000 < 50)
    {
        SAFE_DRAW
        cv::putText(drawingImage, "Connected", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        END_SAFE_DRAW
    }
    else
    {
        SAFE_DRAW
        cv::putText(drawingImage, "Disconnected", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
        END_SAFE_DRAW
        _receiveFPS = 0;
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