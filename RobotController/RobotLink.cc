#include "RobotLink.h"
#include "Clock.h"
#include "../Communication/Communication.h"
#include "Globals.h"
#include "MathUtils.h"
#include "RobotConfig.h"

/**
 * Doesn't do anything except call the timer markStart
 * Must be called for any robotlink
*/
RobotMessage IRobotLink::Receive()
{
    static int valid_count = 0;
    RobotMessage ret = _ReceiveImpl();

    if (ret.type == RobotMessageType::IMU_DATA)
    {
        _lastIMUMessage = ret;
    }
    else if (ret.type == RobotMessageType::CAN_DATA)
    {
        _lastCANMessage = ret;
    }
    else if (ret.type == RobotMessageType::INVALID)
    {
        // _ReceiveImpl returned invalid message type
        return ret;
    }
    else
    {
        // invalid message type from _ReceiveImpl
        std::cerr << "ERROR: invalid message type" << std::endl;
        return ret;
    }

    valid_count++;
    if (valid_count % 100 == 0)
    {
        std::cout << "valid count: " << valid_count << std::endl;
    }

    // valid message set receive delay
    ret.receiveDelay = _receiveClock.getElapsedTime();
    // add to message history
    _messageHistory.push_back(ret);

    // if message history is too long, remove the first element
    if (_messageHistory.size() > MESSAGE_HISTORY_SIZE)
    {
        _messageHistory.pop_front();
    }

    // restart clock
    _receiveClock.markStart();

    return ret;
}

RobotMessage IRobotLink::GetLastIMUMessage()
{
    return _lastIMUMessage;
}

RobotMessage IRobotLink::GetLastCANMessage()
{
    return _lastCANMessage;
}

const std::deque<RobotMessage> &IRobotLink::GetMessageHistory()
{
    return _messageHistory;
}

#define TRANSMITTER_COM_PORT TEXT("COM7")

#define COM_READ_TIMEOUT_MS 100
#define COM_WRITE_TIMEOUT_MS 100


int total_count = 0;

#define SERIAL_BUFFER_SIZE 500
bool readComPort(HANDLE comPort, char &receivedChar)
{
    static DWORD totalBytesRead = 0;
    static DWORD totalStartTime = GetTickCount(); // Start time for overall tracking
    static DWORD lastReportTime = totalStartTime;

    DWORD startTime = GetTickCount(); // Start time for this read
    DWORD dwBytesRead = 0;
    DWORD dwErrors = 0;
    COMSTAT comStat;

    Clock clearCommErrTime;
    if (!ClearCommError(comPort, &dwErrors, &comStat))
    {
        // std::cerr << "COULDNT CLEAR COMM ERR" << std::endl;
        // Handle error
        return false;
    }


    if (comStat.cbInQue == 0)
    {
        // No data to read
        return false;
    }
    // std::cout << "clearCommErrTime: " << clearCommErrTime.getElapsedTime() * 1000 << std::endl;

    // if (comStat.cbInQue > 400)
    // {
        std::cout << std::endl << "comStat.cbInQue: " << comStat.cbInQue << std::endl;
    // }

    // if (comStat.cbInQue > 5)
    // {
    //     comStat.cbInQue = 5;
    // }

    Clock readFileTime;
    // clear entire com buffer
    // clear buffer, move pointer to end
    char buffer[5000];
    ReadFile(comPort, buffer, comStat.cbInQue, &dwBytesRead, NULL);
    receivedChar = 'a';


    // std::cout << "read file time: " << readFileTime.getElapsedTime() * 1000 << "ms" << std::endl;

    return true;




    // Read a single character
    if (!ReadFile(comPort, &receivedChar, 1, &dwBytesRead, NULL) || dwBytesRead == 0)
    {
        // Handle error or no data read
        return false;
    }

    DWORD endTime = GetTickCount(); // End time for this read
    DWORD timeDelta = endTime - startTime;

    // Update total bytes read and time elapsed
    totalBytesRead += dwBytesRead;
    DWORD totalElapsedTime = endTime - totalStartTime;

    // Report every 5 seconds
    if (endTime - lastReportTime >= 5000)
    {
        if (totalElapsedTime > 0)
        {
            double avgBytesPerSec = (totalBytesRead * 1000.0) / totalElapsedTime;
            std::cout << "Average Bytes/sec over " << totalElapsedTime / 1000.0 << " seconds: " << avgBytesPerSec << std::endl;
        }
        lastReportTime = endTime;
    }

    return true;
}

RobotLinkReal::RobotLinkReal() : _receiver(200, [this](char &c)
                                           {
                                            //    static char buffer[SERIAL_BUFFER_SIZE];
                                            //    static int bufferSize = 0;
                                            //    static int bufferIndex = 0;

                                            //    if (bufferIndex >= bufferSize)
                                            //    {
                                                return readComPort(_comPort, c);
                                            //    }

                                            //    if (bufferIndex < bufferSize)
                                            //    {
                                            //        c = buffer[bufferIndex++];
                                            //        return true;
                                            //    }

                                            //    return false;
                                           }),
                                 _serialBuffer(SERIAL_BUFFER_SIZE)
{
    std::cout << "size of robot message: " << sizeof(RobotMessage) << std::endl;
    _InitComPort();
    _sendingClock.markStart();

    _receiverThread = std::thread([this]()
                                  {

        Clock lastMessagePrintClock;
        lastMessagePrintClock.markStart();
        int count = 0;
        while (true)
        {
            Clock updateTimer;

            _receiver.update();
            continue;
            if (updateTimer.getElapsedTime() > 0.02)
            {
                std::cout << "update time: " << updateTimer.getElapsedTime() * 1000 << std::endl;
            }

            // if there is a new message
            if (_receiver.isLatestDataValid())
            {
                Clock lockTimer;
                _lastMessageMutex.lock();

                if (lockTimer.getElapsedTime() > 0.02)
                {
                    std::cout << "lock time: " << lockTimer.getElapsedTime() * 1000 << std::endl;
                }

                _lastMessage = _receiver.getLatestData();
                _lastMessageNew = true;
                _lastMessageMutex.unlock();
                // yield
                // notify anyone waiting on the message
                _lastMessageCV.notify_all();

                // // sleep for 3ms
                // std::this_thread::sleep_for(std::chrono::milliseconds(1));

                count ++;

                if (count % 100 == 0)
                {
                    std::cout << "count: " << count << std::endl;

                    std::cout << "PARSED PACKETS/SEC: " << 100.0 / lastMessagePrintClock.getElapsedTime() << std::endl;
                    lastMessagePrintClock.markStart();
                }
            }
        }
    });
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
    _dcbSerialParams.BaudRate = 460800; // set the baud rate
    _dcbSerialParams.ByteSize = 8;
    _dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    _dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    
    _dcbSerialParams.StopBits = ONESTOPBIT;
    _dcbSerialParams.Parity = NOPARITY;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = COM_READ_TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = COM_WRITE_TIMEOUT_MS;


    if (!SetupComm(_comPort, sizeof(RobotMessage) * 3, sizeof(RobotMessage) * 3)) { // Smaller buffer sizes
        std::cerr << "Error setting COM buffer sizes" << std::endl;
    }


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

void RobotLinkReal::Drive(DriveCommand &command)
{
    return;


    // std::cout << "command.movement: " << command.movement << std::endl;
    command.movement *= -1.0;
    command.turn *= -1.0;
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

        _sendClock.markStart();
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

RobotMessage RobotLinkReal::_ReceiveImpl()
{
    // if (_comPort == INVALID_HANDLE_VALUE)
    // {
    //     SAFE_DRAW
    //     cv::putText(drawingImage, "Failed COM!", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    //     END_SAFE_DRAW

    //     // attempt to reopen
    //     _InitComPort();
    //     return RobotMessage{RobotMessageType::INVALID};
    // }

    return RobotMessage{RobotMessageType::INVALID};

    // this is the lock that will be used to wait for a new message
    static std::unique_lock<std::mutex> lock(_lastMessageMutex);
    if (!_lastMessageNew)
    {
        // TODO: add timeout and return invalid if timeout
        // wait for a new message
        _lastMessageCV.wait(lock, [&]{ return _lastMessageNew; });
    }

    // from here on, we know that _lastMessageNew is true

    // // if latest data is valid
    // if (!_lastMessageNew)
    // {
    //     // unlock the mutex
    //     _lastMessageMutex.unlock();
    //     // force invalid since it's not a new message
    //     return RobotMessage{RobotMessageType::INVALID};
    // }

    // copy over the latest data
    RobotMessage retrievedStruct = _lastMessage;
    // mark as consumed
    _lastMessageNew = false;
    // unlock the mutex
    // _lastMessageMutex.unlock();

    // TODO: deleteme, set type as imu_data
    retrievedStruct.type = RobotMessageType::IMU_DATA;

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

RobotMessage RobotLinkSim::_ReceiveImpl()
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

    // return the message
    return ret;
}

