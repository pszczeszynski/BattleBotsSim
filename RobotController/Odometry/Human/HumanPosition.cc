#include <math.h>
#include "HumanPosition.h"
#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"
#include "../../RobotOdometry.h"
#include "../../RobotController.h"
#include <iostream>

HumanPosition::HumanPosition(ICameraReceiver *videoSource) : OdometryBase(videoSource)
{
    _socket = new ServerSocket("11118");
}

std::vector<int> HumanPosition::_GetDataFromSocket()
{
    std::vector<int> data = {};

    int error = 0;
    // Receive 12 bytes from the socket
    std::string data_str = _socket->receive(&error);

    if (error != 0)
    {
        // call dtor
        delete _socket;
        _socket = new ServerSocket("11118");
    }

    int mode = *(int*)data_str.c_str();
    int x = *(int*)(data_str.c_str() + 4);
    int y = *(int*)(data_str.c_str() + 8);

    if (x != 0 && y != 0)
    {
        data.push_back(mode);
        data.push_back(x);
        data.push_back(y);
    }

    return data;
}

void HumanPosition::_ProcessNewFrame(cv::Mat currFrame, double frameTime)
{
    // convert mat to std::string
    // Encode the image
    std::vector<uchar> buf;
    cv::Mat scaled = currFrame;
    // scale down by 4x.
    cv::resize(currFrame, scaled, cv::Size(WIDTH / 2, HEIGHT / 2), 0, 0, cv::INTER_LINEAR);

    cv::imencode(".jpg", scaled, buf);
    std::string image_string(buf.begin(), buf.end());


    // calculate the size of the message
    int messageSize = 6 * sizeof(uint32_t) + image_string.size();
    int messagePos = 0;

    // Create a message buffer to hold the size and the image data
    std::string message;
    message.resize(messageSize);

    cv::Point2f robotPos = RobotController::GetInstance().odometry.Robot().robotPosition;
    uint32_t robotPosX = htonl(robotPos.x);
    uint32_t robotPosY = htonl(robotPos.y);
    cv::Point2f opponentPos = RobotController::GetInstance().odometry.Opponent().robotPosition;
    uint32_t opponentPosX = htonl(opponentPos.x);
    uint32_t opponentPosY = htonl(opponentPos.y);
    double opponentAngle = RobotController::GetInstance().odometry.Opponent().robotAngle;
    uint32_t opponentAngleDeg = htonl((uint32_t)(opponentAngle * 180 / M_PI));
    uint32_t image_size = htonl(image_string.size()); // Convert to big-endian (network byte order)

    // Copy the robot position into the message buffer
    memcpy(&message[messagePos], &robotPosX, sizeof(uint32_t));
    messagePos += sizeof(uint32_t);
    memcpy(&message[messagePos], &robotPosY, sizeof(uint32_t));
    messagePos += sizeof(uint32_t);
    memcpy(&message[messagePos], &opponentPosX, sizeof(uint32_t));
    messagePos += sizeof(uint32_t);
    memcpy(&message[messagePos], &opponentPosY, sizeof(uint32_t));
    messagePos += sizeof(uint32_t);
    memcpy(&message[messagePos], &opponentAngleDeg, sizeof(uint32_t));
    messagePos += sizeof(uint32_t);
    memcpy(&message[messagePos], &image_size, sizeof(uint32_t));
    messagePos += sizeof(uint32_t);
    // Copy the image data into the message buffer
    memcpy(&message[messagePos], image_string.data(), image_string.size());

    // send the image back
    _socket->reply_to_last_sender(message);

    std::vector<int> data = _GetDataFromSocket();

    if (data.size() == 3)
    {
        cv::Point2f clickPosition(data[1], data[2]);
        DataType type = (DataType)data[0];

        if (clickPosition == _lastReceivedPos && type == _lastReceivedType)
        {
            return;
        }

        std::cout << "Received data: " << data[0] << " " << data[1] << " " << data[2] << std::endl;

        if (type == DataType::ROBOT_POSITION)
        {
            _UpdateData(true, frameTime, &clickPosition, nullptr);
        }
        else if (type == DataType::OPPONENT_POSITION)
        {
            cv::Point2f newPos(data[1], data[2]);
            _UpdateData(false, frameTime, &newPos, nullptr);
        }
        else if (type == DataType::OPPONENT_ANGLE)
        {
            double MIDDLE = WIDTH / 2;
            MIDDLE /= 2; // since image is scaled down by 2
            Angle angle = Angle(atan2(data[2] - MIDDLE, data[1] - MIDDLE));
            _UpdateData(false, frameTime, nullptr, &angle);
        }

        _lastReceivedPos = clickPosition;
        _lastReceivedType = (DataType)data[0];
    }
}

void HumanPosition::_UpdateData(bool isUs, double time, cv::Point2f* pos, Angle* angle)
{
    _updateMutex.lock();

    if (isUs)
    {
        _currDataRobot.id++;
        _currDataRobot.time = time;
        _currDataRobot.Clear(); // doesn't clear id or time
        if (pos != nullptr)
        {
            _currDataRobot.robotPosValid = true;
            _currDataRobot.robotPosition = *pos;
        }
        else
        {
            _currDataRobot.robotPosValid = false;
        }

        if (angle != nullptr)
        {
            _currDataRobot.robotAngleValid = true;
            _currDataRobot.robotAngle = *angle;
        }
        else
        {
            _currDataRobot.robotAngleValid = false;
        }
    }
    else
    {
        _currDataOpponent.id++;
        _currDataOpponent.time = time;
        _currDataOpponent.Clear(); // doesn't clear id or time
        if (pos != nullptr)
        {
            _currDataOpponent.robotPosValid = true;
            _currDataOpponent.robotPosition = *pos;
        }
        else
        {
            _currDataOpponent.robotPosValid = false;
        }
        
        if (angle != nullptr)
        {
            _currDataOpponent.robotAngleValid = true;
            _currDataOpponent.robotAngle = *angle;
        }
        else
        {
            _currDataOpponent.robotAngleValid = false;
        }
    }

    _updateMutex.unlock();
}