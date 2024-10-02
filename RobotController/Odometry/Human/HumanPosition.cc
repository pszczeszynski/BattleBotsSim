#include <math.h>
#include "HumanPosition.h"
#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"
#include "../../RobotOdometry.h"
#include "../../RobotController.h"
#include <iostream>

HumanPosition::HumanPosition(ICameraReceiver *videoSource) : OdometryBase(videoSource)
{
    for (std::string port: {"11117", "11118", "11119"})
    {
        _sockets.push_back({new ServerSocket(port), port, {}});
    }
}

void HumanPosition::_GetDataFromSocket()
{
    for (socket_data_t socket_data : _sockets)
    {
        int error = 0;
        socket_data._data.clear();

        // Try to eceive 12 bytes from the socket
        std::string data_str = server_socket->receive(&error);
        if (data_str.size() == 12)
        {
            int mode = *(int*)data_str.c_str();
            int x = *(int*)(data_str.c_str() + 4);
            int y = *(int*)(data_str.c_str() + 8);

            if (x != 0 && y != 0)
            {
                socket_data._data.push_back(mode);
                socket_data._data.push_back(x);
                socket_data._data.push_back(y);
            }
        }
        if (error != 0)
        {
            delete socket_data._socket;
            socket_data._socket = new ServerSocket(socket_data._port);
        }

    }
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
    uint32_t robotPosX = htonl(robotPos.x / 2);
    uint32_t robotPosY = htonl(robotPos.y / 2);
    cv::Point2f opponentPos = RobotController::GetInstance().odometry.Opponent().robotPosition;
    uint32_t opponentPosX = htonl(opponentPos.x / 2);
    uint32_t opponentPosY = htonl(opponentPos.y / 2);
    double opponentAngle = RobotController::GetInstance().odometry.Opponent().robotAngle;
    // enforce 0 to 360
    while (opponentAngle < 0)
    {
        opponentAngle += 2 * M_PI;
    }
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

    _GetDataFromSocket();

    for (socket_data_t socket_data: _sockets)
    {
        if (socket_data._data.size() == 3)
        {
            cv::Point2f clickPosition(socket_data._data[1], socket_data._data[2]);
            clickPosition *= 2; // since image is scaled down by 2
            DataType type = (DataType)socket_data._data[0];

            if (clickPosition == _lastReceivedPos && type == _lastReceivedType && type != DataType::OPPONENT_ANGLE_VEL)
            {
                return;
            }

            std::cout << "Received data: " << socket_data._data[0] << " " << socket_data._data[1] << " " << socket_data._data[2] << std::endl;

            if (type == DataType::ROBOT_POSITION)
            {
                _UpdateData(true, frameTime, &clickPosition, nullptr);
            }
            else if (type == DataType::OPPONENT_POSITION)
            {
                // call set position on blob + heuristic
                RobotController::GetInstance().odometry.GetBlobOdometry().SetPosition(clickPosition, true);
                RobotController::GetInstance().odometry.GetHeuristicOdometry().SetPosition(clickPosition, true);
                // _UpdateData(false, frameTime, &clickPosition, nullptr);
            }
            else if (type == DataType::OPPONENT_ANGLE)
            {
                double MIDDLE = WIDTH / 2;
                Angle angle = Angle(atan2(clickPosition.y - MIDDLE, clickPosition.x - MIDDLE));
                _UpdateData(false, frameTime, nullptr, &angle);
            }
            else if (type == DataType::OPPONENT_ANGLE_VEL)
            {
                double MIDDLE = WIDTH / 2;

                double angle_vel = (clickPosition.x - MIDDLE) / MIDDLE;
                // square
                angle_vel *= abs(angle_vel);
                const double MAX_ADJUST_SPEED = 2 * M_PI;
                angle_vel *= MAX_ADJUST_SPEED;

                _UpdateData(false, frameTime, nullptr, nullptr, &angle_vel);
            }

            _lastReceivedPos = clickPosition;
            _lastReceivedType = (DataType)data[0];
        }
    }
}

void HumanPosition::_UpdateData(bool isUs, double time, cv::Point2f* pos, Angle* angle, double* angle_vel)
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

        if (angle_vel != nullptr)
        {
            _currDataOpponent.robotAngleVelocity = *angle_vel;
            // increment angle
            double currOpponentAngle = RobotController::GetInstance().odometry.Opponent().robotAngle;

            // read delta time
            double currTime = Clock::programClock.getElapsedTime();
            double elapsedTime = currTime - _lastAngleVelUpdateTimeSec;
            _lastAngleVelUpdateTimeSec = currTime;

            if (elapsedTime < 0.07)
            {
                _currDataOpponent.robotAngle = Angle(currOpponentAngle + *angle_vel * elapsedTime);
                _currDataOpponent.robotAngleValid = true;
            }
        }
    }

    _updateMutex.unlock();
}