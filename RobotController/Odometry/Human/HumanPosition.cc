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

const int NUMBER_OF_FIELDS = 7;
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

    if (data_str.size() != NUMBER_OF_FIELDS * 4)
    {
        return data;
    }

    // parse the 7 values to an int
    for (int i = 0; i < NUMBER_OF_FIELDS; i ++)
    {
        data.push_back(*(int*)(data_str.c_str() + i * 4));
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

    std::vector<int> data = _GetDataFromSocket();

    if (data.size() == NUMBER_OF_FIELDS)
    {
        DataType type = (DataType)data[0];
        cv::Point2f clickPosition(data[1], data[2]);
        clickPosition *= 2; // since image is scaled down by 2  

        const int foreground_min_delta = data[3];
        const int background_heal_rate = data[4];
        const int force_pos_bool = data[5];
        const int force_heal_value = data[6];

        if (clickPosition == _lastReceivedPos && type == _lastReceivedType && type != DataType::OPPONENT_ANGLE_VEL)
        {
            return;
        }


        std::cout << "foreground_min_delta: " << foreground_min_delta << std::endl;
        std::cout << "background_heal_rate: " << background_heal_rate << std::endl;
        std::cout << "force_pos_bool: " << force_pos_bool << std::endl;
        std::cout << "force_heal_value: " << force_heal_value << std::endl;
        

        std::cout << "Received data: " << data[0] << " " << data[1] << " " << data[2] << std::endl;

        if (type == DataType::ROBOT_POSITION)
        {
            RobotController::GetInstance().odometry.GetBlobOdometry().SetPosition(clickPosition, false);
            RobotController::GetInstance().odometry.GetHeuristicOdometry().SetPosition(clickPosition, false);

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

void HumanPosition::_UpdateData(bool isUs, double time, cv::Point2f* pos, Angle* angle, double* angle_vel)
{
    _updateMutex.lock();

    if (isUs)
    {
        _currDataRobot.id++;
        _currDataRobot.time = time;
        _currDataRobot.time_angle = time;
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
        _currDataOpponent.time_angle = time;
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