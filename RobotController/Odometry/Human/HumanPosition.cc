#include <math.h>
#include "HumanPosition.h"
#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"
#include "../../RobotOdometry.h"
#include "../../RobotController.h"
#include "../../UIWidgets/ConfigWidget.h"
#include <iostream>

HumanPosition::HumanPosition(ICameraReceiver *videoSource, std::string port, bool sendHeuristicMat) : OdometryBase(videoSource)
{
    _port = port;
    _sendHeuristicMat = sendHeuristicMat;
    _socket = new ServerSocket(port);
}

const int NUMBER_OF_FIELDS = 11;
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
        _socket = new ServerSocket(_port);
    }

    if (data_str.size() != NUMBER_OF_FIELDS * 4)
    {
        return data;
    }

    // parse the 11 values to an int
    for (int i = 0; i < NUMBER_OF_FIELDS; i ++)
    {
        data.push_back(*(int*)(data_str.c_str() + i * 4));
    }

    return data;
}

void HumanPosition::_ProcessNewFrameTMP(cv::Mat currFrame, double frameTime)
{
    // convert mat to std::string
    // Encode the image
    std::vector<uchar> buf;

    cv::Mat& frameToUse = currFrame;

    if (_sendHeuristicMat)
    {
        cv::Mat& heuristicFrame = RobotController::GetInstance().odometry.GetHeuristicOdometry().GetPrevTrackingMatRef();
        if (!heuristicFrame.empty())
        {
            frameToUse = heuristicFrame;
        }
    }

    cv::Mat scaled;
    // scale down by 4x.
    cv::resize(frameToUse, scaled, cv::Size(WIDTH / 2, HEIGHT / 2), 0, 0, cv::INTER_LINEAR);

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
        const int auto_l_count = data[7];
        const int auto_r_count = data[8];
        const int hard_reboot_count = data[9];
        const int reboot_recovery_count = data[10];

        if (_sendHeuristicMat)
        {
            HEU_FOREGROUND_THRESHOLD = foreground_min_delta;
            HEU_BACKGROUND_AVGING = background_heal_rate;
            RobotController::GetInstance().odometry.GetHeuristicOdometry().force_background_averaging = (bool)force_heal_value;
        }

        if (clickPosition == _lastReceivedPos && type == _lastReceivedType &&
            auto_l_count == _lastAutoLCount && auto_r_count == _lastAutoRCount &&
            hard_reboot_count == _lastHardRebootCount && reboot_recovery_count == _lastRebootRecoveryCount)
        {
            return;
        }

        BlobDetection& blob = RobotController::GetInstance().odometry.GetBlobOdometry();
        HeuristicOdometry& heuristic = RobotController::GetInstance().odometry.GetHeuristicOdometry();


        if (type == DataType::ROBOT_POSITION)
        {
            blob.SetPosition(clickPosition, false);
            if (force_pos_bool)
            {
                heuristic.ForcePosition(clickPosition, false);
            }
            else
            {
                heuristic.SetPosition(clickPosition, false);
            }
        }
        else if (type == DataType::OPPONENT_POSITION)
        {
            // call set position on blob + heuristic
            blob.SetPosition(clickPosition, true);
            if (force_pos_bool)
            {
                heuristic.ForcePosition(clickPosition, true);
            }
            else
            {
                heuristic.SetPosition(clickPosition, true);
            }
        }
        else if (type == DataType::OPPONENT_ANGLE)
        {
            double MIDDLE = WIDTH / 2;
            Angle angle = Angle(atan2(clickPosition.y - MIDDLE, clickPosition.x - MIDDLE));
            _UpdateData(false, frameTime, nullptr, &angle);
        }

        if (auto_l_count > _lastAutoLCount)
        {
            heuristic.MatchStart(ConfigWidget::leftStart, ConfigWidget::rightStart);
        }

        if (auto_r_count > _lastAutoRCount)
        {
            heuristic.MatchStart(ConfigWidget::rightStart, ConfigWidget::leftStart);
        }

        if (hard_reboot_count > _lastHardRebootCount)
        {
            heuristic.set_currFrame_to_bg = true;
        }


        

        

        _lastReceivedPos = clickPosition;
        _lastReceivedType = (DataType)data[0];
        _lastAutoLCount = auto_l_count;
        _lastAutoRCount = auto_r_count;
        _lastHardRebootCount = hard_reboot_count;
        _lastRebootRecoveryCount = reboot_recovery_count;
    }
}

#define MSG_FIELD_SEPERATOR '%'
#define IMG_SCALE 2
void HumanPosition::_ProcessNewFrame(cv::Mat currFrame, double frameTime)
{
    // convert mat to std::string
    // Encode the image
    std::vector<uchar> buf;

    cv::Mat& frameToUse = currFrame;

    if (_sendHeuristicMat)
    {
        cv::Mat& heuristicFrame = RobotController::GetInstance().odometry.GetHeuristicOdometry().GetPrevTrackingMatRef();
        if (!heuristicFrame.empty())
        {
            frameToUse = heuristicFrame;
        }
    }

    // Scale down image
    cv::Mat scaled;
    cv::resize(frameToUse, scaled, cv::Size(WIDTH / IMG_SCALE, HEIGHT / IMG_SCALE), 0, 0, cv::INTER_LINEAR);

    cv::imencode(".jpg", scaled, buf);
    std::string image_string(buf.begin(), buf.end());

    // ************************************
    // ********* SEND DATA ****************
    // DATA ORDER:
    // 1) Number of Fields (not including this field)
    // 2,3) Robot X,Y
    // 4,5) Opponent X,Y
    // 6) Opponent Angle
    // 7) Image Size
    // 8) Image Data
    
    // Create a message buffer 
    std::string message;

    int numOfFields = 0;

    // Add robot and opponent stuff
    cv::Point2f robotPos = RobotController::GetInstance().odometry.Robot().robotPosition;
    cv::Point2i  scaled_robotpos = robotPos / IMG_SCALE;

    message += std::to_string(scaled_robotpos.x) + MSG_FIELD_SEPERATOR + std::to_string(scaled_robotpos.y) + MSG_FIELD_SEPERATOR;
    numOfFields += 2;

    cv::Point2f opponentPos = RobotController::GetInstance().odometry.Opponent().robotPosition;
    cv::Point2i scaled_opponentpos = opponentPos / IMG_SCALE;

    message += std::to_string(scaled_opponentpos.x) + MSG_FIELD_SEPERATOR + std::to_string(scaled_opponentpos.y) + MSG_FIELD_SEPERATOR;
    numOfFields += 2;


    double opponentAngle = RobotController::GetInstance().odometry.Opponent().robotAngle;
    // enforce 0 to 360
    while (opponentAngle < 0)
    {
        opponentAngle += 2 * M_PI;
    }

    message += std::to_string((int) (opponentAngle * 180 / M_PI)) + MSG_FIELD_SEPERATOR;
    numOfFields++;
    
    uint32_t image_size = htonl(image_string.size()); // Convert to big-endian (network byte order)
    numOfFields++;
   
    // Copy the image data into the message buffer
    message += image_string;
    message = std::to_string(numOfFields) + MSG_FIELD_SEPERATOR + message;
 
    // send the image back
    _socket->reply_to_last_sender(message);

    // **************************************
    // ******** RECEIVE DATA ****************
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
        const int auto_l_count = data[7];
        const int auto_r_count = data[8];
        const int hard_reboot_count = data[9];
        const int reboot_recovery_count = data[10];

        if (_sendHeuristicMat)
        {
            HEU_FOREGROUND_THRESHOLD = foreground_min_delta;
            HEU_BACKGROUND_AVGING = background_heal_rate;
            RobotController::GetInstance().odometry.GetHeuristicOdometry().force_background_averaging = (bool)force_heal_value;
        }

        if (clickPosition == _lastReceivedPos && type == _lastReceivedType &&
            auto_l_count == _lastAutoLCount && auto_r_count == _lastAutoRCount &&
            hard_reboot_count == _lastHardRebootCount && reboot_recovery_count == _lastRebootRecoveryCount)
        {
            return;
        }

        BlobDetection& blob = RobotController::GetInstance().odometry.GetBlobOdometry();
        HeuristicOdometry& heuristic = RobotController::GetInstance().odometry.GetHeuristicOdometry();


        if (type == DataType::ROBOT_POSITION)
        {
            blob.SetPosition(clickPosition, false);
            if (force_pos_bool)
            {
                heuristic.ForcePosition(clickPosition, false);
            }
            else
            {
                heuristic.SetPosition(clickPosition, false);
            }
        }
        else if (type == DataType::OPPONENT_POSITION)
        {
            // call set position on blob + heuristic
            blob.SetPosition(clickPosition, true);
            if (force_pos_bool)
            {
                heuristic.ForcePosition(clickPosition, true);
            }
            else
            {
                heuristic.SetPosition(clickPosition, true);
            }
        }
        else if (type == DataType::OPPONENT_ANGLE)
        {
            double MIDDLE = WIDTH / 2;
            Angle angle = Angle(atan2(clickPosition.y - MIDDLE, clickPosition.x - MIDDLE));
            _UpdateData(false, frameTime, nullptr, &angle);
        }

        if (auto_l_count > _lastAutoLCount)
        {
            heuristic.MatchStart(ConfigWidget::leftStart, ConfigWidget::rightStart);
        }

        if (auto_r_count > _lastAutoRCount)
        {
            heuristic.MatchStart(ConfigWidget::rightStart, ConfigWidget::leftStart);
        }

        if (hard_reboot_count > _lastHardRebootCount)
        {
            heuristic.set_currFrame_to_bg = true;
        }


        

        

        _lastReceivedPos = clickPosition;
        _lastReceivedType = (DataType)data[0];
        _lastAutoLCount = auto_l_count;
        _lastAutoRCount = auto_r_count;
        _lastHardRebootCount = hard_reboot_count;
        _lastRebootRecoveryCount = reboot_recovery_count;
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