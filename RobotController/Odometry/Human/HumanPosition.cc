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

const int NUMBER_OF_FIELDS = 16;
std::vector<int> HumanPosition::_GetDataFromSocket()
{
    std::vector<int> data = {};

    int error = 0;

    // saved lastSenderAddr and length
    _socket->last_sender_addr = last_sender_addr;
    _socket->last_sender_addr_len = last_sender_addr_len;

    // Receive NUMBER_OF_FIELDS data
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


#define MSG_FIELD_SEPERATOR '%'
#define MSG_BINARY_SEPERATOR '$'
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
    // 1,2) Robot X,Y
    // 3,4) Opponent X,Y
    // 5) Opponent Angle
    // 6)  HEU_FOREGROUND_THRESHOLD
    // Last) Image Size
    // $ 
    // Image Data
    
    
    // Create a message buffer 
    std::string message = std::to_string(11115) + MSG_FIELD_SEPERATOR;

    int numOfFields = 1;

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

    message += std::to_string(HEU_FOREGROUND_THRESHOLD) + MSG_FIELD_SEPERATOR;
    numOfFields++;
    
    message += std::to_string(HEU_UNTRACKED_MOVING_BLOB_AVGING) + MSG_FIELD_SEPERATOR;
    numOfFields++;

    message += std::string((RobotController::GetInstance().odometry.GetHeuristicOdometry().force_background_averaging) ? "1" : "0") + MSG_FIELD_SEPERATOR;
    numOfFields++;

    message += std::to_string(image_string.size()) + MSG_BINARY_SEPERATOR;
    numOfFields++;
   
    // Copy the image data into the message buffer
    message += image_string;
    // message = std::to_string(numOfFields) + MSG_FIELD_SEPERATOR + message;
 
    // send the image back
    _socket->reply_to_last_sender(message);

    // **************************************
    // ******** RECEIVE DATA ****************
    std::vector<int> data = _GetDataFromSocket();
    bool password_ok = false;
    int i = 0;
    if (data.size() == NUMBER_OF_FIELDS)
    {
        int password = data[i++];
        if( password == 11115)
        { password_ok = true;}
    }

    if( password_ok)
    {        
        // save the last sender address
        last_sender_addr = _socket->last_sender_addr;
        last_sender_addr_len = _socket->last_sender_addr_len;
        
        int datatype_index = i;
        DataType type = (DataType)data[i++];
        const int x = data[i++];
        const int y = data[i++];
        cv::Point2f clickPosition(x, y);
        clickPosition *= 2; // since image is scaled down by 2  


        const int foreground_min_delta = data[i++];
        const int background_heal_rate = data[i++];
        const int force_pos_bool = data[i++];
        const int force_heal_value = data[i++];
        const int auto_l_count = data[i++];
        const int auto_r_count = data[i++];
        const int hard_reboot_count = data[i++];
        const int reboot_recovery_count = data[i++];
        const int load_start_count = data[i++];
        const int load_saved_count = data[i++];      
        const int save_count = data[i++];    
        const int new_slider_data = data[i++];

        // *************************************
        // Do button presses
        HeuristicOdometry& heuristic = RobotController::GetInstance().odometry.GetHeuristicOdometry();

        if (auto_l_count > _lastAutoLCount)
        {
            TrackingWidget::GetInstance()->AutoMatchStart(true);
               
        }

        if (auto_r_count > _lastAutoRCount)
        {
            TrackingWidget::GetInstance()->AutoMatchStart(false);

        }


        if (hard_reboot_count > _lastHardRebootCount)
        {
            heuristic.set_currFrame_to_bg = true;
        }

        
        if (load_saved_count > _lastLoadSavedCount)
        {
            heuristic.load_background = true;
        }

        if (save_count > _lastSavedCount)
        {
            heuristic.save_background = true;
        }
        
        _lastAutoLCount = auto_l_count;
        _lastAutoRCount = auto_r_count;
        _lastHardRebootCount = hard_reboot_count;
        _lastRebootRecoveryCount = reboot_recovery_count;
        _lastLoadStartCount = load_start_count;
        _lastLoadSavedCount = load_saved_count;
        _lastSavedCount = save_count;

        //if (_sendHeuristicMat)
        //{
            // Only do if new data available
            if( new_slider_data > 0)
            {
                HEU_FOREGROUND_THRESHOLD = foreground_min_delta;
                HEU_UNTRACKED_MOVING_BLOB_AVGING = background_heal_rate;
                RobotController::GetInstance().odometry.GetHeuristicOdometry().force_background_averaging = (bool)force_heal_value;
            }
        //}

        if (clickPosition == _lastReceivedPos && type == _lastReceivedType )
        {
            return;
        }

        BlobDetection& blob = RobotController::GetInstance().odometry.GetBlobOdometry();


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

 
        _lastReceivedPos = clickPosition;
        _lastReceivedType = (DataType)data[datatype_index];

    }
}

void HumanPosition::_UpdateData(bool isUs, double time, cv::Point2f* pos, Angle* angle, double* angle_vel)
{
    _updateMutex.lock();

    if (isUs)
    {
        _currDataRobot.id++;
        _currDataRobot.frameID = frameID;
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
        _currDataOpponent.frameID = frameID;
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