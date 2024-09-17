#include <math.h>
#include "HumanPosition.h"
#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"
#include <iostream>

HumanPosition::HumanPosition(ICameraReceiver *videoSource) : OdometryBase(videoSource), _socket("11118")
{
}

std::vector<int> HumanPosition::_GetDataFromSocket()
{
    std::vector<int> data = {};

    std::cout << "receiving" << std::endl;
    // Receive 12 bytes from the socket
    std::string data_str = _socket.receive();
    std::cout << "recieved data" << data_str << std::endl;

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
    std::cout << "in _ProcessNewFrame" << std::endl;
    // convert mat to std::string
    // Encode the image
    std::vector<uchar> buf;
    cv::Mat scaled = currFrame;
    // scale down by 4x.
    cv::resize(currFrame, scaled, cv::Size(WIDTH / 2, HEIGHT / 2), 0, 0, cv::INTER_LINEAR);

    cv::imencode(".jpg", scaled, buf);
    std::string image_string(buf.begin(), buf.end());

    // Get the image size
    uint32_t image_size = image_string.size();
    uint32_t net_image_size = htonl(image_size); // Convert to big-endian (network byte order)

    // Create a message buffer to hold the size and the image data
    std::string message;
    message.resize(4 + image_string.size());

    // Copy the size into the message buffer
    memcpy(&message[0], &net_image_size, 4);

    // Copy the image data into the message buffer
    memcpy(&message[4], image_string.data(), image_string.size());

    // send the image back
    _socket.reply_to_last_sender(message);

    std::vector<int> data = _GetDataFromSocket();

    if (data.size() == 3)
    {
        if (data[0] == (int) DataType::ROBOT_POSITION)
        {
            cv::Point2f newPos(data[1], data[2]);
            _UpdateData(true, frameTime, &newPos, nullptr);
        }
        else if (data[0] == (int) DataType::OPPONENT_POSITION)
        {
            cv::Point2f newPos(data[1], data[2]);
            _UpdateData(false, frameTime, &newPos, nullptr);
        }
        else if (data[0] == (int) DataType::OPPONENT_ANGLE)
        {
            const double MIDDLE = WIDTH / 2;
            Angle angle = Angle(atan2(data[1] - MIDDLE, data[2] - MIDDLE));
            _UpdateData(false, frameTime, nullptr, &angle);
        }
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
        else if (angle != nullptr)
        {
            _currDataRobot.robotAngleValid = true;
            _currDataRobot.robotAngle = *angle;
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
        else if (angle != nullptr)
        {
            _currDataOpponent.robotAngleValid = true;
            _currDataOpponent.robotAngle = *angle;
        }
    }

    _updateMutex.unlock();
}