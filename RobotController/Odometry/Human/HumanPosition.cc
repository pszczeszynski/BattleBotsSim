#include <math.h>
#include "HumanPosition.h"
#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"

HumanPosition::HumanPosition(ICameraReceiver *videoSource) : _socket("11117"), OdometryBase(videoSource)
{
    _socketThread = std::thread([this]() {
        ICameraReceiver& camera = ICameraReceiver::GetInstance();
        long last_id = -1;
        while (true)
        {
            // spin until new frame is ready
            if (!camera.NewFrameReady(last_id))
            {
                continue;
            }

            // get the next frame
            cv::Mat image;
            last_id = camera.GetFrame(image, last_id);

            // convert mat to std::string
            std::vector<uchar> buf;
            cv::imencode(".jpg", image, buf);
            std::string image_string(buf.begin(), buf.end());

            // send the image back
            _socket.reply_to_last_sender(image_string);

            std::vector<int> data = _GetDataFromSocket();

            if (data && data.size() == 3)
            {
                if (data[0] == 0)
                {
                    cv::Point2f newPos(data[1], data[2]);
                    _updateMutex.lock();
                    _currDataRobot.robotPosValid = true;
                    _currDataRobot.robotPosition = newPos;
                    _updateMutex.unlock();
                }
                else if (data[0] == 1)
                {
                    cv::Point2f newPos(data[1], data[2]);
                    _updateMutex.lock();
                    _currDataOpponent.robotPosValid = true;
                    _currDataOpponent.robotPosition = newPos;
                    _updateMutex.unlock();
                }
                else if (data[0] == 2)
                {
                    float angle = tan(abs(data[1] - 360) / abs(data[2] - 360));
                    if (data[1] < 360 and data[2] < 360) {
                        angle = M_PI - angle;
                    }
                    if (data[1] < 360 and data[2] > 360) {
                        angle = M_PI + angle;
                    }
                    if (data[1] > 360 and data[2] > 360) {
                        angle = 2 * M_PI - angle;
                    }
                    _updateMutex.lock();
                    _currDataOpponent.robotAngleValid = true;
                    _currDataOpponent.robotAngle = Angle(angle);
                }
            }
        }
    });
}

cv::Point2f HumanPosition::_GetDataFromSocket()
{
    std::vector<int> data;

    // Receive 12 bytes from the socket
    std::string data = _socket.receive(12);

    int mode = *(int*)data.c_str();
    int x = *(int*)(data.c_str() + 4);
    int y = *(int*)(data.c_str() + 8);

    if (x != 0 && y != 0)
    {
        data.push_back(mode);
        data.push_back(x);
        data.push_back(y);
    }
    return NULL;
}
