#include "HumanPosition.h"

HumanPosition::HumanPosition() : _socket("11117")
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
                if (data[0] == 1)
                {
                    _lastAngleMutex.lock();
                    _lastAngle = tan(data[2]/data[1]);
                    _lastAngleMutex.unlock();
                }
                else
                {
                    cv::Point2f newPos(data[1], data[2]);
                    _lastPosMutex.lock();
                    _lastPos = newPos;
                    _lastPosMutex.unlock();
                }
            }
        }
    });
}

cv::Point2f HumanPosition::GetPosition()
{
    _lastPosMutex.lock();
    cv::Point2f pos = _lastPos;
    _lastPosMutex.unlock();

    return pos;
}

float HumanPosition::GetAngle()
{
    _lastAngleMutex.lock();
    float angle = _lastAngle;
    _lastAngleMutex.unlock();

    return angle;
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
