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

            cv::Point2f newPos = GetPosition()

            if (newPos != NULL)
            {
                _lastPosMutex.lock();
                _lastPos = newPos;
                _lastPosMutex.unlock();
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

cv::Point2f HumanPosition::_GetDataFromSocket()
{
    // Receive 8 bytes from the socket
    std::string data = _socket.receive(8);

    // int x is the first 4 bytes
    int x = *(int*)data.c_str();

    // int y is the next 4 bytes
    int y = *(int*)(data.c_str() + 4);

    if (x != 0 && y != 0)
    {
        return cv::Point2f(x, y);
    }
    return NULL;
}
