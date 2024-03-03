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

            std::string data = _socket.receive();

            cv::Mat image;
            // get the next frame
            last_id = camera.GetFrame(image, last_id);

            // convert mat to std::string
            std::vector<uchar> buf;
            cv::imencode(".jpg", image, buf);
            std::string image_string(buf.begin(), buf.end());

            // send the image back
            _socket.reply_to_last_sender(image_string);

            // get the latest frame
            if (camera.NewFrameReady(-1))
            {
                cv::Mat frame;
                camera.GetFrame(frame, -1);
                cv::imwrite("frame.jpg", frame);
            }
        }
    });
}

cv::Point2f HumanPosition::GetPosition()
{
    return cv::Point2f(0, 0);
}
