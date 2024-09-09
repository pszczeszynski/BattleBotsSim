#pragma once

#include "../OdometryBase.h"
#include "../../ServerSocket.h"
#include "../../CameraReceiver.h"
#include <opencv2/opencv.hpp>


class HumanPosition : public OdometryBase
{
public:
    HumanPosition();
    cv::Point2f GetPosition();
    float GetAngle();
private:
    ServerSocket _socket;
    std::thread _socketThread;

    cv2::Point2f _lastPos;
    std::mutex _lastPosMutex;

    float _lastAngle;
    std::mutex _lastAngleMutex;

    std::vector<int> _GetDataFromSocket();
}