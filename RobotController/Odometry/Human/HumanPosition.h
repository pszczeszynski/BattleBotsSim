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
private:
    ServerSocket _socket;
    std::thread _socketThread;

    cv2::Point2f _lastPos;
    cv2::Mutex _lastPosMutex;

    cv2:Point2f _GetDataFromSocket();
}