#pragma once

#include "../OdometryBase.h"
#include "../../ServerSocket.h"
#include "../../CameraReceiver.h"


class HumanPosition : public OdometryBase
{
public:
    HumanPosition();
    cv::Point2f GetPosition();
private:
    ServerSocket _socket;
    std::thread _socketThread;
};