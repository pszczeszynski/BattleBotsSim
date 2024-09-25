#pragma once

#include "../OdometryBase.h"
#include "../../ServerSocket.h"
#include "../../CameraReceiver.h"
#include <opencv2/opencv.hpp>

enum class DataType
{
    ROBOT_POSITION,
    OPPONENT_POSITION,
    OPPONENT_ANGLE
};

class HumanPosition : public OdometryBase
{
public:
    HumanPosition(ICameraReceiver *videoSource);

protected:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime);
private:
    ServerSocket* _socket;
    std::vector<int> _GetDataFromSocket();
    void _UpdateData(bool isUs, double time, cv::Point2f * pos, Angle * angle);
};
