#pragma once

#include "../OdometryBase.h"
#include "../../ServerSocket.h"
#include "../../CameraReceiver.h"
#include <opencv2/opencv.hpp>

enum class DataType
{
    ROBOT_POSITION,
    OPPONENT_POSITION,
    OPPONENT_ANGLE,
    OPPONENT_ANGLE_VEL
};

class HumanPosition : public OdometryBase
{
public:
    HumanPosition(ICameraReceiver *videoSource);

protected:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime);
private:
    cv::Point2f _lastReceivedPos;
    DataType _lastReceivedType;
    ServerSocket* _socket;
    std::vector<int> _GetDataFromSocket();
    void _UpdateData(bool isUs, double time, cv::Point2f * pos, Angle * angle, double* angle_vel = nullptr);

    double _lastAngleVelUpdateTimeSec = 0;
};
