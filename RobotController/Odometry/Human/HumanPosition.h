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
    HumanPosition(ICameraReceiver *videoSource, std::string port, bool sendHeuristicMat = false);

protected:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override; 
private:
    std::string _port;
    bool _sendHeuristicMat = false;

    cv::Point2f _lastReceivedPos;
    int _lastAutoLCount = 0;
    int _lastAutoRCount = 0;
    int _lastHardRebootCount = 0;
    int _lastRebootRecoveryCount = 0;
    int _lastLoadStartCount = 0;
    int _lastLoadSavedCount = 0;
    int _lastSavedCount = 0;

    SOCKADDR last_sender_addr;
    int last_sender_addr_len;

    DataType _lastReceivedType;
    ServerSocket* _socket;
    std::vector<int> _GetDataFromSocket();
    void _UpdateData(bool isUs, double time, cv::Point2f *pos, Angle *angle);

    double _lastAngleVelUpdateTimeSec = 0;
};
