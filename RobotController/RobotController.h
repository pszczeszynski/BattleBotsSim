#pragma once
//#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "Vision.h"
#include "RobotOdometry.h"
//#include "RobotLink.h"
#include "Input/Gamepad.h"
#include "../Communication/Communication.h"
#include "Extrapolator.h"
#include "MatQueue.h"
#include "SelfRighter.h"
class RobotController
{
public:
    RobotController();
    static RobotController& GetInstance();

    IMUData& GetIMUData();
    CANData GetCANData();

    float& GetFrontWeaponTargetPowerRef();
    float& GetBackWeaponTargetPowerRef();

    //IRobotLink& GetRobotLink();

    cv::Mat& GetDrawingImage();

    Clock visionClock;

    void Run();
private:
    double _CalculateOrbitRadius(cv::Point2f opponentPosEx);
    cv::Point2f _NoMoreAggressiveThanTangent(cv::Point2f ourPosition, cv::Point2f opponentPosEx, double orbitRadius, cv::Point2f currentTargetPoint, bool circleDirection);

    DriveCommand RobotLogic();
    DriveCommand OrbitMode();
    DriveCommand ManualMode();
    DriveCommand DriveToPosition(const cv::Point2f& targetPos, bool chooseNewTarget);
    void UpdateRobotTrackers(VisionClassification classification);
    void UpdateSpinnerPowers();

    void GuiLogic();

    cv::Mat drawingImage;

public:
#ifdef XBOX
    XBox gamepad;
#else
    DualSense gamepad;
#endif
private:

    RobotMessage _lastIMUMessage;
    RobotMessage _lastCANMessage;
    std::mutex _lastCanMessageMutex;

    RobotMessageType _lastMessageType;
    RobotSimState exState;

#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    //RobotLinkSim robotLink;
#else
    CameraReceiver overheadCamL_real;
    //RobotLinkReal robotLink;
#endif

    Vision vision;

    SelfRighter _selfRighter;

    float _frontWeaponPower = 0;
    float _backWeaponPower = 0;

    bool _orbiting = false;
    bool _killing = false;
};
