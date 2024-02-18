#pragma once
#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "Vision.h"
#include "RobotOdometry.h"
#include "RobotLink.h"
#include "Input/Gamepad.h"
#include "../Communication/Communication.h"
#include "Extrapolator.h"
#include "MatQueue.h"
#include "SelfRighter.h"
#include "MovementStrategy.h"
#include "Weapons.h"

// Modes of operation:
// SIMULATION - use Unity output data, this define takes precedence
// VIDEO_READ - If not simulation and this is defined, use video files
// ELSE - use real robot hardware

#ifndef SIMULATION
    // Define VIDEO_READ in build command line using the VIDEO option
    // #define VIDEO_READ
#endif 

class RobotController
{
public:
    RobotController();
    static RobotController& GetInstance();

    IMUData& GetIMUData();
    CANData GetCANData();

    IRobotLink& GetRobotLink();

    cv::Mat& GetDrawingImage();
    Gamepad& GetGamepad();

    Clock visionClock;

    void Run();
private:
    cv::Point2f _NoMoreAggressiveThanTangent(cv::Point2f ourPosition, cv::Point2f opponentPosEx, double orbitRadius, cv::Point2f currentTargetPoint, bool circleDirection);

    DriveCommand RobotLogic();
    DriveCommand OrbitMode();
    DriveCommand AvoidMode();
    DriveCommand ManualMode();
    DriveCommand DriveToPosition(const cv::Point2f& targetPos, bool chooseNewTarget);
    void UpdateRobotTrackers(VisionClassification classification);

    cv::Mat drawingImage;

    MovementStrategy _movementStrategy;

public:
    XBox gamepad;
private:

    RobotMessage _lastIMUMessage;
    RobotMessage _lastCANMessage;
    std::mutex _lastCanMessageMutex;

#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    RobotLinkSim robotLink;
#elif defined(VIDEO_FILES)
    CameraReceiverVideo overheadCamL_video;
    RobotLinkReal robotLink;
#else 
    CameraReceiver overheadCamL_real;
    RobotLinkReal robotLink;
#endif

    Vision vision;

    SelfRighter _selfRighter;

    bool _orbiting = false;
    bool _killing = false;
};
