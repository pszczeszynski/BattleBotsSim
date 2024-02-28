#pragma once
#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "RobotOdometry.h"
#include "RobotLink.h"
#include "Input/Gamepad.h"
#include "../Communication/Communication.h"
#include "Extrapolator.h"
#include "MatQueue.h"
#include "SelfRighter.h"
#include "MovementStrategy.h"
#include "Strategies/Orbit.h"
#include "Strategies/Kill.h"
#include "Weapons.h"


class RobotController
{
public:
    RobotController();
    static RobotController& GetInstance();

    IMUData& GetIMUData();
    long GetIMUFrame(IMUData &output, long old_id, double* frameTime);
    CANData GetCANData();

    IRobotLink& GetRobotLink();

    cv::Mat& GetDrawingImage(); // Returns the image for the overlays
    Gamepad& GetGamepad();

    Clock visionClock;
    RobotOdometry odometry;

    void Run();
private:
    DriveCommand RobotLogic();
    DriveCommand AvoidMode();
    DriveCommand ManualMode();
    DriveCommand DriveToPosition(const cv::Point2f& targetPos, bool chooseNewTarget);
   
    int UpdateDrawingImage();

    // Not used at the moment
    void UpdateRobotTrackers();

    cv::Mat drawingImage;

    MovementStrategy _movementStrategy;

public:
    XBox gamepad;
private:
    void ApplyMoveScales(DriveCommand& command);
    void DrawStatusIndicators();

    // IMU DATA
    // Mutex and CV to allow odometry to independently poll this
    std::mutex _imudataMutex;
    std::condition_variable _imuCV;
    RobotMessage _lastIMUMessage;
    std::mutex _lastImuMessageMutex;
    
    int _imuID = 0; // Message id tracking
    double _imuTime = 0; // Time of the latest message

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

    ICameraReceiver& videoSource;

    SelfRighter _selfRighter;

    bool _orbiting = false;
    bool _killing = false;

    Orbit orbitMode;
    Kill killMode;

};
