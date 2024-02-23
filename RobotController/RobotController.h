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
    long GetIMUFrame(IMUData &output, long old_id, double* frameTime);
    CANData GetCANData();

    IRobotLink& GetRobotLink();

    cv::Mat& GetDrawingImage(); // Returns the image for the overlays
    cv::Mat GetFinalImageCopy(); // Returns the final image (background + overlays)
    Gamepad& GetGamepad();

    Clock visionClock;
    RobotOdometry odometry;

    void Run();
private:
    cv::Point2f _NoMoreAggressiveThanTangent(cv::Point2f ourPosition, cv::Point2f opponentPosEx, double orbitRadius, cv::Point2f currentTargetPoint, bool circleDirection);

    DriveCommand RobotLogic();
    DriveCommand OrbitMode();
    DriveCommand AvoidMode();
    DriveCommand ManualMode();
    DriveCommand DriveToPosition(const cv::Point2f& targetPos, bool chooseNewTarget);
   
    // Not used at the moment
    void UpdateRobotTrackers();

    std::mutex _imageLock;
    cv::Mat drawingImage;
    cv::Mat latestVideoImage;
    cv::Mat latestOverlay;

    MovementStrategy _movementStrategy;

public:
    XBox gamepad;
private:

    // IMU DATA
    // Mutex and CV to allow odometry to independently poll this
    std::mutex _imudataMutex;
    std::condition_variable _imuCV;
    RobotMessage _lastIMUMessage;
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
};
