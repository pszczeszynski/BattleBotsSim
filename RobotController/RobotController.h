#pragma once
#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "RobotOdometry.h"
#include "RobotLink.h"
#include "Input/Gamepad.h"
#include "../Common/Communication.h"
#include "Extrapolator.h"
#include "MatQueue.h"
#include "SelfRighter.h"
#include "Strategies/Orbit.h"
#include "Strategies/Kill.h"
#include "Strategies/AStarAttack.h"
#include "Weapons.h"
#include "UIWidgets/FieldWidget.h"
#include "UIWidgets/TrackingWidget.h"


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
    Gamepad& GetGamepad2();

    void StartForceOrbit();
    void StopForceOrbit();
    void StartForceKill();
    void StopForceKill();

    Clock visionClock;
    RobotOdometry odometry;

    void Run();
    XBox gamepad;
    XBox gamepad2;

    void DumpVideo();

private:
    DriverStationMessage RobotLogic();
    DriverStationMessage ManualMode();

    int UpdateDrawingImage();

    cv::Mat drawingImage;

    void ApplyMoveScales(DriverStationMessage& command);
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
    bool _guiOrbit = false;
    bool _guiKill = false;

    Orbit orbitMode;
    AStarAttack aStarMode;
    Kill killMode;




    /////// WIDGETS ///////
    FieldWidget _fieldWidget;
    TrackingWidget _trackingWidget;
};
