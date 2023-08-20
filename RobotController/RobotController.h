#pragma once
#include "ServerSocket.h"
#include "Clock.h"

#include "CameraReceiver.h"
#include "Vision.h"
#include <QObject>
#include <QPixmap>
#include "RobotOdometry.h"
#include "RobotLink.h"
#include "RobotControllerGUI.h"
#include "Input/Gamepad.h"
#include "../Communication/Communication.h"
#include "Extrapolator.h"
#include "MatQueue.h"
#include "SelfRighter.h"

class RobotController : public QObject
{
    Q_OBJECT
public:
    RobotController();
    static RobotController& GetInstance();

    IMUData& GetIMUData();
    CANData GetCANData();

    float& GetFrontWeaponTargetPowerRef();
    float& GetBackWeaponTargetPowerRef();

    MatQueue drawingImageQueue;

    IRobotLink& GetRobotLink();

    Clock visionClock;

signals:
    // updates the image of the field in the GUI
    void RefreshFieldImageSignal();
public slots:
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

    void ProduceDrawingImage();

    cv::Mat drawingImage;

#ifdef XBOX
    XBox gamepad;
#else
    DualSense gamepad;
#endif

    RobotMessage _lastIMUMessage;
    RobotMessage _lastCANMessage;
    std::mutex _lastCanMessageMutex;

    RobotMessageType _lastMessageType;
    RobotSimState exState;

#ifdef SIMULATION
    CameraReceiverSim overheadCamL_sim;
    RobotLinkSim robotLink;
#else
    CameraReceiver overheadCamL_real;
    RobotLinkReal robotLink;
#endif

    Vision vision;

    SelfRighter _selfRighter;

    float _frontWeaponPower = 0;
    float _backWeaponPower = 0;

    bool _orbiting = false;
    bool _killing = false;
};
