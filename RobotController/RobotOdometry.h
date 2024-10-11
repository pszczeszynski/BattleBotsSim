#pragma once
#include <opencv2/opencv.hpp>
#include "Clock.h"
#include "OpponentProfile.h"
#include "MathUtils.h"
#include "Globals.h"
#include "CVRotation.h"
#include "CameraReceiver.h"
#include "Odometry/OdometryBase.h"
#include "Odometry/BlobDetection/BlobDetection.h"
#include "Odometry/Heuristic1/HeuristicOdometry.h"
#include "Odometry/Human/HumanPosition.h"
#include "Odometry/IMU/OdometryIMU.h"
#include "Odometry/Neural/CVPosition.h"
#include "UIWidgets/ImageWidget.h"

// #define DEFAULT_ODOMETRY_EXTRAPOLATION 0
#define DEFAULT_ODOMETRY_EXTRAPOLATION Clock::programClock.getElapsedTime()

class OdometryIMU;

enum OdometryAlg
{
    Blob = 0,
    Heuristic,
    IMU,
    Neural,
    Human,
    NeuralRot
};


class RobotOdometry
{
public: 
    RobotOdometry(ICameraReceiver &videoSource);

    bool Run(OdometryAlg algorithm); // Runs the algorithm specified
    bool Stop(OdometryAlg algorithm); // Stops the algorithm specified
    bool IsRunning(OdometryAlg algorithm); // Returns true if the algorithm is running

    // Retrieve data. It extrapolates to current time if currTime is specified
    OdometryData Robot(double currTime = DEFAULT_ODOMETRY_EXTRAPOLATION);
    OdometryData Opponent(double currTime =  DEFAULT_ODOMETRY_EXTRAPOLATION);


    float GetIMUOffset();

    void Update(); // Updates the odometry based on current data

    void FuseAndUpdatePositions();

    bool IsTrackingGoodQuality();

    void SwitchRobots(); // Switches who's who

    // used for calibration
    void UpdateForceSetAngle(double newAngle, bool opponentRobot );
    void UpdateForceSetPosAndVel(cv::Point2f position, cv::Point2f velocity, bool opponentRobot );

    void ForceSetPositionOfAlg(OdometryAlg alg, cv::Point2f pos, bool opponent);
    void ForceSetVelocityOfAlg(OdometryAlg alg, cv::Point2f vel, bool opponent);

    HeuristicOdometry& GetHeuristicOdometry();
    CVPosition& GetNeuralOdometry();
    BlobDetection& GetBlobOdometry();
    void _AdjustAngleWithArrowKeys();

private:
    // Video source to initialize odometry olgorithms with
    ICameraReceiver& _videoSource;

    // Tunings
    const double _dataAgeThreshold = 0.1; // How old data can be before we consider it invalid

    // Some of our odometry algorithms
    BlobDetection _odometry_Blob;
    OdometryData _dataRobot_Blob;
    OdometryData _dataOpponent_Blob;

    HeuristicOdometry _odometry_Heuristic;
    OdometryData _dataRobot_Heuristic;
    OdometryData _dataOpponent_Heuristic;
    OdometryData _dataOpponent_Heuristic_prev;

    CVPosition _odometry_Neural;
    OdometryData _dataRobot_Neural;

    CVRotation _odometry_NeuralRot;
    OdometryData _dataRobot_NeuralRot;

    HumanPosition _odometry_Human;
    OdometryData _dataRobot_Human;
    bool _dataRobot_Human_is_new = false;
    OdometryData _dataOpponent_Human;
    bool _dataOpponent_Human_is_new = false;

    OdometryIMU _odometry_IMU;
    OdometryData _dataRobot_IMU;


    // Final Data
    std::mutex _updateMutex;    // Mutex for updating core results
    OdometryData _dataRobot;
    OdometryData _dataOpponent;

    Angle CalcAnglePathTangent();
    bool _visualAngleValid = false;
    Clock _lastVisualAngleValidClock;

    cv::Point2f _lastPositionWhenUpdatedAngle; // angle updated based on displacement
    Clock _lastAccelIntegrateClock;

    #define VISUAL_VELOCITY_HISTORY_SIZE 10
    std::deque<cv::Point2f> _visualVelocities;
};
