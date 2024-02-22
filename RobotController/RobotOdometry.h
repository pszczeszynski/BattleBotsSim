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

enum OdometryAlg
{
    Blob = 0,
    Heuristic,
    Neural
};


class RobotOdometry
{
public: 
    RobotOdometry(ICameraReceiver &videoSource);

    bool Run(OdometryAlg algorithm); // Runs the algorithm specified
    bool Stop(OdometryAlg algorithm); // Stops the algorithm specified
    bool IsRunning(OdometryAlg algorithm); // Returns true if the algorithm is running

    // Retrieve data. It extrapolates to current time if currTime is specified
    OdometryData Robot(double currTime = 0);
    OdometryData Opponent(double currTime = 0);

    void Update(); // Updates the odometry based on current data

    void SwitchRobots(); // Switches who's who

    // used for calibration
    void UpdateForceSetAngle(double newAngle, bool opponentRobot );
    void UpdateForceSetPosAndVel(cv::Point2f position, cv::Point2f velocity, bool opponentRobot );

    // CODE that needs to be stripped out into an independent heuristic
    void UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame);
    void UpdateIMUOnly(cv::Mat& frame);

    HeuristicOdometry& GetHeuristicOdometry();

private:
    // Video source to initialize odometry olgorithms with
    ICameraReceiver& _videoSource;

    // Some of our odometry algorithms
    BlobDetection _odometry_Blob;
    OdometryData _dataRobot_Blob;
    OdometryData _dataOpponent_Blob;

    HeuristicOdometry _odometry_Heuristic;
    OdometryData _dataRobot_Heuristic;
    OdometryData _dataOpponent_Heuristic;

    // Final Data
    std::mutex _updateMutex;    // Mutex for updating core results
    OdometryData _dataRobot;
    OdometryData _dataOpponent;



    void _PostUpdate(cv::Point2f position, cv::Point2f velocity, Angle angle);
    double _UpdateAndGetIMUAngle();


    double _lastIMUAngle;


    Angle CalcAnglePathTangent();
    bool _visualAngleValid = false;
    Clock _lastVisualAngleValidClock;

    cv::Point2f _lastPositionWhenUpdatedAngle; // angle updated based on displacement
    Clock _lastAccelIntegrateClock;

    #define VISUAL_VELOCITY_HISTORY_SIZE 10
    std::deque<cv::Point2f> _visualVelocities;
};
