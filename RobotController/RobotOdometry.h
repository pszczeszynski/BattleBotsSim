#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "Clock.h"
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
#include "Odometry/OpenCVTracker/OpenCVTracker.h"
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
    NeuralRot,
    OpenCV,
    Gyro
};


class RobotOdometry
{
public: 
    RobotOdometry(ICameraReceiver &videoSource);
    ~RobotOdometry();

    bool Run(OdometryAlg algorithm); // Runs the algorithm specified
    bool Stop(OdometryAlg algorithm); // Stops the algorithm specified
    bool IsRunning(OdometryAlg algorithm); // Returns true if the algorithm is running
    void AutoMatchStart(); // Put odometry into auto start mode waiting for brightness to finish
    void MatchStart(bool partOfAuto = false);     // Just force an auto start (e.g. dont wait for brightness to finish but do everything else)
    
    // Retrieve data. It extrapolates to current time if currTime is specified
    OdometryData Robot(double currTime = DEFAULT_ODOMETRY_EXTRAPOLATION);
    OdometryData Opponent(double currTime =  DEFAULT_ODOMETRY_EXTRAPOLATION);


    float GetIMUOffset();

    void Update(int videoID); // Updates the odometry based on current data

    void FuseAndUpdatePositions(int videoID);

    bool IsTrackingGoodQuality();

    void SwitchRobots(); // Switches who's who

    // used for calibration
    void UpdateForceSetAngle(double newAngle, bool opponentRobot );
    void UpdateForceSetPosAndVel(cv::Point2f position, cv::Point2f velocity, bool opponentRobot );

    void ForceSetPositionOfAlg(OdometryAlg alg, cv::Point2f pos, bool opponent);
    void ForceSetVelocityOfAlg(OdometryAlg alg, cv::Point2f vel, bool opponent);

    OdometryIMU& GetIMUOdometry();
    HeuristicOdometry& GetHeuristicOdometry();
    CVPosition& GetNeuralOdometry();
    BlobDetection& GetBlobOdometry();
    CVRotation& GetNeuralRotOdometry();
    OpenCVTracker& GetOpenCVOdometry();

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

    HumanPosition _odometry_Human_Heuristic;

    bool _dataRobot_Human_is_new = false;
    OdometryData _dataOpponent_Human;
    bool _dataOpponent_Human_is_new = false;

    OdometryIMU _odometry_IMU;
    OdometryData _dataRobot_IMU;

    OpenCVTracker _odometry_opencv;
    OdometryData _dataRobot_opencv;
    OdometryData _dataOpponent_opencv;


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


    void LogOdometryToFile(std::string& extraHeader, std::string& extraData);
    std::stringstream GetOdometryLog( const std::string& name, OdometryData& odometry, bool doheader = false);
    bool _logOdometryFileOpen = false;
    std::string _logOdometryFileName = "./Logs/odometry_log.csv";
    std::ofstream _logOdometryFile; 

    // debug image
    void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0)); // Returns an image that is used for debugging purposes. It can be empty if no debug image is available
    std::mutex _mutexDebugImage;
    std::string _debugString;
};
