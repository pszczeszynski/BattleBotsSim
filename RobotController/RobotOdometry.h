#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "Clock.h"
#include "MathUtils.h"
#include "Globals.h"
#include "CVRotation.h"
#include "CameraReceiver.h"
#include "OdometryPoller.h"
#include "Odometry/OdometryBase.h"
#include "Odometry/BlobDetection/BlobDetection.h"
#include "Odometry/Heuristic1/HeuristicOdometry.h"
#include "Odometry/Human/HumanPosition.h"
#include "Odometry/IMU/OdometryIMU.h"
#include "Odometry/Neural/CVPosition.h"
#include "Odometry/LKFlowTracker/LKFlowTracker.h"

#ifdef USE_OPENCV_TRACKER
#include "Odometry/OpenCVTracker/OpenCVTracker.h"
#endif
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
    LKFlow,
    Gyro
};

// Back-annotation instructions for algorithms that need position/angle updates
struct BackAnnotation {
    // Swap operations (must happen before other back-annotations)
    bool swapHeuristic = false;
    bool swapBlob = false;
    
    // Robot back-annotation
    bool setRobotPos_Heuristic = false;
    bool setRobotPos_Blob = false;
    bool forceRobotPos_Heuristic = false;
    bool forceRobotPos_Blob = false;
    bool setRobotAngle_Heuristic = false;
    bool setRobotAngle_Blob = false;
    
    // Opponent back-annotation
    bool setOpponentPos_Heuristic = false;
    bool setOpponentPos_Blob = false;
    bool setOpponentAngle_Heuristic = false;
    bool setOpponentAngle_Blob = false;
    bool setOpponentAngle_LKFlow = false;
    
    // ROI update for LKFlow
    bool updateLKFlowROI = false;
    cv::Rect lkFlowROI;
};

// Output of fusion process
struct FusionOutput {
    // Fused odometry results
    OdometryData robot;
    OdometryData opponent;
    
    // Back-annotation instructions
    BackAnnotation backAnnotate;
    
    // Debug information
    std::string debugString;
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

    void Update(); // Updates the odometry based on current data

    FusionOutput Fuse(const RawInputs& inputs, double now,
                      const OdometryData& prevRobot,
                      const OdometryData& prevOpponent);
    void ApplyBackAnnotation(const BackAnnotation& backAnnotate, 
                             const OdometryData& robot, 
                             const OdometryData& opponent);
    void DrawTrackingVisualization();

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
    LKFlowTracker& GetLKFlowOdometry();
#ifdef USE_OPENCV_TRACKER
    OpenCVTracker& GetOpenCVOdometry();
#endif

    void _AdjustAngleWithArrowKeys();

    // Get the current debug string for external display (thread-safe)
    std::string GetDebugString();

private:
    // Video source to initialize odometry olgorithms with
    ICameraReceiver& _videoSource;

    // Tunings
    const double _dataAgeThreshold = 0.1; // How old data can be before we consider it invalid

    // Input polling
    OdometryPoller _poller;
    RawInputs _prevInputs;

    // Odometry algorithms
    BlobDetection _odometry_Blob;
    HeuristicOdometry _odometry_Heuristic;
    CVPosition _odometry_Neural;
    CVRotation _odometry_NeuralRot;
    HumanPosition _odometry_Human;
    HumanPosition _odometry_Human_Heuristic;
    OdometryIMU _odometry_IMU;
    LKFlowTracker _odometry_LKFlow;

#ifdef USE_OPENCV_TRACKER
    OpenCVTracker _odometry_opencv;
#endif


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
