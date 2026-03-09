#pragma once
#include <array>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "CVRotation.h"
#include "CameraReceiver.h"
#include "Clock.h"
#include "MathUtils.h"
#include "Odometry/BlobDetection/BlobDetection.h"
#include "Odometry/Heuristic1/HeuristicOdometry.h"
#include "Odometry/Human/HumanPosition.h"
#include "Odometry/IMU/OdometryIMU.h"
#include "Odometry/LKFlowTracker/LKFlowTracker.h"
#include "Odometry/ManualOverride/ManualOverrideOdometry.h"
#include "Odometry/Neural/CVPosition.h"
#include "Odometry/OdometryBase.h"
#include "OdometryPoller.h"
#include "Odometry/OdometryLogger.h"
#include "Odometry/OpenCVTracker/OpenCVTracker.h"

// #define DEFAULT_ODOMETRY_EXTRAPOLATION 0
#define DEFAULT_ODOMETRY_EXTRAPOLATION Clock::programClock.getElapsedTime()

class OdometryIMU;

// Back-annotation instructions for algorithms that need position/angle updates
struct BackAnnotation {
  // Swap operations (must happen before other back-annotations)
  bool swapHeuristic = false;
  bool swapBlob = false;

  bool forceRobotPos_Heuristic = false;
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

class RobotOdometry {
 public:
  RobotOdometry(ICameraReceiver& videoSource);
  ~RobotOdometry();

  bool Run(OdometryAlg algorithm);   // Runs the algorithm specified
  bool Stop(OdometryAlg algorithm);  // Stops the algorithm specified
  bool IsRunning(
      OdometryAlg algorithm);  // Returns true if the algorithm is running
  void AutoMatchStart();       // Put odometry into auto start mode waiting for
                               // brightness to finish
  void MatchStart(
      bool partOfAuto = false);  // Just force an auto start (e.g. dont wait for
                                 // brightness to finish but do everything else)

  // Retrieve data. It extrapolates to current time if currTime is specified
  OdometryData Robot(double currTime = DEFAULT_ODOMETRY_EXTRAPOLATION);
  OdometryData Opponent(double currTime = DEFAULT_ODOMETRY_EXTRAPOLATION);

  void Update();  // Updates the odometry based on current data

  FusionOutput Fuse(RawInputs inputs, double now,
                    const OdometryData& prevRobot,
                    const OdometryData& prevOpponent);
  void ApplyBackAnnotation(const BackAnnotation& backAnnotate,
                           const OdometryData& robot,
                           const OdometryData& opponent);

  // used for calibration
  void UpdateForceSetAngle(double newAngle, bool opponentRobot);
  void UpdateForceSetPosAndVel(cv::Point2f position, cv::Point2f velocity,
                               bool opponentRobot);

  OdometryIMU& GetIMUOdometry();
  HeuristicOdometry& GetHeuristicOdometry();
  CVPosition& GetNeuralOdometry();
  BlobDetection& GetBlobOdometry();
  CVRotation& GetNeuralRotOdometry();
  LKFlowTracker& GetLKFlowOdometry();
  ManualOverrideOdometry& GetManualOverrideOdometry();
  OpenCVTracker& GetOpenCVOdometry();

  // Get the current debug string for external display (thread-safe)
  std::string GetDebugString();

  void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0));

 private:
  // Video source to initialize odometry olgorithms with
  ICameraReceiver& _videoSource;

  // Input polling
  OdometryPoller _poller;
  RawInputs _prevInputs;

  // Odometry algorithms (stored for lifetime and typed access)
  BlobDetection _odometry_Blob;
  HeuristicOdometry _odometry_Heuristic;
  CVPosition _odometry_Neural;
  CVRotation _odometry_NeuralRot;
  HumanPosition _odometry_Human;
  HumanPosition _odometry_Human_Heuristic;
  OdometryIMU _odometry_IMU;
  LKFlowTracker _odometry_LKFlow;
  ManualOverrideOdometry _odometry_Override;
  OpenCVTracker _odometry_opencv;

  // Dispatch table: std::array indexed by OdometryAlg.
  std::array<OdometryBase*, kOdometryAlgCount> _algorithms{};
  void _InitAlgorithmTable();

  // Final Data
  std::mutex _updateMutex;  // Mutex for updating core results
  OdometryData _dataRobot;
  OdometryData _dataOpponent;

  Angle CalcAnglePathTangent();
  bool _visualAngleValid = false;

  /// Slowly blends opponent angle towards velocity direction when moving fast.
  /// Uses a slow moving average; result is back-annotated via fusion output.
  void _BlendOpponentAngleTowardsVelocity(FusionOutput& output);
  Clock _lastVisualAngleValidClock;

  cv::Point2f
      _lastPositionWhenUpdatedAngle;  // angle updated based on displacement
  Clock _lastAccelIntegrateClock;

#define VISUAL_VELOCITY_HISTORY_SIZE 10
  std::deque<cv::Point2f> _visualVelocities;

  void LogOdometryToFile(std::string& extraHeader, std::string& extraData);
  std::stringstream GetOdometryLog(const std::string& name,
                                   OdometryData& odometry,
                                   bool doheader = false);
  bool _logOdometryFileOpen = false;
  std::string _logOdometryFileName = "./Logs/odometry_log.csv";
  std::ofstream _logOdometryFile;

  std::mutex _mutexDebugImage;
  std::string _debugString;

  OdometryLogger _fusionLogger;
};
