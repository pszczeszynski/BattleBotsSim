#include "RobotOdometry.h"

#include "../Common/Communication.h"
#include "Globals.h"
#include "Input/InputState.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "SafeDrawing.h"
#include "SimulationState.h"
#include "imgui.h"
#include "odometry/Neural/CVPosition.h"

// Statemachine definitions
enum FUSION_SM {
  FUSION_NORMAL = 0,  // Normal State
  FUSION_WAIT_FOR_BG  // Wait for background to stabilize

};

FUSION_SM fusionStateMachine = FUSION_NORMAL;

RobotOdometry::RobotOdometry(ICameraReceiver &videoSource)
    : _videoSource(videoSource),
      _odometry_Blob(&videoSource),
      _odometry_Heuristic(&videoSource),
      _odometry_Neural(&videoSource),
      _odometry_Human(&videoSource, "11120", false),
      _odometry_Human_Heuristic(&videoSource, "11121", true),
      _odometry_NeuralRot(&videoSource),
      _odometry_LKFlow(&videoSource)
#ifdef USE_OPENCV_TRACKER
      ,
      _odometry_opencv(&videoSource)
#endif
      ,
      _poller(_odometry_Blob, _odometry_Heuristic, _odometry_Neural,
              _odometry_NeuralRot, _odometry_IMU, _odometry_Human,
              _odometry_Human_Heuristic, _odometry_LKFlow
#ifdef USE_OPENCV_TRACKER
              ,
              _odometry_opencv
#endif
      ) {
}

RobotOdometry::~RobotOdometry() {
  // If log file open, close it
  if (_logOdometryFileOpen) {
    _logOdometryFile.close();
    _logOdometryFileOpen = false;
  }
}

void RobotOdometry::_AdjustAngleWithArrowKeys() {
  static Clock updateClock;

  float speed = 1.0;
  // if shift is held, multiply by 2
  if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift)) {
    speed = 2.0;
  }

  Angle angleUserAdjust =
      Angle(updateClock.getElapsedTime() * 90 * M_PI / 180.0 * speed);
  updateClock.markStart();

  if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftArrow)) {
    UpdateForceSetAngle(_dataRobot.GetAngle() - angleUserAdjust, false);
  } else if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightArrow)) {
    UpdateForceSetAngle(_dataRobot.GetAngle() + angleUserAdjust, false);
  }

  // opponent with up and 1 and 3
  if (InputState::GetInstance().IsKeyDown(ImGuiKey_1)) {
    UpdateForceSetAngle(_dataOpponent.GetAngle() - angleUserAdjust, true);
  } else if (InputState::GetInstance().IsKeyDown(ImGuiKey_3)) {
    UpdateForceSetAngle(_dataOpponent.GetAngle() + angleUserAdjust, true);
  }
}

// Put odometry into auto start mode waiting for brightness to finish
void RobotOdometry::AutoMatchStart() {
  _odometry_Heuristic.AutoMatchStart();

  // Fix the positions now just for visuals, but will re-run later
  MatchStart(true);

  fusionStateMachine = FUSION_WAIT_FOR_BG;
}

// Just force an auto start (e.g. dont wait for brightness to finish but do
// everything else)
void RobotOdometry::MatchStart(bool partOfAuto) {
  double currTime = Clock::programClock.getElapsedTime();

  // Set all the positions and angles
  _dataRobot.robotPosition = TrackingWidget::robotMouseClickPoint;
  _dataRobot.SetAngle(Angle(TrackingWidget::robotMouseClickAngle), 0, currTime,
                      true);
  _dataOpponent.robotPosition = TrackingWidget::opponentMouseClickPoint;
  _dataOpponent.SetAngle(Angle(TrackingWidget::opponentMouseClickAngle), 0,
                         currTime, true);
  _dataRobot.robotVelocity = cv::Point2f(0, 0);
  _dataOpponent.robotVelocity = cv::Point2f(0, 0);
  _dataRobot.time = currTime;
  _dataOpponent.time = currTime;
  _dataRobot.robotPosValid = true;
  _dataOpponent.robotPosValid = true;

  _odometry_Heuristic.ForcePosition(_dataRobot.robotPosition, false);
  _odometry_Heuristic.SetAngle(_dataRobot.GetAngle(), false, _dataRobot.time,
                               _dataRobot.GetAngleVelocity(), true);
  _odometry_Heuristic.SetVelocity(_dataRobot.robotVelocity, false);

  _odometry_Heuristic.ForcePosition(_dataOpponent.robotPosition, true);
  _odometry_Heuristic.SetAngle(_dataOpponent.GetAngle(), true,
                               _dataOpponent.time,
                               _dataOpponent.GetAngleVelocity(), true);
  _odometry_Heuristic.SetVelocity(_dataOpponent.robotVelocity, true);

  _odometry_Blob.ForcePosition(_dataRobot.robotPosition, false);
  _odometry_Blob.SetAngle(_dataRobot.GetAngle(), false, _dataRobot.time,
                          _dataRobot.GetAngleVelocity(), true);
  _odometry_Blob.SetVelocity(_dataRobot.robotVelocity, false);

  _odometry_Blob.ForcePosition(_dataOpponent.robotPosition, true);
  _odometry_Blob.SetAngle(_dataOpponent.GetAngle(), true, _dataOpponent.time,
                          _dataOpponent.GetAngleVelocity(), true);
  _odometry_Blob.SetVelocity(_dataOpponent.robotVelocity, true);

  // Set ROI for LKFlowTracker (only tracks opponent)
  if (_odometry_LKFlow.IsRunning()) {
    int roiSize = 20;
    cv::Rect roi(static_cast<int>(_dataOpponent.robotPosition.x -
                                  static_cast<float>(roiSize) / 2.0f),
                 static_cast<int>(_dataOpponent.robotPosition.y -
                                  static_cast<float>(roiSize) / 2.0f),
                 roiSize, roiSize);
    _odometry_LKFlow.SetROI(roi);
  }

#ifdef USE_OPENCV_TRACKER
  _odometry_opencv.ForcePosition(_dataOpponent.robotPosition, true);
  _odometry_opencv.SetAngle(_dataOpponent.GetAngle(), true, _dataOpponent.time,
                            _dataOpponent.GetAngleVelocity(), true);
  _odometry_opencv.SetVelocity(_dataOpponent.robotVelocity, true);
#endif

  fusionStateMachine = FUSION_NORMAL;

  if (!partOfAuto) {
    _odometry_Heuristic.MatchStart();
  }
}

// Updates internal Odometry data
void RobotOdometry::Update() {
  // ******************************
  // Retrieve new data if available
  TrackingWidget *trackingInfo = TrackingWidget::GetInstance();

  // Poll all odometry algorithms
  RawInputs inputs = _poller.Poll(trackingInfo, _prevInputs);

  // No new data and thus nothing to do
  if (!inputs.HasUpdates()) {
    return;
  }

  // Store for next iteration
  _prevInputs = inputs;

  // Handle state machine
  if (fusionStateMachine == FUSION_WAIT_FOR_BG) {
    // If heuristic is not active, leave this state
    if (!inputs.IsRunning(RawInputs::US_HEURISTIC |
                          RawInputs::THEM_HEURISTIC)) {
      fusionStateMachine = FUSION_NORMAL;
    }

    // Check if background is ready
    if (_odometry_Heuristic.IsBackgroundStable()) {
      fusionStateMachine = FUSION_NORMAL;
      MatchStart(true);
    }
  }

  // Perform fusion only in normal state
  if (fusionStateMachine == FUSION_NORMAL) {
    double currTime = Clock::programClock.getElapsedTime();

    // Copy baseline state under lock before fusion
    OdometryData prevRobot, prevOpponent;
    {
      std::lock_guard<std::mutex> lock(_updateMutex);
      prevRobot = _dataRobot;
      prevOpponent = _dataOpponent;
    }

    // Call fusion (pure function, no shared state access)
    FusionOutput fusionResult = Fuse(inputs, currTime, prevRobot, prevOpponent);

    // Apply fusion results
    std::unique_lock<std::mutex> locker(_updateMutex);
    _dataRobot = fusionResult.robot;
    _dataOpponent = fusionResult.opponent;

    // Copy debug string
    {
      std::unique_lock<std::mutex> debugLocker(_mutexDebugImage);
      _debugString = fusionResult.debugString;
    }
    locker.unlock();

    // Apply back-annotation (these modify algorithm state, so outside mutex)
    // Pass fused outputs directly to avoid race conditions with other threads
    ApplyBackAnnotation(fusionResult.backAnnotate, fusionResult.robot,
                        fusionResult.opponent);

    // Draw debug visualization
    DrawTrackingVisualization();
  }

  GetDebugImage(trackingInfo->GetDebugImage("Fusion"),
                trackingInfo->GetDebugOffset("Fusion"));

  // // If IMU is running, then use IMU's angle information
  // if (_odometry_IMU.IsRunning() && _dataRobot_IMU.robotAngleValid)
  // {
  //     _dataRobot.robotAngleValid = true;
  //     _dataRobot.robotAngle = _dataRobot_IMU.robotAngle;
  //     _dataRobot.robotAngleVelocity = _dataRobot_IMU.robotAngleVelocity;
  // }

  // _dataOpponent.robotAngle = Angle(opponentRotationSim);

  // locker will get unlocked here automatically
}

FusionOutput RobotOdometry::Fuse(const RawInputs &inputs, double now,
                                 const OdometryData &prevRobot,
                                 const OdometryData &prevOpponent) {
  FusionOutput output;

  // Initialize output with previous state (for extrapolation baseline)
  // These are passed as parameters to avoid reading _dataRobot/_dataOpponent without lock
  output.robot = prevRobot;
  output.opponent = prevOpponent;

  // Extrapolate input data to current time
  OdometryData ext_dataRobot_Blob = inputs.us_blob.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_Heuristic = inputs.us_heuristic.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_Neural = inputs.us_neural;
  OdometryData ext_dataRobot_NeuralRot = inputs.us_neuralrot.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_IMU = inputs.us_imu.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_Human = inputs.us_human.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_Human = inputs.them_human.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_Blob = inputs.them_blob.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_Heuristic =
      inputs.them_heuristic.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_LKFlow = inputs.them_lkflow.ExtrapolateBoundedTo(now);

  // ******************************
  // HUMAN OVERRIDES
  // ******************************
  std::string debugROStringForVideo_tmp = "";

  bool humanThemAngle_valid = false;
  // Opponent rotation
  if (inputs.IsRunning(RawInputs::THEM_HUMAN) &&
      ext_dataOpponent_Human.IsAngleValid() && inputs.them_human_is_new) {
    // Check if it arrived within reasonable time
    if (ext_dataOpponent_Human.GetAge() < _dataAgeThreshold) {
      // Set opponent data (note: with 0 angular velocity)
      output.opponent.SetAngle(ext_dataOpponent_Human.GetAngle(), 0,
                               ext_dataOpponent_Human.GetAngleFrameTime(),
                               true);

      // Mark that we should force these angles to be the human's
      ext_dataOpponent_Heuristic.InvalidateAngle();
      ext_dataOpponent_Blob.InvalidateAngle();

      // Set flag so the Human branch will be selected in THEM ROT
      humanThemAngle_valid = true;

      debugROStringForVideo_tmp +=
          "Human_Rot set = " +
          std::to_string(ext_dataOpponent_Human.GetAngle()) + "\n";
    }
  }

  // ******************************
  // HELPER VARIABLES
  // Use runningMask to check running state at time of poll (prevents race
  // conditions)
  // ******************************

  bool heuristicUsValid = inputs.IsRunning(RawInputs::US_HEURISTIC) &&
                          (inputs.us_heuristic.GetAge() < _dataAgeThreshold);
  bool heuristicUsPos_valid = heuristicUsValid && inputs.us_heuristic.robotPosValid;
  bool heuristicUsAngle_valid = heuristicUsValid && inputs.us_heuristic.IsAngleValid();
  bool heuristicThemValid = inputs.IsRunning(RawInputs::THEM_HEURISTIC) &&
                            (inputs.them_heuristic.GetAge() < _dataAgeThreshold);
  bool heuristicThemPos_valid =
      heuristicThemValid && inputs.them_heuristic.robotPosValid;
  bool heuristicThemAngle_valid =
      heuristicThemValid && inputs.them_heuristic.IsAngleValid();

  bool blobUsValid = inputs.IsRunning(RawInputs::US_BLOB) &&
                     (inputs.us_blob.GetAge() < _dataAgeThreshold);
  bool blobUsPos_valid = blobUsValid && inputs.us_blob.robotPosValid;
  bool blobUsAngle_valid = blobUsValid && inputs.us_blob.IsAngleValid();
  bool blobThemValid = inputs.IsRunning(RawInputs::THEM_BLOB) &&
                       (inputs.them_blob.GetAge() < _dataAgeThreshold);
  bool blobThemPos_valid = blobThemValid && inputs.them_blob.robotPosValid;
  bool blobThemAngle_valid = blobThemValid && inputs.them_blob.IsAngleValid();

  bool lkFlowThemValid = inputs.IsRunning(RawInputs::THEM_LKFLOW) &&
                         (inputs.them_lkflow.GetAge() < _dataAgeThreshold);
  bool lkFlowThemAngle_valid = lkFlowThemValid && inputs.them_lkflow.IsAngleValid();

  bool neuralUsPos_valid =
      inputs.IsRunning(RawInputs::US_NEURAL) && inputs.us_neural.robotPosValid;  // &&
  //  (inputs.us_neural.GetAge() < _dataAgeThreshold);

  bool neuralRot_valid = inputs.IsRunning(RawInputs::US_NEURALROT) &&
                         inputs.us_neuralrot.IsAngleValid() &&
                         (inputs.us_neuralrot.GetAge() < _dataAgeThreshold);

  bool imuValid = inputs.IsRunning(RawInputs::US_IMU) &&
                  (inputs.us_imu.GetAge() < _dataAgeThreshold);
  bool imuUsRot_valid = imuValid && inputs.us_imu.IsAngleValid();

  debugROStringForVideo_tmp += "ALG U_P U_V U_R UAV T_P T_V T_R TAV\n";
  debugROStringForVideo_tmp += "Heu  " + std::to_string(heuristicUsPos_valid) +
                               "   " + std::to_string(heuristicUsPos_valid) +
                               "    " + std::to_string(heuristicUsAngle_valid) +
                               "    " + std::to_string(heuristicUsAngle_valid) +
                               "   " + std::to_string(heuristicThemPos_valid) +
                               "    " + std::to_string(heuristicThemPos_valid) +
                               "   " +
                               std::to_string(heuristicThemAngle_valid) + "\n";
  debugROStringForVideo_tmp += "Blb  " + std::to_string(blobUsPos_valid) +
                               "   " + std::to_string(blobUsPos_valid) +
                               "    " + std::to_string(blobUsAngle_valid) +
                               "    " + std::to_string(blobUsAngle_valid) +
                               "   " + std::to_string(blobThemPos_valid) +
                               "    " + std::to_string(blobThemPos_valid) +
                               "   " + std::to_string(blobThemAngle_valid) +
                               "\n";
  debugROStringForVideo_tmp +=
      "Neu  " + std::to_string(neuralUsPos_valid) + "\n";
  debugROStringForVideo_tmp += "NeR  " + std::string("   ") + "   " +
                               std::string("   ") + "    " +
                               std::to_string(neuralRot_valid) + "\n";
  debugROStringForVideo_tmp += "IMU  " + std::string("   ") + "   " +
                               std::string("   ") + "    " +
                               std::to_string(imuUsRot_valid) + "\n";
  debugROStringForVideo_tmp +=
      "LKFl " + std::string("   ") + "   " + std::string("   ") + "    " +
      std::string("   ") + "   " + std::string("   ") + "    " +
      std::string("   ") + "   " + std::to_string(lkFlowThemAngle_valid) + "\n";

  // Log the odometry data to file
  std::string logAlgHeader = "ALG UP UR TP TR";
  std::string logAlgData = "Heu:" + std::to_string(heuristicUsPos_valid) +
                           std::to_string(heuristicUsAngle_valid) +
                           std::to_string(heuristicThemPos_valid) +
                           std::to_string(heuristicThemAngle_valid);
  logAlgData += " Blb:" + std::to_string(blobUsPos_valid) + "_" +
                std::to_string(blobThemPos_valid) + "_";
  logAlgData += " Neu:" + std::to_string(neuralUsPos_valid);
  logAlgData += " NeR:" + std::string("_") + std::to_string(neuralRot_valid);
  logAlgData += " IMU:" + std::string("_") + std::to_string(imuUsRot_valid);
  LogOdometryToFile(logAlgHeader, logAlgData);

  // If we determine a force position is required, mark it using this variable
  bool forceUsPosition = false;

  // ******************************
  // Prechecks
  // ******************************

  // G1) if Neural US = Heuristic.them: SWAP Heuristic
  if (neuralUsPos_valid && heuristicThemPos_valid) {
    // Check if point is inside bounding box
    if (ext_dataOpponent_Heuristic.IsPointInside(
            ext_dataRobot_Neural.robotPosition)) {
      debugROStringForVideo_tmp += "G1: Swap Robots, Heu Both invalidated\n";

      // Mark for swap (will be applied in back-annotation)
      output.backAnnotate.swapHeuristic = true;

      heuristicThemPos_valid = false;
      heuristicThemAngle_valid = false;
      heuristicUsPos_valid = false;
      heuristicUsAngle_valid = false;

      ext_dataRobot_Heuristic.InvalidatePosition();
      ext_dataRobot_Heuristic.InvalidateAngle();
      ext_dataOpponent_Heuristic.InvalidatePosition();
      ext_dataOpponent_Heuristic.InvalidateAngle();
    }
  }

  // Same check for Blob
  if (neuralUsPos_valid && blobThemPos_valid) {
    // Check if point is inside bounding box
    if (ext_dataOpponent_Blob.IsPointInside(
            ext_dataRobot_Neural.robotPosition)) {
      debugROStringForVideo_tmp += "G1: Swap Robots, Blob Both invalidated\n";

      // Mark for swap (will be applied in back-annotation)
      output.backAnnotate.swapBlob = true;

      blobThemPos_valid = false;
      blobThemAngle_valid = false;
      blobUsPos_valid = false;
      blobUsAngle_valid = false;

      ext_dataRobot_Blob.InvalidatePosition();
      ext_dataRobot_Blob.InvalidateAngle();
      ext_dataOpponent_Blob.InvalidatePosition();
      ext_dataOpponent_Blob.InvalidateAngle();
    }
  }

  // G2) if Neural != Heuristic.us && Neural=Blob.us: Heuristic.ForceUs(Neural)
  if (neuralUsPos_valid && blobUsPos_valid &&
      ext_dataRobot_Blob.IsPointInside(ext_dataRobot_Neural.robotPosition)) {
    // Force position if heuristic doesn't agree
    if (!heuristicUsPos_valid || !ext_dataRobot_Heuristic.IsPointInside(
                                     ext_dataRobot_Neural.robotPosition)) {
      debugROStringForVideo_tmp += "G2: Force position, Heu US invalidated\n";

      heuristicUsPos_valid = false;
      heuristicUsAngle_valid = false;
      ext_dataRobot_Heuristic.InvalidatePosition();
      ext_dataRobot_Heuristic.InvalidateAngle();

      output.robot.robotPosValid = true;
      output.robot.robotPosition = ext_dataRobot_Neural.robotPosition;

      forceUsPosition = true;
      // What about velocity? We'll keep velocity where it was calculated before
      // for now Check below for velocity calculation
    }
  }

  // ******************************
  // PRIORITIES
  // ******************************
  //  US POS:
  //           Rule:  1) Heuristic, 2) Neural Pos, 3) Blob
  //           Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold,
  //           setpos on blob If Heuristic.invalid, 1) setpos(blob) 2)
  //           setpos(neural)
  debugROStringForVideo_tmp += "US POS = ";

  if (heuristicUsPos_valid) {
    debugROStringForVideo_tmp += "Heu";
    output.robot.robotPosition = ext_dataRobot_Heuristic.robotPosition;
    output.robot.time = ext_dataRobot_Heuristic.time;
    output.robot.robotPosValid = true;

    // If blob position isn't inside our rectangle then set position
    if (blobUsPos_valid && !ext_dataRobot_Heuristic.IsPointInside(
                               ext_dataRobot_Blob.robotPosition)) {
      ext_dataRobot_Blob.robotPosValid = false;
    }
  } else if (neuralUsPos_valid) {
    debugROStringForVideo_tmp += "Neu";
    output.robot.robotPosition = ext_dataRobot_Neural.robotPosition;
    output.robot.time = ext_dataRobot_Neural.time;
    output.robot.robotPosValid = true;
  } else if (blobUsPos_valid) {
    debugROStringForVideo_tmp += "Blob";
    output.robot.robotPosition = ext_dataRobot_Blob.robotPosition;
    output.robot.time = ext_dataRobot_Blob.time;
    output.robot.robotPosValid = true;
  }
  debugROStringForVideo_tmp += "\nUS Vel = ";

  //  US VEL:
  //           Rule: 1) Heuristic, 2) Blob
  if (heuristicUsPos_valid) {
    debugROStringForVideo_tmp += "Heu";
    output.robot.robotVelocity = ext_dataRobot_Heuristic.robotVelocity;
  } else if (blobUsPos_valid) {
    debugROStringForVideo_tmp += "Blob";
    output.robot.robotVelocity = ext_dataRobot_Blob.robotVelocity;
  } else {
    debugROStringForVideo_tmp += "OldVel";
    // output.robot.robotVelocity = ext_dataRobot_Neural.robotVelocity;
  }

  //  US ROT:
  //            Rule: 1) IMU (Neural is already fused), 2) Neural Rot 3)
  //            Heuristic 4) Blob
  debugROStringForVideo_tmp += "\nUS Rot = ";
  if (imuUsRot_valid) {
    debugROStringForVideo_tmp += "Imu";
    output.robot.SetAngle(ext_dataRobot_IMU.GetAngle(),
                          ext_dataRobot_IMU.GetAngleVelocity(),
                          ext_dataRobot_IMU.GetAngleFrameTime(), true);

    // Reset heuristic angle calculation
    ext_dataRobot_Heuristic.InvalidateAngle();
    ext_dataRobot_Blob.InvalidateAngle();
  } else if (neuralRot_valid) {
    debugROStringForVideo_tmp += "NeuRot";

    // TODO: decide if we want to use neuralRot velocity
    // Currently uses heuristic's angular velocity or otherwise 0
    double angleVelocity = ext_dataRobot_Heuristic.IsAngleValid()
                               ? ext_dataRobot_Heuristic.GetAngleVelocity()
                               : 0;
    output.robot.SetAngle(ext_dataRobot_NeuralRot.GetAngle(), angleVelocity,
                          ext_dataRobot_NeuralRot.GetAngleFrameTime(), true);

    // Set heuristor to neural
    ext_dataRobot_Heuristic.InvalidateAngle();
    ext_dataRobot_Blob.InvalidateAngle();
  } else if (heuristicUsAngle_valid) {
    debugROStringForVideo_tmp += "Heu";
    output.robot.SetAngle(ext_dataRobot_Heuristic.GetAngle(),
                          ext_dataRobot_Heuristic.GetAngleVelocity(),
                          ext_dataRobot_Heuristic.GetAngleFrameTime(), true);
    ext_dataRobot_Blob.InvalidateAngle();
  } else if (blobUsAngle_valid) {
    debugROStringForVideo_tmp += "Blob";
    output.robot.SetAngle(ext_dataRobot_Blob.GetAngle(),
                          ext_dataRobot_Blob.GetAngleVelocity(),
                          ext_dataRobot_Blob.GetAngleFrameTime(), true);
  }

  //  THEM POS:
  //            Rule: 1) Heuristic, 2) Blob
  //            Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold,
  //            setpos on blob If Heuristic.invalid, setpos(blob)
  debugROStringForVideo_tmp += "\nTHEM POS = ";
  if (heuristicThemPos_valid) {
    debugROStringForVideo_tmp += "Heu";
    output.opponent.robotPosition = ext_dataOpponent_Heuristic.robotPosition;
    output.opponent.time =
        ext_dataOpponent_Heuristic.time;  // Use extrapolated time!
    output.opponent.robotPosValid = true;

    // If blob position isn't inside our rectangle then set position
    if (blobThemPos_valid && !ext_dataOpponent_Heuristic.IsPointInside(
                                 ext_dataOpponent_Blob.robotPosition)) {
      ext_dataOpponent_Blob.robotPosValid = false;
    }
  } else if (blobThemPos_valid) {
    debugROStringForVideo_tmp += "Blob";
    output.opponent.robotPosition = ext_dataOpponent_Blob.robotPosition;
    output.opponent.time = ext_dataOpponent_Blob.time;
    output.opponent.robotPosValid = true;

    ext_dataOpponent_Heuristic.robotPosValid = false;
  }

  // Set ROI for LKFlowTracker when opponent position is finalized
  if (output.opponent.robotPosValid &&
      inputs.IsRunning(RawInputs::THEM_LKFLOW)) {
    // Use opponent blob size to determine ROI size, or default to 140x140
    int roiSize = 70;

    cv::Rect roi(static_cast<int>(output.opponent.robotPosition.x -
                                  static_cast<float>(roiSize) / 2.0f),
                 static_cast<int>(output.opponent.robotPosition.y -
                                  static_cast<float>(roiSize) / 2.0f),
                 roiSize, roiSize);
    // Mark that we need to update the ROI
    output.backAnnotate.updateLKFlowROI = true;
    output.backAnnotate.lkFlowROI = roi;
  }

  debugROStringForVideo_tmp += "\nTHEM VEL = ";
  //  THEM VEL:
  //            Rule: 1) Heuristic, 2) Blob
  if (heuristicThemPos_valid) {
    debugROStringForVideo_tmp += "Heu";
    output.opponent.robotVelocity = ext_dataOpponent_Heuristic.robotVelocity;
  } else if (blobThemPos_valid) {
    debugROStringForVideo_tmp += "Blob";
    output.opponent.robotVelocity = ext_dataOpponent_Blob.robotVelocity;
  }

  debugROStringForVideo_tmp += "\nTHEM ROT = ";
  //  THEM ROT:
  //            Rule: 1) LKFlow, 2) Human, 3) Heuristic, 4) Blob
  if (lkFlowThemAngle_valid) {
    debugROStringForVideo_tmp += "LKFlow";
    output.opponent.SetAngle(ext_dataOpponent_LKFlow.GetAngle(),
                             ext_dataOpponent_LKFlow.GetAngleVelocity(),
                             ext_dataOpponent_LKFlow.GetAngleFrameTime(), true);
  } else if (humanThemAngle_valid) {
    debugROStringForVideo_tmp += "Human";
    output.opponent.SetAngle(ext_dataOpponent_Human.GetAngle(),
                             ext_dataOpponent_Human.GetAngleVelocity(),
                             ext_dataOpponent_Human.GetAngleFrameTime(), true);
  } else if (heuristicThemAngle_valid) {
    debugROStringForVideo_tmp += "Heu";
    output.opponent.SetAngle(ext_dataOpponent_Heuristic.GetAngle(),
                             ext_dataOpponent_Heuristic.GetAngleVelocity(),
                             ext_dataOpponent_Heuristic.GetAngleFrameTime(),
                             true);
  } else if (blobThemAngle_valid) {
    debugROStringForVideo_tmp += "Blob";
    output.opponent.SetAngle(ext_dataOpponent_Blob.GetAngle(),
                             ext_dataOpponent_Blob.GetAngleVelocity(),
                             ext_dataOpponent_Blob.GetAngleFrameTime(), true);
  }

  // *********************************************
  // Back Annotate data to invalid readings
  // *********************************************

  // Robot Position and velocity:
  //   Heuristic, Blob need back-annotated robot positions and velocities
  if (!ext_dataRobot_Heuristic.robotPosValid) {
    output.backAnnotate.setRobotPos_Heuristic = true;
    output.backAnnotate.forceRobotPos_Heuristic = forceUsPosition;
  }

  if (!ext_dataRobot_Blob.robotPosValid) {
    output.backAnnotate.setRobotPos_Blob = true;
    output.backAnnotate.forceRobotPos_Blob = forceUsPosition;
  }

  // Robot Angle + velocity:
  // Heuristic, Blob will need it back annotated
  if (!ext_dataRobot_Heuristic.IsAngleValid()) {
    output.backAnnotate.setRobotAngle_Heuristic = true;
  }

  if (!ext_dataRobot_Blob.IsAngleValid() && output.robot.IsAngleValid()) {
    output.backAnnotate.setRobotAngle_Blob = true;
  }

  // OPPONENT same things:
  // Opponent Position and velocity:
  //   Heuristic, Blob need back-annotated robot positions and velocities
  if (!ext_dataOpponent_Heuristic.robotPosValid &&
      output.opponent.robotPosValid) {
    output.backAnnotate.setOpponentPos_Heuristic = true;
  }

  if (!ext_dataOpponent_Blob.robotPosValid && output.opponent.robotPosValid) {
    output.backAnnotate.setOpponentPos_Blob = true;
  }

  // Opponent angle
  // Heuristic, Blob, LKFlow will need it back annotated
  if (!ext_dataOpponent_Heuristic.IsAngleValid() &&
      output.opponent.IsAngleValid()) {
    output.backAnnotate.setOpponentAngle_Heuristic = true;
  }

  if (!ext_dataOpponent_Blob.IsAngleValid() && output.opponent.IsAngleValid()) {
    output.backAnnotate.setOpponentAngle_Blob = true;
  }

  if (!ext_dataOpponent_LKFlow.IsAngleValid() &&
      output.opponent.IsAngleValid()) {
    output.backAnnotate.setOpponentAngle_LKFlow = true;
  }

#ifdef FORCE_SIM_DATA
  // Use thread-safe SimulationState instead of global variables
  auto robotData = SimulationState::GetInstance().GetRobotData();
  auto opponentData = SimulationState::GetInstance().GetOpponentData();

  output.robot.robotPosValid = true;
  output.robot.robotPosition = robotData.position;
  output.robot.robotVelocity = robotData.velocity;
  output.robot.time = robotData.timestamp;

  output.opponent.robotPosValid = true;
  output.opponent.robotPosition = opponentData.position;
  output.opponent.robotVelocity = opponentData.velocity;
  output.opponent.SetAngle(Angle(opponentData.rotation),
                           opponentData.rotationVelocity,
                           opponentData.timestamp, true);
  output.opponent.time = opponentData.timestamp;
#endif

  // ******************************
  //  FINISHED
  // ******************************

  // Set the debug string
  output.debugString = debugROStringForVideo_tmp;

  return output;
}

void RobotOdometry::ApplyBackAnnotation(const BackAnnotation &backAnnotate,
                                        const OdometryData &robot,
                                        const OdometryData &opponent) {
  // CRITICAL: Apply swaps FIRST, before any other back-annotations
  // This ensures the algorithm state is correct for subsequent operations
  if (backAnnotate.swapHeuristic) {
    _odometry_Heuristic.SwitchRobots();
  }

  if (backAnnotate.swapBlob) {
    _odometry_Blob.SwitchRobots();
  }

  // Robot Position and velocity - Heuristic
  if (backAnnotate.setRobotPos_Heuristic) {
    if (backAnnotate.forceRobotPos_Heuristic) {
      _odometry_Heuristic.ForcePosition(robot.robotPosition, false);
    } else {
      _odometry_Heuristic.SetPosition(robot.robotPosition, false);
    }
    _odometry_Heuristic.SetVelocity(robot.robotVelocity, false);
  }

  // Robot Position and velocity - Blob
  if (backAnnotate.setRobotPos_Blob) {
    if (backAnnotate.forceRobotPos_Blob) {
      _odometry_Blob.ForcePosition(robot.robotPosition, false);
    } else {
      _odometry_Blob.SetPosition(robot.robotPosition, false);
    }
    _odometry_Blob.SetVelocity(robot.robotVelocity, false);
  }

  // Robot Angle - Heuristic
  if (backAnnotate.setRobotAngle_Heuristic) {
    _odometry_Heuristic.SetAngle(robot.GetAngle(), false,
                                 robot.GetAngleFrameTime(),
                                 robot.GetAngleVelocity(), true);
  }

  // Robot Angle - Blob
  if (backAnnotate.setRobotAngle_Blob) {
    _odometry_Blob.SetAngle(robot.GetAngle(), false,
                            robot.GetAngleFrameTime(),
                            robot.GetAngleVelocity(), true);
  }

  // Opponent Position and velocity - Heuristic
  if (backAnnotate.setOpponentPos_Heuristic) {
    _odometry_Heuristic.SetPosition(opponent.robotPosition, true);
    _odometry_Heuristic.SetVelocity(opponent.robotVelocity, true);
  }

  // Opponent Position and velocity - Blob
  if (backAnnotate.setOpponentPos_Blob) {
    _odometry_Blob.SetPosition(opponent.robotPosition, true);
    _odometry_Blob.SetVelocity(opponent.robotVelocity, true);
  }

  // Opponent Angle - Heuristic
  if (backAnnotate.setOpponentAngle_Heuristic) {
    _odometry_Heuristic.SetAngle(opponent.GetAngle(), true,
                                 opponent.GetAngleFrameTime(),
                                 opponent.GetAngleVelocity(), true);
  }

  // Opponent Angle - Blob
  if (backAnnotate.setOpponentAngle_Blob) {
    _odometry_Blob.SetAngle(opponent.GetAngle(), true,
                            opponent.GetAngleFrameTime(),
                            opponent.GetAngleVelocity(), true);
  }

  // Opponent Angle - LKFlow
  if (backAnnotate.setOpponentAngle_LKFlow) {
    _odometry_LKFlow.SetAngle(opponent.GetAngle(), true,
                              opponent.GetAngleFrameTime(),
                              opponent.GetAngleVelocity(), true);
  }

  // LKFlow ROI update
  if (backAnnotate.updateLKFlowROI) {
    _odometry_LKFlow.SetROI(backAnnotate.lkFlowROI);
  }
}

void RobotOdometry::DrawTrackingVisualization() {
  TrackingWidget *trackingWidget = TrackingWidget::GetInstance();

  cv::Mat trackingMat;
  if (trackingWidget) {
    trackingMat = TrackingWidget::GetInstance()->GetTrackingMat();
  }

  if (!trackingMat.empty()) {
    // Draw robot as a square
    int size = (MIN_ROBOT_BLOB_SIZE + MAX_ROBOT_BLOB_SIZE) / 4;
    cv::rectangle(trackingMat,
                  _dataRobot.robotPosition - cv::Point2f(size, size),
                  _dataRobot.robotPosition + cv::Point2f(size, size),
                  cv::Scalar(255, 255, 255), 2);

    // Draw opponent as a circle
    size = (MIN_OPPONENT_BLOB_SIZE + MAX_OPPONENT_BLOB_SIZE) / 4;
    safe_circle(trackingMat, _dataOpponent.robotPosition, size,
                cv::Scalar(255, 255, 255), 2);
  }
}

/*
 * Returns true if the tracking data can be trusted + used for orbit + kill mode
 * False otherwise.
 *
 * Criteria are:
 * 1. Good if both blob + heuristic are valid + agree, and neural net isn't
 * running
 * 2. Good if the current selected robot pos with the neural net + neural net
 * valid + recent
 * 3. Bad otherwise
 */
bool RobotOdometry::IsTrackingGoodQuality() {
  const double AGREEMENT_DIST_THRESH_PX = 50;

  bool heuristicValid = _odometry_Heuristic.IsRunning() &&
                        _prevInputs.us_heuristic.robotPosValid &&
                        _prevInputs.us_heuristic.GetAge() < 0.3;
  bool blobValid =
      _odometry_Blob
          .IsRunning();  // don't include valid, since it might just be stopped
  // the neural is valid if it isn't too old && it's running
  bool neuralValid = _odometry_Neural.IsRunning() &&
                     _prevInputs.us_neural.robotPosValid &&
                     _prevInputs.us_neural.GetAge() < 0.1;

  // crying, return false :(
  if (!heuristicValid && !blobValid && !neuralValid) {
    return false;
  }

  // if heuristic and blob valid, but neural net isn't running
  if (heuristicValid && blobValid && !neuralValid) {
    double distBetweenRobot = cv::norm(_prevInputs.us_heuristic.robotPosition -
                                       _prevInputs.us_blob.robotPosition);
    double distBetweenOpponent =
        cv::norm(_prevInputs.them_heuristic.robotPosition -
                 _prevInputs.them_blob.robotPosition);

    // return true if they agree, false otherwise
    return distBetweenRobot < AGREEMENT_DIST_THRESH_PX &&
           distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
  }

  // if the neural net is running and predicting
  if (neuralValid) {
    // check for agreement with neural net + agreement between the other two
    // algorithms for the opponent
    double distToRobot = cv::norm(_dataRobot.robotPosition -
                                  _prevInputs.us_neural.robotPosition);
    double distBetweenOpponent =
        cv::norm(_prevInputs.them_heuristic.robotPosition -
                 _prevInputs.them_blob.robotPosition);

    return distToRobot < AGREEMENT_DIST_THRESH_PX &&
           distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
  }

  // otherwise return false, since we don't have enough confidence
  return false;
}

/**
 * @brief returns the odometry data extrapolated to current time
 */
OdometryData RobotOdometry::Robot(double currTime) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData currData = _dataRobot;
  locker.unlock();

  // extrapolate it to the current time
  return currData.ExtrapolateBoundedTo(currTime);
}

/**
 * @brief Returns the angle to add to the global angle to get the internal imu
 * angle (in radians)
 */
float RobotOdometry::GetIMUOffset() { return _odometry_IMU.GetOffset(); }

OdometryData RobotOdometry::Opponent(double currTime) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData currData = _dataOpponent;
  locker.unlock();

  // currData.ExtrapolateBoundedTo(currTime);

  return currData;
}

static double GetImuAngleRad() {
  // get angular velocity from imu
  return RobotController::GetInstance().GetIMUData().rotation;
}

static double GetImuAngleVelocityRadPerSec() {
  // get angular velocity from imu
  return RobotController::GetInstance().GetIMUData().rotationVelocity;
}

/**
 * Allows artificially setting the angle of the robot
 * This is mainly used for manual recalibration.
 *
 * @return the new angle
 */
void RobotOdometry::UpdateForceSetAngle(double newAngle, bool opponentRobot) {
  // Go through each Odometry and update it
  double currTime = Clock::programClock.getElapsedTime();
  _odometry_Blob.SetAngle(Angle(newAngle), opponentRobot, currTime, 0, true);
  _odometry_Heuristic.SetAngle(Angle(newAngle), opponentRobot, currTime, 0,
                               true);
  _odometry_IMU.SetAngle(Angle(newAngle), opponentRobot, currTime, 0, true);
  _odometry_LKFlow.SetAngle(Angle(newAngle), opponentRobot, currTime, 0, true);

  // Update our own data
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;
  odoData.SetAngle(Angle(newAngle), currTime, 0, true);
}

/**
 * Allows artificially setting the position and velocity of the robot
 * This is mainly used for manual recalibration.
 *
 * @param newPos - the new position of the robot
 * @param newVel - the new velocity of the robot
 */
void RobotOdometry::UpdateForceSetPosAndVel(cv::Point2f newPos,
                                            cv::Point2f newVel,
                                            bool opponentRobot) {
  // Go through each Odometry and update it
  _odometry_Blob.SetPosition(newPos, opponentRobot);
  _odometry_Blob.SetVelocity(newVel, opponentRobot);

  _odometry_Heuristic.SetPosition(newPos, opponentRobot);
  _odometry_Heuristic.SetVelocity(newVel, opponentRobot);

#ifdef USE_OPENCV_TRACKER
  _odometry_opencv.SetPosition(newPos, opponentRobot);
#endif

  // Update our own data
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

  odoData.robotPosition = newPos;
  odoData.robotVelocity = newVel;
  odoData.robotPosValid = true;
}

// Switch position of robots
void RobotOdometry::SwitchRobots() {
  // Switch all the Odometry
  _odometry_Blob.SwitchRobots();
  _odometry_Heuristic.SwitchRobots();

  // Update our own data
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData tempData = _dataRobot;
  _dataRobot = _dataOpponent;
  _dataOpponent = tempData;

  _dataRobot.isUs = true;
  _dataOpponent.isUs = false;
}

// Run Code
bool RobotOdometry::Run(OdometryAlg algorithm) {
  switch (algorithm) {
    case OdometryAlg::Blob:
      return _odometry_Blob.Run();

    case OdometryAlg::Heuristic:
      return _odometry_Heuristic.Run();

    case OdometryAlg::IMU:
      return _odometry_IMU.Run();

    case OdometryAlg::Neural:
      return _odometry_Neural.Run();

    case OdometryAlg::Human:
      return _odometry_Human.Run() && _odometry_Human_Heuristic.Run();

    case OdometryAlg::NeuralRot:
      return _odometry_NeuralRot.Run();

    case OdometryAlg::LKFlow:
      return _odometry_LKFlow.Run();

#ifdef USE_OPENCV_TRACKER
    case OdometryAlg::OpenCV:
      return _odometry_opencv.Run();
#endif
    default:
      break;
  }

  return false;
}

// Stop Code
bool RobotOdometry::Stop(OdometryAlg algorithm) {
  switch (algorithm) {
    case OdometryAlg::Blob:
      return _odometry_Blob.Stop();

    case OdometryAlg::Heuristic:
      return _odometry_Heuristic.Stop();

    case OdometryAlg::IMU:
      return _odometry_IMU.Stop();

    case OdometryAlg::Neural:
      return _odometry_Neural.Stop();

    case OdometryAlg::Human:
      return _odometry_Human.Stop() && _odometry_Human_Heuristic.Stop();

    case OdometryAlg::NeuralRot:
      return _odometry_NeuralRot.Stop();

    case OdometryAlg::LKFlow:
      return _odometry_LKFlow.Stop();

#ifdef USE_OPENCV_TRACKER
    case OdometryAlg::OpenCV:
      return _odometry_opencv.Stop();
#endif
  }

  return false;
}

// IsRunning Code
bool RobotOdometry::IsRunning(OdometryAlg algorithm) {
  switch (algorithm) {
    case OdometryAlg::Blob:
      return _odometry_Blob.IsRunning();

    case OdometryAlg::Heuristic:
      return _odometry_Heuristic.IsRunning();

    case OdometryAlg::IMU:
      return _odometry_IMU.IsRunning();

    case OdometryAlg::Neural:
      return _odometry_Neural.IsRunning();

    case OdometryAlg::Human:
      return _odometry_Human.IsRunning() &&
             _odometry_Human_Heuristic.IsRunning();

    case OdometryAlg::NeuralRot:
      return _odometry_NeuralRot.IsRunning();

    case OdometryAlg::LKFlow:
      return _odometry_LKFlow.IsRunning();

#ifdef USE_OPENCV_TRACKER
    case OdometryAlg::OpenCV:
      return _odometry_opencv.IsRunning();
#endif
    default:
      break;
  }

  return false;
}

HeuristicOdometry &RobotOdometry::GetHeuristicOdometry() {
  return _odometry_Heuristic;
}

OdometryIMU &RobotOdometry::GetIMUOdometry() { return _odometry_IMU; }

CVPosition &RobotOdometry::GetNeuralOdometry() { return _odometry_Neural; }

BlobDetection &RobotOdometry::GetBlobOdometry() { return _odometry_Blob; }

CVRotation &RobotOdometry::GetNeuralRotOdometry() {
  return _odometry_NeuralRot;
}

LKFlowTracker &RobotOdometry::GetLKFlowOdometry() { return _odometry_LKFlow; }

#ifdef USE_OPENCV_TRACKER
OpenCVTracker &RobotOdometry::GetOpenCVOdometry() { return _odometry_opencv; }
#endif

std::string getCurrentDateTime() {
  std::time_t now = std::time(nullptr);
  char time_str[20];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S",
                std::localtime(&now));
  return std::string(time_str);
}

void RobotOdometry::LogOdometryToFile(std::string &extraHeader,
                                      std::string &extraData) {
  if (!LOG_ODOMETRY_DATA) {
    if (_logOdometryFile.is_open()) {
      // Close the file
      _logOdometryFileOpen = false;
      _logOdometryFile.close();
    }

    return;
  }

  if (LOG_ODOMETRY_DATA && !_logOdometryFileOpen) {
    // Open the file in append mode
    _logOdometryFileOpen = true;
    _logOdometryFile.open(_logOdometryFileName, std::ios::app);

    // Add header
    if (_logOdometryFile.is_open()) {
      _logOdometryFile
          << std::endl
          << "[" << getCurrentDateTime() << "]" << std::endl
          << extraHeader << ","
          << GetOdometryLog("UsHeu", _prevInputs.us_heuristic, true).str()
          << "," << GetOdometryLog("UsBlob", _prevInputs.us_blob, true).str()
          << ","
          << GetOdometryLog("UsNeural", _prevInputs.us_neural, true).str()
          << ","
          << GetOdometryLog("UsNeuralRot", _prevInputs.us_neuralrot, true).str()
          << "," << GetOdometryLog("UsIMU", _prevInputs.us_imu, true).str()
          << "," << GetOdometryLog("UsHuman", _prevInputs.us_human, true).str()
          << ","
          << GetOdometryLog("ThemHeu", _prevInputs.them_heuristic, true).str()
          << ","
          << GetOdometryLog("ThemBlob", _prevInputs.them_blob, true).str()
          << ","
          << GetOdometryLog("ThemHuman", _prevInputs.them_human, true).str()
          << std::endl;
    }
  }

  // Add current Data to the file
  if (_logOdometryFile.is_open()) {
    // Write the data to the file
    _logOdometryFile
        << extraData << ","
        << GetOdometryLog("UsHeu", _prevInputs.us_heuristic).str() << ","
        << GetOdometryLog("UsBlob", _prevInputs.us_blob).str() << ","
        << GetOdometryLog("UsNeural", _prevInputs.us_neural).str() << ","
        << GetOdometryLog("UsNeuralRot", _prevInputs.us_neuralrot).str() << ","
        << GetOdometryLog("UsIMU", _prevInputs.us_imu).str() << ","
        << GetOdometryLog("UsHuman", _prevInputs.us_human).str() << ","
        << GetOdometryLog("ThemHeu", _prevInputs.them_heuristic).str() << ","
        << GetOdometryLog("ThemBlob", _prevInputs.them_blob).str() << ","
        << GetOdometryLog("ThemHuman", _prevInputs.them_human).str()
        << std::endl;
  }
}

std::stringstream RobotOdometry::GetOdometryLog(const std::string &name,
                                                OdometryData &odometry,
                                                bool doheader) {
  std::stringstream ss;
  if (doheader) {
    ss << name
       << ",PosValid,AngleValid,frameID,frameTime,PosX,PosY,Angle,VelX,VelY,"
          "VelMag,VelAng,AVel";
  } else {
    ss << std::fixed << std::setprecision(2) << name << ","
       << ((odometry.robotPosValid) ? "1" : "0") << ","
       << ((odometry.IsAngleValid()) ? "1" : "0") << "," << odometry.frameID
       << "," << odometry.time << "," << odometry.robotPosition.x << ","
       << odometry.robotPosition.y << "," << odometry.GetAngle() << ","
       << odometry.robotVelocity.x << "," << odometry.robotVelocity.y << ","
       << std::sqrt(odometry.robotVelocity.x * odometry.robotVelocity.x +
                    odometry.robotVelocity.y * odometry.robotVelocity.y)
       << "," << std::atan2(odometry.robotVelocity.y, odometry.robotVelocity.x)
       << "," << odometry.GetAngleVelocity();
  }

  return ss;
}

/**
 * @brief Forces the position of a tracking algorithm to be a certain value
 *
 * @param alg - the algorithm to set the position of
 * @param pos - the position to set it to
 * @param opponent - whether to set the opponent or not
 */
void RobotOdometry::ForceSetPositionOfAlg(OdometryAlg alg, cv::Point2f pos,
                                          bool opponent) {
  if (alg == OdometryAlg::Blob) {
    _odometry_Blob.SetPosition(pos, opponent);
  } else if (alg == OdometryAlg::Heuristic) {
    _odometry_Heuristic.SetPosition(pos, opponent);
  } else if (alg == OdometryAlg::IMU) {
    _odometry_IMU.SetPosition(pos, opponent);
  } else if (alg == OdometryAlg::Neural) {
    _odometry_Neural.SetPosition(pos, opponent);
  }
#ifdef USE_OPENCV_TRACKER
  else if (alg == OdometryAlg::OpenCV) {
    _odometry_opencv.SetPosition(pos, opponent);
  }
#endif
}

/**
 * @brief Forces the velocity of a tracking algorithm to be a certain value
 *
 * @param alg - the algorithm to set the velocity of
 * @param vel - the velocity to set it to
 * @param opponent - whether to set the opponent or not
 */
void RobotOdometry::ForceSetVelocityOfAlg(OdometryAlg alg, cv::Point2f vel,
                                          bool opponent) {
  if (alg == OdometryAlg::Blob) {
    _odometry_Blob.SetVelocity(vel, opponent);
  } else if (alg == OdometryAlg::Heuristic) {
    _odometry_Heuristic.SetVelocity(vel, opponent);
  } else if (alg == OdometryAlg::IMU) {
    std::cerr << "ERROR: Cannot set velocity for IMU" << std::endl;
  } else if (alg == OdometryAlg::Neural) {
    _odometry_Neural.SetVelocity(vel, opponent);
  } else if (alg == OdometryAlg::NeuralRot) {
    std::cerr << "ERROR: Cannot set velocity for NeuralRot" << std::endl;
  }
#ifdef USE_OPENCV_TRACKER
  else if (alg == OdometryAlg::OpenCV) {
    std::cerr << "ERROR: Cannot set velocity for opencv" << std::endl;
  }
#endif
}

std::string RobotOdometry::GetDebugString() {
  std::unique_lock<std::mutex> locker(_mutexDebugImage);
  std::string debugCopy = _debugString;
  locker.unlock();
  return debugCopy;
}

void RobotOdometry::GetDebugImage(cv::Mat &debugImage, cv::Point offset) {
  // This should not happen, but in case it does, we will create an empty image
  if (debugImage.empty()) {
    debugImage = cv::Mat(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0));
  }

  debugImage = cv::Mat::zeros(debugImage.size(),
                              debugImage.type());  // Clear the target image

  // X-coordinates for left and right columns
  const int leftX = 10 + offset.x;  // Left column for Robot Data

  // Draw robot data (top-left)
  int yLeft = 20 + offset.y;  // Start at top
  printText("Final Robot Data:", debugImage, yLeft, leftX);
  _dataRobot.GetDebugImage(debugImage, cv::Point(leftX + 10, yLeft + 14));

  yLeft = 20 + offset.y;  // Reset to top

  // Draw opponent data next to it
  printText("Final Opp Data:", debugImage, yLeft, leftX + 230);
  _dataOpponent.GetDebugImage(debugImage,
                              cv::Point(leftX + 10 + 230, yLeft + 14));

  yLeft += 140;  // Move down for the next section

  // Check if debugImage is empty    // Get unique access to _debugImage
  std::unique_lock<std::mutex> locker(_mutexDebugImage);

  // Add debug string
  printText(_debugString, debugImage, yLeft, offset.x);

  locker.unlock();  // Unlock mutex after operation
}
