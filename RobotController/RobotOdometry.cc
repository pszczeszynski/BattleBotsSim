#include "RobotOdometry.h"

#include <optional>

#include "../Common/Communication.h"
#include "Globals.h"
#include "Input/InputState.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "SafeDrawing.h"
#include "imgui.h"
#include "odometry/Neural/CVPosition.h"

namespace {
enum FUSION_SM {
  FUSION_NORMAL = 0,  // Normal State
  FUSION_WAIT_FOR_BG  // Wait for background to stabilize
};

FUSION_SM fusionStateMachine = FUSION_NORMAL;

constexpr double _dataAgeThreshold = 0.1;
template <typename T>
bool isFresh(const std::optional<T> &opt) {
  return opt && opt->GetAge() < _dataAgeThreshold;
}
}  // namespace

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

  if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftArrow) &&
      _dataRobot.angle.has_value()) {
    UpdateForceSetAngle(_dataRobot.angle.value().angle - angleUserAdjust,
                        false);
  } else if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightArrow) &&
             _dataRobot.angle.has_value()) {
    UpdateForceSetAngle(_dataRobot.angle.value().angle + angleUserAdjust,
                        false);
  }

  // opponent with up and 1 and 3
  if (InputState::GetInstance().IsKeyDown(ImGuiKey_1) &&
      _dataOpponent.angle.has_value()) {
    UpdateForceSetAngle(_dataOpponent.angle.value().angle - angleUserAdjust,
                        true);
  } else if (InputState::GetInstance().IsKeyDown(ImGuiKey_3)) {
    UpdateForceSetAngle(_dataOpponent.angle.value().angle + angleUserAdjust,
                        true);
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
  _dataRobot.pos = PositionData(TrackingWidget::robotMouseClickPoint,
                                cv::Point2f(0, 0), currTime);
  _dataRobot.angle =
      AngleData(Angle(TrackingWidget::robotMouseClickAngle), 0, currTime);

  _dataOpponent.pos = PositionData(TrackingWidget::opponentMouseClickPoint,
                                   cv::Point2f(0, 0), currTime);
  _dataOpponent.angle =
      AngleData(Angle(TrackingWidget::opponentMouseClickAngle), 0, currTime);

  if (_dataRobot.angle.has_value()) {
    _odometry_Heuristic.SetAngle(_dataRobot.angle.value(), false);
  }
  if (_dataRobot.pos.has_value()) {
    _odometry_Heuristic.SetVelocity(_dataRobot.pos.value().velocity, false);
  }
  if (_dataOpponent.angle.has_value()) {
    _odometry_Heuristic.SetAngle(_dataOpponent.angle.value(), true);
  }

  if (_dataRobot.pos.has_value()) {
    _odometry_Blob.ForcePosition(_dataRobot.pos.value().position, false);
  }
  if (_dataRobot.angle.has_value()) {
    _odometry_Blob.SetAngle(_dataRobot.angle.value(), false);
  }
  if (_dataOpponent.pos.has_value()) {
    _odometry_Blob.ForcePosition(_dataOpponent.pos.value().position, true);
  }
  if (_dataOpponent.angle.has_value()) {
    _odometry_Blob.SetAngle(_dataOpponent.angle.value(), true);
  }

  // Set ROI for LKFlowTracker (only tracks opponent)
  if (_odometry_LKFlow.IsRunning()) {
    int roiSize = 20;
    cv::Rect roi(static_cast<int>(_dataOpponent.GetPositionOrZero().x -
                                  static_cast<float>(roiSize) / 2.0f),
                 static_cast<int>(_dataOpponent.GetPositionOrZero().y -
                                  static_cast<float>(roiSize) / 2.0f),
                 roiSize, roiSize);
    _odometry_LKFlow.SetROI(roi);
  }

#ifdef USE_OPENCV_TRACKER
  if (_dataOpponent.pos.has_value()) {
    _odometry_opencv.ForcePosition(_dataOpponent.pos.value().position, true);
  }
  if (_dataOpponent.angle.has_value()) {
    _odometry_opencv.SetAngle(_dataOpponent.angle.value(), true);
  }
  if (_dataOpponent.pos.has_value()) {
    _odometry_opencv.SetVelocity(_dataOpponent.pos.value().velocity, true);
  }
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

  if (fusionStateMachine == FUSION_WAIT_FOR_BG) {
    // If heuristic is not active, leave this state
    if (!_odometry_Heuristic.IsRunning()) {
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
}

FusionOutput RobotOdometry::Fuse(const RawInputs &inputs, double now,
                                 const OdometryData &prevRobot,
                                 const OdometryData &prevOpponent) {
  FusionOutput output{};

  // Initialize output with previous state (for extrapolation baseline)
  output.robot = prevRobot;
  output.opponent = prevOpponent;

  // Extrapolate input data to current time
  OdometryData ext_dataRobot_Blob = inputs.us_blob.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_Heuristic =
      inputs.us_heuristic.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_Neural = inputs.us_neural;
  OdometryData ext_dataRobot_NeuralRot =
      inputs.us_neuralrot.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_IMU = inputs.us_imu.ExtrapolateBoundedTo(now);
  OdometryData ext_dataRobot_Human = inputs.us_human.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_Human =
      inputs.them_human.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_Blob =
      inputs.them_blob.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_Heuristic =
      inputs.them_heuristic.ExtrapolateBoundedTo(now);
  OdometryData ext_dataOpponent_LKFlow =
      inputs.them_lkflow.ExtrapolateBoundedTo(now);

  // If we determine a force position is required, mark it using this variable
  bool forceUsPosition = false;

  // Opponent rotation
  if (isFresh(inputs.them_human.angle) && inputs.them_human_is_new) {
    output.opponent.angle = ext_dataOpponent_Human.angle;
    output.opponent.angle.value().velocity = 0;

    output.backAnnotate.setOpponentAngle_Heuristic = true;
    output.backAnnotate.setOpponentAngle_Blob = true;
    output.backAnnotate.setOpponentAngle_LKFlow = true;
  }

  // G1) if Neural US = Heuristic.them: SWAP Heuristic
  if (isFresh(inputs.us_neural.pos) && isFresh(inputs.them_heuristic.pos) &&
      ext_dataRobot_Neural.pos.has_value() &&
      ext_dataOpponent_Heuristic.IsPointInside(
          ext_dataRobot_Neural.pos.value().position)) {
    output.backAnnotate.swapHeuristic = true;

    ext_dataRobot_Heuristic.pos.reset();
    ext_dataRobot_Heuristic.angle.reset();
    ext_dataOpponent_Heuristic.pos.reset();
    ext_dataOpponent_Heuristic.angle.reset();
  }

  // Same check for Blob
  if (isFresh(inputs.us_neural.pos) && isFresh(inputs.them_blob.pos) &&
      ext_dataOpponent_Blob.IsPointInside(
          ext_dataRobot_Neural.pos.value().position)) {
    // Mark for swap (will be applied in back-annotation)
    output.backAnnotate.swapBlob = true;

    ext_dataRobot_Blob.pos.reset();
    ext_dataRobot_Blob.angle.reset();
    ext_dataOpponent_Blob.pos.reset();
    ext_dataOpponent_Blob.angle.reset();
  }

  // G2) if Neural != Heuristic.us && Neural=Blob.us: Heuristic.ForceUs(Neural)
  if (isFresh(ext_dataRobot_Neural.pos) && isFresh(ext_dataRobot_Blob.pos) &&
      ext_dataRobot_Blob.IsPointInside(
          ext_dataRobot_Neural.pos.value().position)) {
    // Force position if heuristic doesn't agree
    if (!isFresh(ext_dataRobot_Heuristic.pos) ||
        !ext_dataRobot_Heuristic.IsPointInside(
            ext_dataRobot_Neural.pos.value().position)) {
      ext_dataRobot_Heuristic.pos.reset();
      ext_dataRobot_Heuristic.angle.reset();

      output.robot.pos = PositionData(ext_dataRobot_Neural.pos.value().position,
                                      ext_dataRobot_Neural.pos.value().velocity,
                                      ext_dataRobot_Neural.pos.value().time);
      if (ext_dataRobot_Neural.pos.value().rect.has_value()) {
        output.robot.pos.value().rect = ext_dataRobot_Neural.pos.value().rect;
      }

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

  if (isFresh(ext_dataRobot_Heuristic.pos)) {
    output.robot.pos =
        PositionData(ext_dataRobot_Heuristic.pos.value().position,
                     ext_dataRobot_Heuristic.pos.value().velocity,
                     ext_dataRobot_Heuristic.pos.value().time);
    if (ext_dataRobot_Heuristic.pos.value().rect.has_value()) {
      output.robot.pos.value().rect = ext_dataRobot_Heuristic.pos.value().rect;
    }

    // If blob position isn't inside our rectangle then set position
    if (isFresh(ext_dataRobot_Blob.pos) &&
        !ext_dataRobot_Heuristic.IsPointInside(
            ext_dataRobot_Blob.pos.value().position)) {
      ext_dataRobot_Blob.pos.reset();
    }
  } else if (isFresh(ext_dataRobot_Neural.pos)) {
    output.robot.pos = PositionData(ext_dataRobot_Neural.pos.value().position,
                                    ext_dataRobot_Neural.pos.value().velocity,
                                    ext_dataRobot_Neural.pos.value().time);
    if (ext_dataRobot_Neural.pos.value().rect.has_value()) {
      output.robot.pos.value().rect = ext_dataRobot_Neural.pos.value().rect;
    }
  } else if (isFresh(ext_dataRobot_Blob.pos)) {
    output.robot.pos = PositionData(ext_dataRobot_Blob.pos.value().position,
                                    ext_dataRobot_Blob.pos.value().velocity,
                                    ext_dataRobot_Blob.pos.value().time);
    if (ext_dataRobot_Blob.pos.value().rect.has_value()) {
      output.robot.pos.value().rect = ext_dataRobot_Blob.pos.value().rect;
    }
  }

  //  US VEL:
  //           Rule: 1) Heuristic, 2) Blob
  if (isFresh(ext_dataRobot_Heuristic.pos) && output.robot.pos.has_value()) {
    output.robot.pos.value().velocity =
        ext_dataRobot_Heuristic.pos.value().velocity;
  } else if (isFresh(ext_dataRobot_Blob.pos) && output.robot.pos.has_value()) {
    output.robot.pos.value().velocity = ext_dataRobot_Blob.pos.value().velocity;
  }

  //  US ROT:
  //            Rule: 1) IMU (Neural is already fused), 2) Neural Rot 3)
  //            Heuristic 4) Blob
  if (isFresh(ext_dataRobot_IMU.angle)) {
    output.robot.angle = AngleData(ext_dataRobot_IMU.angle.value().angle,
                                   ext_dataRobot_IMU.angle.value().velocity,
                                   ext_dataRobot_IMU.angle.value().time);

    // Reset heuristic angle calculation
    ext_dataRobot_Heuristic.angle.reset();
    ext_dataRobot_Blob.angle.reset();
  } else if (isFresh(ext_dataRobot_NeuralRot.angle)) {
    // TODO: decide if we want to use neuralRot velocity
    // Currently uses heuristic's angular velocity or otherwise 0
    double angleVelocity = ext_dataRobot_Heuristic.angle.has_value()
                               ? ext_dataRobot_Heuristic.angle.value().velocity
                               : 0;
    output.robot.angle =
        AngleData(ext_dataRobot_NeuralRot.angle.value().angle, angleVelocity,
                  ext_dataRobot_NeuralRot.angle.value().time);

    // Set heuristor to neural
    ext_dataRobot_Heuristic.angle.reset();
    ext_dataRobot_Blob.angle.reset();
  } else if (isFresh(ext_dataRobot_Blob.angle)) {
    output.robot.angle = AngleData(ext_dataRobot_Blob.angle.value().angle,
                                   ext_dataRobot_Blob.angle.value().velocity,
                                   ext_dataRobot_Blob.angle.value().time);
  }

  //  THEM POS:
  //            Rule: 1) Heuristic, 2) Blob
  //            Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold,
  //            setpos on blob If Heuristic.invalid, setpos(blob)
  if (isFresh(ext_dataOpponent_Heuristic.pos)) {
    output.opponent.pos =
        PositionData(ext_dataOpponent_Heuristic.pos.value().position,
                     ext_dataOpponent_Heuristic.pos.value().velocity,
                     ext_dataOpponent_Heuristic.pos.value().time);
    if (ext_dataOpponent_Heuristic.pos.value().rect.has_value()) {
      output.opponent.pos.value().rect =
          ext_dataOpponent_Heuristic.pos.value().rect;
    }

    // If blob position isn't inside our rectangle then set position
    if (isFresh(ext_dataOpponent_Blob.pos) &&
        !ext_dataOpponent_Heuristic.IsPointInside(
            ext_dataOpponent_Blob.pos.value().position)) {
      ext_dataOpponent_Blob.pos.reset();
    }
  } else if (isFresh(ext_dataOpponent_Blob.pos)) {
    output.opponent.pos =
        PositionData(ext_dataOpponent_Blob.pos.value().position,
                     ext_dataOpponent_Blob.pos.value().velocity,
                     ext_dataOpponent_Blob.pos.value().time);
    if (ext_dataOpponent_Blob.pos.value().rect.has_value()) {
      output.opponent.pos.value().rect = ext_dataOpponent_Blob.pos.value().rect;
    }

    ext_dataOpponent_Heuristic.pos.reset();
  }

  // Set ROI for LKFlowTracker when opponent position is finalized
  if (output.opponent.pos.has_value() && _odometry_LKFlow.IsRunning()) {
    // Use opponent blob size to determine ROI size, or default to 140x140
    int roiSize = 70;

    cv::Rect roi(static_cast<int>(output.opponent.pos.value().position.x -
                                  static_cast<float>(roiSize) / 2.0f),
                 static_cast<int>(output.opponent.pos.value().position.y -
                                  static_cast<float>(roiSize) / 2.0f),
                 roiSize, roiSize);
    // Mark that we need to update the ROI
    output.backAnnotate.updateLKFlowROI = true;
    output.backAnnotate.lkFlowROI = roi;
  }

  //  THEM VEL:
  //            Rule: 1) Heuristic, 2) Blob
  if (isFresh(ext_dataOpponent_Heuristic.pos)) {
    if (output.opponent.pos.has_value()) {
      output.opponent.pos.value().velocity =
          ext_dataOpponent_Heuristic.pos.value().velocity;
    }
  } else if (isFresh(ext_dataOpponent_Blob.pos)) {
    if (output.opponent.pos.has_value()) {
      output.opponent.pos.value().velocity =
          ext_dataOpponent_Blob.pos.value().velocity;
    }
  }

  //  THEM ROT:
  //            Rule: 1) LKFlow, 2) Human, 3) Heuristic, 4) Blob
  if (isFresh(ext_dataOpponent_LKFlow.angle)) {
    output.opponent.angle =
        AngleData(ext_dataOpponent_LKFlow.angle.value().angle,
                  ext_dataOpponent_LKFlow.angle.value().velocity,
                  ext_dataOpponent_LKFlow.angle.value().time);
  } else if (isFresh(ext_dataOpponent_Human.angle)) {
    output.opponent.angle =
        AngleData(ext_dataOpponent_Human.angle.value().angle,
                  ext_dataOpponent_Human.angle.value().velocity,
                  ext_dataOpponent_Human.angle.value().time);
  } else if (isFresh(ext_dataOpponent_Blob.angle)) {
    output.opponent.angle =
        AngleData(ext_dataOpponent_Blob.angle.value().angle,
                  ext_dataOpponent_Blob.angle.value().velocity,
                  ext_dataOpponent_Blob.angle.value().time);
  }

  // *********************************************
  // Back Annotate data to invalid readings
  // *********************************************

  // Robot Position and velocity:
  //   Heuristic, Blob need back-annotated robot positions and velocities
  if (isFresh(output.robot.pos)) {
    if (!isFresh(ext_dataRobot_Heuristic.pos)) {
      output.backAnnotate.setRobotPos_Heuristic = true;
    }
    if (!isFresh(ext_dataRobot_Blob.pos)) {
      output.backAnnotate.setRobotPos_Blob = true;
    }
  }

  // Robot Angle + velocity:
  // Heuristic, Blob will need it back annotated
  if (isFresh(output.robot.angle)) {
    if (!isFresh(ext_dataRobot_Heuristic.angle)) {
      output.backAnnotate.setRobotAngle_Heuristic = true;
    }
    if (!isFresh(ext_dataRobot_Blob.angle)) {
      output.backAnnotate.setRobotAngle_Blob = true;
    }
  }

  // OPPONENT same things:
  if (isFresh(output.opponent.pos)) {
    if (!isFresh(ext_dataOpponent_Heuristic.pos)) {
      output.backAnnotate.setOpponentPos_Heuristic = true;
    }
    if (!isFresh(ext_dataOpponent_Blob.pos)) {
      output.backAnnotate.setOpponentPos_Blob = true;
    }
  }

  if (isFresh(output.opponent.angle)) {
    if (!isFresh(ext_dataOpponent_Heuristic.angle)) {
      output.backAnnotate.setOpponentAngle_Heuristic = true;
    }
    if (!isFresh(ext_dataOpponent_Blob.angle)) {
      output.backAnnotate.setOpponentAngle_Blob = true;
    }
    if (!isFresh(ext_dataOpponent_LKFlow.angle)) {
      output.backAnnotate.setOpponentAngle_LKFlow = true;
    }
  }

#ifdef FORCE_SIM_DATA
  // Use thread-safe SimulationState instead of global variables
  auto robotData = SimulationState::GetInstance().GetRobotData();
  auto opponentData = SimulationState::GetInstance().GetOpponentData();

  output.robot.pos =
      PositionData(robotData.position, robotData.velocity, robotData.timestamp);
  output.opponent.pos = PositionData(
      opponentData.position, opponentData.velocity, opponentData.timestamp);
  output.opponent.angle =
      AngleData(Angle(opponentData.rotation), opponentData.rotationVelocity,
                opponentData.timestamp);
#endif

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
  if (backAnnotate.setRobotPos_Heuristic && robot.pos.has_value()) {
    if (backAnnotate.forceRobotPos_Heuristic) {
      _odometry_Heuristic.ForcePosition(robot.pos.value().position, false);
    } else {
      _odometry_Heuristic.SetPosition(robot.pos.value().position, false);
    }
    _odometry_Heuristic.SetVelocity(robot.pos.value().velocity, false);
  }

  // Robot Position and velocity - Blob
  if (backAnnotate.setRobotPos_Blob && robot.pos.has_value()) {
    if (backAnnotate.forceRobotPos_Blob) {
      _odometry_Blob.ForcePosition(robot.pos.value().position, false);
    } else {
      _odometry_Blob.SetPosition(robot.pos.value().position, false);
    }
    _odometry_Blob.SetVelocity(robot.pos.value().velocity, false);
  }

  // Robot Angle - Heuristic
  if (backAnnotate.setRobotAngle_Heuristic && robot.angle.has_value()) {
    _odometry_Heuristic.SetAngle(robot.angle.value(), false);
  }

  // Robot Angle - Blob
  if (backAnnotate.setRobotAngle_Blob && robot.angle.has_value()) {
    _odometry_Blob.SetAngle(robot.angle.value(), false);
  }

  // Opponent Position and velocity - Heuristic
  if (backAnnotate.setOpponentPos_Heuristic && opponent.pos.has_value()) {
    _odometry_Heuristic.SetPosition(opponent.pos.value().position, true);
    _odometry_Heuristic.SetVelocity(opponent.pos.value().velocity, true);
  }

  // Opponent Position and velocity - Blob
  if (backAnnotate.setOpponentPos_Blob && opponent.pos.has_value()) {
    _odometry_Blob.SetPosition(opponent.pos.value().position, true);
    _odometry_Blob.SetVelocity(opponent.pos.value().velocity, true);
  }

  // Opponent Angle - Heuristic
  if (backAnnotate.setOpponentAngle_Heuristic && opponent.angle.has_value()) {
    _odometry_Heuristic.SetAngle(opponent.angle.value(), true);
  }

  // Opponent Angle - Blob
  if (backAnnotate.setOpponentAngle_Blob && opponent.angle.has_value()) {
    _odometry_Blob.SetAngle(opponent.angle.value(), true);
  }

  // Opponent Angle - LKFlow
  if (backAnnotate.setOpponentAngle_LKFlow && opponent.angle.has_value()) {
    _odometry_LKFlow.SetAngle(opponent.angle.value(), true);
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
    if (_dataRobot.pos.has_value()) {
      cv::rectangle(trackingMat,
                    _dataRobot.pos.value().position - cv::Point2f(size, size),
                    _dataRobot.pos.value().position + cv::Point2f(size, size),
                    cv::Scalar(255, 255, 255), 2);
    }

    if (_dataOpponent.pos.has_value()) {
      // Draw opponent as a circle
      size = (MIN_OPPONENT_BLOB_SIZE + MAX_OPPONENT_BLOB_SIZE) / 4;
      safe_circle(trackingMat, _dataOpponent.pos.value().position, size,
                  cv::Scalar(255, 255, 255), 2);
    }
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
                        _prevInputs.us_heuristic.pos.has_value() &&
                        _prevInputs.us_heuristic.pos.value().GetAge() < 0.3;
  bool blobValid =
      _odometry_Blob
          .IsRunning();  // don't include valid, since it might just be stopped
  // the neural is valid if it isn't too old && it's running
  bool neuralValid = _odometry_Neural.IsRunning() &&
                     _prevInputs.us_neural.pos.has_value() &&
                     _prevInputs.us_neural.pos.value().GetAge() < 0.1;

  // crying, return false :(
  if (!heuristicValid && !blobValid && !neuralValid) {
    return false;
  }

  // if heuristic and blob valid, but neural net isn't running
  if (heuristicValid && blobValid && !neuralValid &&
      _prevInputs.us_heuristic.pos.has_value()) {
    double distBetweenRobot =
        cv::norm(_prevInputs.us_heuristic.pos.value().position -
                 _prevInputs.us_blob.pos.value().position);
    double distBetweenOpponent =
        cv::norm(_prevInputs.them_heuristic.pos.value().position -
                 _prevInputs.them_blob.pos.value().position);

    // return true if they agree, false otherwise
    return distBetweenRobot < AGREEMENT_DIST_THRESH_PX &&
           distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
  }

  // if the neural net is running and predicting
  if (neuralValid && _dataRobot.pos.has_value() &&
      _prevInputs.us_neural.pos.has_value()) {
    // check for agreement with neural net + agreement between the other two
    // algorithms for the opponent
    double distToRobot = cv::norm(_dataRobot.pos.value().position -
                                  _prevInputs.us_neural.pos.value().position);
    double distBetweenOpponent =
        cv::norm(_prevInputs.them_heuristic.pos.value().position -
                 _prevInputs.them_blob.pos.value().position);

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
  AngleData angleData(Angle(newAngle), 0, currTime);
  _odometry_Blob.SetAngle(angleData, opponentRobot);
  _odometry_Heuristic.SetAngle(angleData, opponentRobot);
  _odometry_IMU.SetAngle(angleData, opponentRobot);
  _odometry_LKFlow.SetAngle(angleData, opponentRobot);

  // Update our own data
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;
  odoData.angle = angleData;
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

  odoData.pos =
      PositionData(newPos, newVel, Clock::programClock.getElapsedTime());
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
       << ((odometry.pos.has_value()) ? "1" : "0") << ","
       << ((odometry.angle.has_value()) ? "1" : "0") << "," << odometry.id
       << "," << (odometry.pos.has_value() ? odometry.pos.value().time : 0)
       << ","
       << (odometry.pos.has_value() ? odometry.pos.value().position.x : -1)
       << ","
       << (odometry.pos.has_value() ? odometry.pos.value().position.y : -1)
       << ","
       << (odometry.angle.has_value() ? odometry.angle.value().angle : Angle(0))
       << ","
       << (odometry.pos.has_value() ? odometry.pos.value().velocity.x : 0)
       << ","
       << (odometry.pos.has_value() ? odometry.pos.value().velocity.y : 0)
       << ","
       << (odometry.pos.has_value()
               ? std::sqrt(odometry.pos.value().velocity.x *
                               odometry.pos.value().velocity.x +
                           odometry.pos.value().velocity.y *
                               odometry.pos.value().velocity.y)
               : 0)
       << ","
       << (odometry.pos.has_value()
               ? std::atan2(odometry.pos.value().velocity.y,
                            odometry.pos.value().velocity.x)
               : 0)
       << ","
       << (odometry.angle.has_value() ? odometry.angle.value().velocity : 0);
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
