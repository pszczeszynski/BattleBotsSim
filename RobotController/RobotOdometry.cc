#include "RobotOdometry.h"

#include <optional>

#include "../Common/Communication.h"
#include "Globals.h"
#include "Input/InputState.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "imgui.h"
#include "odometry/Neural/CVPosition.h"
#include "SimulationState.h"

namespace {
enum FUSION_SM {
  FUSION_NORMAL = 0,  // Normal State
  FUSION_WAIT_FOR_BG  // Wait for background to stabilize
};

FUSION_SM fusionStateMachine = FUSION_NORMAL;

constexpr double _dataAgeThreshold = 0.1;
template <typename T>
bool isFresh(const std::optional<T> &opt) {
  return opt.has_value() && opt->GetAge() < _dataAgeThreshold;
}

// If blob position moved much faster than LK flow (same target), disqualify
// blob.
void MaybeDisqualifyBlobByLKFlow(
    const std::optional<PositionData> &curr_lk_pos,
    const std::optional<PositionData> &prev_lk_pos,
    const std::optional<PositionData> &curr_blob_pos,
    const std::optional<PositionData> &prev_blob_pos,
    std::optional<PositionData> &blob_pos_out,
    std::optional<AngleData> &blob_angle_out,
    double speedRatioThreshold = 1.5) {
  if (!isFresh(curr_lk_pos) || !prev_lk_pos.has_value() ||
      !isFresh(curr_blob_pos) || !prev_blob_pos.has_value()) {
    return;
  }
  double delta_time_lk = curr_lk_pos->time - prev_lk_pos->time;
  double delta_time_blob = curr_blob_pos->time - prev_blob_pos->time;
  if (delta_time_lk <= 0 || delta_time_blob <= 0) return;

  double lk_speed =
      cv::norm(curr_lk_pos->position - prev_lk_pos->position) / delta_time_lk;
  double blob_speed =
      cv::norm(curr_blob_pos->position - prev_blob_pos->position) /
      delta_time_blob;

  if (blob_speed > lk_speed * speedRatioThreshold) {
    std::cout << "disqualifying blob because it changed too much" << std::endl;
    blob_pos_out.reset();
    blob_angle_out.reset();
  }
}

constexpr double kMinSeparationToUseBlob = 125;

void MaybeDisqualifyBlobByDistance(OdometryData &usBlob, OdometryData &themBlob,
                                   const OdometryData &prevRobot,
                                   const OdometryData &prevOpponent) {
  if (!prevRobot.pos.has_value() || !prevOpponent.pos.has_value()) {
    return;
  }

  double distBetweenPrev = cv::norm(prevRobot.pos.value().position -
                                    prevOpponent.pos.value().position);

  if (distBetweenPrev < kMinSeparationToUseBlob) {
    usBlob.pos.reset();
    usBlob.angle.reset();
    themBlob.pos.reset();
    themBlob.angle.reset();
  }
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
      _odometry_LKFlow(&videoSource),
      _odometry_Override(),
      _odometry_opencv(&videoSource),
      _poller(_odometry_Blob, _odometry_Heuristic, _odometry_Neural,
              _odometry_NeuralRot, _odometry_IMU, _odometry_Human,
              _odometry_Human_Heuristic, _odometry_LKFlow, _odometry_Override,
              _odometry_opencv) {
  _InitAlgorithmTable();
  Run(OdometryAlg::ManualOverride);
}

void RobotOdometry::_InitAlgorithmTable() {
  _algorithms[OdometryAlg::Blob] = &_odometry_Blob;
  _algorithms[OdometryAlg::Heuristic] = &_odometry_Heuristic;
  _algorithms[OdometryAlg::IMU] = &_odometry_IMU;
  _algorithms[OdometryAlg::Neural] = &_odometry_Neural;
  _algorithms[OdometryAlg::Human] = &_odometry_Human;
  _algorithms[OdometryAlg::NeuralRot] = &_odometry_NeuralRot;
  _algorithms[OdometryAlg::LKFlow] = &_odometry_LKFlow;
  _algorithms[OdometryAlg::Gyro] = nullptr;
  _algorithms[OdometryAlg::ManualOverride] = &_odometry_Override;
  _algorithms[OdometryAlg::OpenCV] = &_odometry_opencv;
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
      _dataOpponent.angle.has_value()) {
    UpdateForceSetAngle(_dataOpponent.angle.value().angle - angleUserAdjust,
                        true);
  } else if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightArrow) &&
             _dataOpponent.angle.has_value()) {
    UpdateForceSetAngle(_dataOpponent.angle.value().angle + angleUserAdjust,
                        true);
  }

  // opponent with up and 1 and 3
  if (InputState::GetInstance().IsKeyDown(ImGuiKey_1)) {
    if (auto a = _dataOpponent.angle) {
      UpdateForceSetAngle(a.value().angle - angleUserAdjust, true);
    }
  } else if (InputState::GetInstance().IsKeyDown(ImGuiKey_3)) {
    if (auto a = _dataOpponent.angle) {
      UpdateForceSetAngle(a.value().angle + angleUserAdjust, true);
    }
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

  // Set manual override source so fusion uses these as priority
  _odometry_Override.SetPosition(
      PositionData(TrackingWidget::robotMouseClickPoint, cv::Point2f(0, 0),
                   currTime),
      false);
  _odometry_Override.SetAngle(
      AngleData(Angle(TrackingWidget::robotMouseClickAngle), 0, currTime),
      false);
  _odometry_Override.SetPosition(
      PositionData(TrackingWidget::opponentMouseClickPoint, cv::Point2f(0, 0),
                   currTime),
      true);
  _odometry_Override.SetAngle(
      AngleData(Angle(TrackingWidget::opponentMouseClickAngle), 0, currTime),
      true);

  // Back-annotate other algorithms so they stay in sync
  PositionData robotPos(TrackingWidget::robotMouseClickPoint, cv::Point2f(0, 0),
                        currTime);
  AngleData robotAngle(Angle(TrackingWidget::robotMouseClickAngle), 0,
                       currTime);
  PositionData opponentPos(TrackingWidget::opponentMouseClickPoint,
                           cv::Point2f(0, 0), currTime);
  AngleData opponentAngle(Angle(TrackingWidget::opponentMouseClickAngle), 0,
                          currTime);

  _odometry_Heuristic.SetAngle(robotAngle, false);
  _odometry_Blob.SetAngle(robotAngle, false);
  _odometry_LKFlow.SetAngle(robotAngle, false);
  _odometry_Heuristic.SetVelocity(robotPos.velocity, false);
  _odometry_Blob.SetPosition(robotPos, false);
  _odometry_LKFlow.SetPosition(robotPos, false);

  _odometry_Heuristic.SetAngle(opponentAngle, true);
  _odometry_Blob.SetAngle(opponentAngle, true);
  _odometry_LKFlow.SetAngle(opponentAngle, true);
  _odometry_Heuristic.ForcePosition(opponentPos, true);
  _odometry_Blob.ForcePosition(opponentPos, true);
  _odometry_LKFlow.ForcePosition(opponentPos, true);

  _odometry_opencv.SetPosition(robotPos, false);
  _odometry_opencv.SetPosition(opponentPos, true);
  _odometry_opencv.SetAngle(opponentAngle, true);
  _odometry_opencv.SetVelocity(opponentPos.velocity, true);

  fusionStateMachine = FUSION_NORMAL;

  if (!partOfAuto) {
    _odometry_Heuristic.MatchStart();
  }
}

// Updates internal Odometry data
void RobotOdometry::Update() {
  RawInputs inputs = _poller.Poll(_prevInputs);

  // No new data and thus nothing to do
  if (!inputs.HasUpdates()) {
    // Store for next iteration
    _prevInputs = inputs;
    return;
  }

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
  }

  // Store for next iteration
  _prevInputs = inputs;
}

FusionOutput RobotOdometry::Fuse(RawInputs inputs, double now,
                                 const OdometryData &prevRobot,
                                 const OdometryData &prevOpponent) {
  FusionOutput output{};
  // ******************************
  // We have the following sources of data:
  // ALGORITH  | US POS |  US VEL | US ROT | US A.VEL | THEM POS | THEM VEL |  THEM ROT | THEM A.VEL
  // -------------------------------------------------------------------------------------------------
  // Heuristic |   X    |    X   |     X   |    X     |    X     |   X      |     X     |     X
  // Blob      |   X    |    X   |     X   |    X     |    X     |   X      |     X     |     X
  // Neural    |   X    |        |         |          |          |          |           |
  // NeuralRot |        |        |     X   |          |          |          |           |
	// LKR       |   X    |    X   |     X   |    X     |    X     |   X      |     X     |     X
  // IMU       |        |        |     X   |    X     |          |          |           |
  // Human     |        |        |     X   |          |          |          |     X     |     
  //


  // Initialize output with previous state (for extrapolation baseline)
  output.robot = prevRobot;
  output.opponent = prevOpponent;

  // GLOBAL PRECHECK:
	//
  // Step 1) Check blob sanity
  //
  // If blob moved much faster than LK flow (same target), disqualify blob.
  MaybeDisqualifyBlobByLKFlow(
      inputs.them_lkflow.pos, _prevInputs.them_lkflow.pos, inputs.them_blob.pos,
      _prevInputs.them_blob.pos, inputs.them_blob.pos, inputs.them_blob.angle);
  MaybeDisqualifyBlobByLKFlow(inputs.us_lkflow.pos, _prevInputs.us_lkflow.pos,
                              inputs.us_blob.pos, _prevInputs.us_blob.pos,
                              inputs.us_blob.pos, inputs.us_blob.angle);

  MaybeDisqualifyBlobByDistance(inputs.us_blob, inputs.them_blob, prevRobot,
                                prevOpponent);


  //      If Neural says we should be swapped, then swap
  //      G1) if Neural = Heuristic.them: SWAP Heuristic
  //
  //      // US POS: If Neural agrees with blob, use Neural
  //      G2) if Neural != Heuristic.us && (Neural=Blob.us || Neural=LKR.us): Heuristic.ForcePosUs(Neural) (and invalidate Heuristic).
  //		    if Neural != LKR.us && (Neural=Blob.us || Neural=Heuristic.us): LKR.ForcePosUs(Neural) (and invalidate LKR)
	//	
  // Neural-based rules and disqualifications
  if (isFresh(inputs.us_neural.pos)) {
    // G1) if Neural US = Heuristic.them: SWAP Heuristic and invalidate them
    if (isFresh(inputs.them_heuristic.pos) &&
        inputs.them_heuristic.IsPointInside(
            inputs.us_neural.pos.value().position)) {
      output.backAnnotate.swapHeuristic = true;
      inputs.them_heuristic.pos.reset();
      inputs.them_heuristic.angle.reset();
    }
    // Same for Blob
    if (isFresh(inputs.them_blob.pos) &&
        inputs.them_blob.IsPointInside(inputs.us_neural.pos.value().position)) {
      output.backAnnotate.swapBlob = true;
      inputs.them_blob.pos.reset();
      inputs.them_blob.angle.reset();
    }

    if (isFresh(inputs.us_blob.pos) &&
        !inputs.us_blob.IsPointInside(inputs.us_neural.pos.value().position)) {
      inputs.us_blob.pos.reset();
      inputs.us_blob.angle.reset();
    }

    if (isFresh(inputs.us_lkflow.pos) &&
        !inputs.us_lkflow.IsPointInside(
            inputs.us_neural.pos.value().position)) {
      inputs.us_lkflow.pos.reset();
      inputs.us_lkflow.angle.reset();
    }

    // Neural agrees with blob, but heuristic doesn't agree
    if (isFresh(inputs.us_blob.pos) &&
        inputs.us_blob.IsPointInside(inputs.us_neural.pos.value().position) &&
        // Heuristic is invalid or doesn't agree with neural
        (!isFresh(inputs.us_heuristic.pos) ||
         !inputs.us_heuristic.IsPointInside(
             inputs.us_neural.pos.value().position))) {
      inputs.us_heuristic.pos.reset();
      inputs.us_heuristic.angle.reset();
      output.robot.pos = inputs.us_neural.pos;
      output.backAnnotate.forceRobotPos_Heuristic = true;
    }
  }

  //  US POS:
	//			 If( Heuristic.Combined || Lower confidence)  1) Neural, 2) Blob*, 3) LKR, 4) Heuristic
	//			 else  1) Heuristic, 2) Neural, 3) Blob, 4) LKR
  // Note: Blob now diqualifies himself when combined so no need to check here.
  // *: Unlikely for blob not to be combined if heuristic is combined
  // Robot position (manual override has highest priority)
  if (isFresh(inputs.us_override.pos)) {
    output.robot.pos = inputs.us_override.pos;
  } else if (isFresh(inputs.us_heuristic.pos)) {   
    // If Heuristic is combined or has non 100% confidence then deprioritize it
    if(inputs.us_heuristic.userDataDouble.find("Combined") != inputs.us_heuristic.userDataDouble.end() ||
       inputs.us_heuristic.userDataDouble.find("Confidence") != inputs.us_heuristic.userDataDouble.end()){

        if (isFresh(inputs.us_blob.pos)) {
          output.robot.pos = inputs.us_blob.pos;
        } else if (isFresh(inputs.us_lkflow.pos)) {
          output.robot.pos = inputs.us_lkflow.pos;
        } else if (isFresh(inputs.us_neural.pos)) {
          output.robot.pos = inputs.us_neural.pos;
        } else {
          output.robot.pos = inputs.us_heuristic.pos;
        } 
    }
    else  {
      output.robot.pos = inputs.us_heuristic.pos;
      // If blob position isn't inside our rectangle then set position
      if (isFresh(inputs.us_blob.pos) &&
          !inputs.us_heuristic.IsPointInside(
              inputs.us_blob.pos.value().position)) {
        inputs.us_blob.pos.reset();
      }
    }
  } else if (isFresh(inputs.us_blob.pos)) {
    output.robot.pos = inputs.us_blob.pos;
  } else if (isFresh(inputs.us_lkflow.pos)) {
    output.robot.pos = inputs.us_lkflow.pos;
  } else if (isFresh(inputs.us_neural.pos)) {
    output.robot.pos = inputs.us_neural.pos;
  }

  // Robot velocity
  //  US VEL:
	//			 If( Heuristic.Combined || Lower confidence) 1) Blob, 2) LKR, 3) Heuristic
	//			 else:  1) Heuristic, 2) Neural, 3) blob, 4) LKR
  // Note: Blob now diqualifies himself when combined so no need to check here
  if (output.robot.pos.has_value()) {
    if (isFresh(inputs.us_override.pos)) {
      output.robot.pos.value().velocity =
          inputs.us_override.pos.value().velocity;
    } else if (isFresh(inputs.us_heuristic.pos)) {
      // If Heuristic is combined or has non 100% confidence then deprioritize it
      if(inputs.us_heuristic.userDataDouble.find("Combined") != inputs.us_heuristic.userDataDouble.end() ||
      inputs.us_heuristic.userDataDouble.find("Confidence") != inputs.us_heuristic.userDataDouble.end()){

        if (isFresh(inputs.us_blob.pos)) {
          output.robot.pos.value().velocity = inputs.us_blob.pos.value().velocity;
        } else if (isFresh(inputs.us_lkflow.pos)) {
          output.robot.pos.value().velocity = inputs.us_lkflow.pos.value().velocity;
        } else {
          output.robot.pos.value().velocity = inputs.us_heuristic.pos.value().velocity;
        } 
      }
      else {
        output.robot.pos.value().velocity = inputs.us_heuristic.pos.value().velocity;
      }
    } else if (isFresh(inputs.us_blob.pos)) {
      output.robot.pos.value().velocity = inputs.us_blob.pos.value().velocity;
    } else if (isFresh(inputs.us_lkflow.pos)) {
      output.robot.pos.value().velocity = inputs.us_lkflow.pos.value().velocity;
    }
  }

  // Robot angle (manual override has highest priority)
  // No Heuristic backfall???
  if (isFresh(inputs.us_override.angle)) {
    output.robot.angle = inputs.us_override.angle;
  } else if (isFresh(inputs.us_imu.angle)) {
    output.robot.angle = inputs.us_imu.angle;
  } else if (isFresh(inputs.us_lkflow.angle)) {
    output.robot.angle = inputs.us_lkflow.angle;
  } else if (isFresh(inputs.us_neuralrot.angle)) {
    output.robot.angle = inputs.us_neuralrot.angle;
    output.robot.angle.value().velocity = 0;
  }

   //  THEM POS:
	//			 If( Heuristic.Combined || Lower confidence)  1) Blob, 2) LKR, 3) Heuristic
	//			 else  1) Heuristic, 3) Blob, 4) LKR
  // Note: Blob now diqualifies himself when combined so no need to check here
  // Opponent position (manual override has highest priority)
  if (isFresh(inputs.them_override.pos)) {
    output.opponent.pos = inputs.them_override.pos;
  } else if (isFresh(inputs.them_heuristic.pos)) {
      // If Heuristic is combined or has non 100% confidence then deprioritize it
      if(inputs.them_heuristic.userDataDouble.find("Combined") != inputs.them_heuristic.userDataDouble.end() ||
         inputs.them_heuristic.userDataDouble.find("Confidence") != inputs.them_heuristic.userDataDouble.end()){
       if (isFresh(inputs.them_blob.pos)) {
          output.opponent.pos.value().velocity = inputs.them_blob.pos.value().velocity;
        } else if (isFresh(inputs.them_lkflow.pos)) {
          output.opponent.pos.value().velocity = inputs.them_lkflow.pos.value().velocity;
        } else {
          output.opponent.pos.value().velocity = inputs.them_heuristic.pos.value().velocity;
        } 
      } else {
        output.opponent.pos = inputs.them_heuristic.pos;
      }
  } else if (isFresh(inputs.them_blob.pos)) {
    output.opponent.pos = inputs.them_blob.pos;
  } else if (isFresh(inputs.them_lkflow.pos)) {
    output.opponent.pos = inputs.them_lkflow.pos;
  }

  
  //			 If( Heuristic.Combined || Lower confidence)  1) Blob, 2) LKR, 3) Heuristic
	//			 else  1) Heuristic, 3) Blob, 4) LKR
  // Opponent velocity
  
  if (output.opponent.pos.has_value()) {
    if (isFresh(inputs.them_override.pos)) {
      output.opponent.pos.value().velocity =
          inputs.them_override.pos.value().velocity;
    } else if (isFresh(inputs.them_heuristic.pos)) {
            // If Heuristic is combined or has non 100% confidence then deprioritize it
      if(inputs.them_heuristic.userDataDouble.find("Combined") != inputs.them_heuristic.userDataDouble.end() ||
         inputs.them_heuristic.userDataDouble.find("Confidence") != inputs.them_heuristic.userDataDouble.end()){

        if (isFresh(inputs.them_blob.pos)) {
          output.opponent.pos.value().velocity = inputs.them_blob.pos.value().velocity;
        } else if (isFresh(inputs.them_lkflow.pos)) {
          output.opponent.pos.value().velocity = inputs.them_lkflow.pos.value().velocity;
        } else {
          output.opponent.pos.value().velocity = inputs.them_heuristic.pos.value().velocity;
        } 
      }
      else {
        output.opponent.pos.value().velocity = inputs.them_heuristic.pos.value().velocity;
      }
      
    } else if (isFresh(inputs.them_blob.pos)) {
      output.opponent.pos.value().velocity = inputs.them_blob.pos.value().velocity;
    } else if (isFresh(inputs.them_lkflow.pos)) {
      output.opponent.pos.value().velocity = inputs.them_lkflow.pos.value().velocity;
    }
  }

  // Opponent rotation (manual override has highest priority)
  if (isFresh(inputs.them_override.angle)) {
    output.opponent.angle = inputs.them_override.angle;
  } else if (isFresh(inputs.us_human.angle)) {
    output.opponent.angle = inputs.us_human.angle;
  } else if (isFresh(inputs.them_lkflow.angle)) {
    output.opponent.angle = inputs.them_lkflow.angle;
  } else if (isFresh(inputs.them_heuristic.angle)) {
    output.opponent.angle = inputs.them_heuristic.angle;
  }

  // fuse towards the blob angle if it is fresh
  if (isFresh(inputs.them_blob.angle)) {
    AngleData blobAngle = inputs.them_blob.angle.value();
    double deltaTime = now - blobAngle.time;
    double interpolateAmount = (std::min)(1.0, deltaTime * 1.0);
    output.opponent.angle.value().angle =
        InterpolateAngles(output.opponent.angle.value().angle, blobAngle.angle,
                          interpolateAmount);
    std::cout << "fused towards blob angle: "
              << output.opponent.angle.value().angle.degrees() << std::endl;
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

  // Extrapolate output to current time
  output.robot = output.robot.ExtrapolateBoundedTo(now);
  output.opponent = output.opponent.ExtrapolateBoundedTo(now);

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

  // Robot Position
  if (isFresh(robot.pos)) {
    if (backAnnotate.forceRobotPos_Heuristic) {
      _odometry_Heuristic.ForcePosition(robot.pos.value(), false);
    } else {
      _odometry_Heuristic.SetPosition(robot.pos.value(), false);
    }
    _odometry_Heuristic.SetVelocity(robot.pos.value().velocity, false);
    _odometry_Blob.SetPosition(robot.pos.value(), false);
    _odometry_Blob.SetVelocity(robot.pos.value().velocity, false);
    _odometry_LKFlow.SetPosition(robot.pos.value(), false);
    _odometry_LKFlow.SetVelocity(robot.pos.value().velocity, false);
    _odometry_opencv.SetPosition(robot.pos.value(), false);
  }

  // Robot Angle
  if (isFresh(robot.angle)) {
    _odometry_Heuristic.SetAngle(robot.angle.value(), false);
    _odometry_Blob.SetAngle(robot.angle.value(), false);
    _odometry_LKFlow.SetAngle(robot.angle.value(), false);
    _odometry_opencv.SetAngle(robot.angle.value(), false);
    _odometry_IMU.SetAngle(robot.angle.value(), false);
  }

  // Opponent Position
  if (isFresh(opponent.pos)) {
    _odometry_Heuristic.SetPosition(opponent.pos.value(), true);
    _odometry_Heuristic.SetVelocity(opponent.pos.value().velocity, true);
    _odometry_Blob.SetPosition(opponent.pos.value(), true);
    _odometry_Blob.SetVelocity(opponent.pos.value().velocity, true);
    _odometry_LKFlow.SetPosition(opponent.pos.value(), true);
    _odometry_LKFlow.SetVelocity(opponent.pos.value().velocity, true);
    _odometry_opencv.SetPosition(opponent.pos.value(), true);
  }

  // Opponent Angle
  if (isFresh(opponent.angle)) {
    _odometry_Heuristic.SetAngle(opponent.angle.value(), true);
    _odometry_Blob.SetAngle(opponent.angle.value(), true);
    _odometry_LKFlow.SetAngle(opponent.angle.value(), true);
    _odometry_opencv.SetAngle(opponent.angle.value(), true);
  }
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

OdometryData RobotOdometry::Opponent(double currTime) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData currData = _dataOpponent;
  locker.unlock();

  currData.ExtrapolateBoundedTo(currTime);

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
  double currTime = Clock::programClock.getElapsedTime();
  AngleData angleData(Angle(newAngle), 0, currTime);
  _odometry_Override.SetAngle(angleData, opponentRobot);
  _odometry_Blob.SetAngle(angleData, opponentRobot);
  _odometry_Heuristic.SetAngle(angleData, opponentRobot);
  _odometry_IMU.SetAngle(angleData, opponentRobot);
  _odometry_LKFlow.SetAngle(angleData, opponentRobot);
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
  PositionData posData(newPos, newVel, Clock::programClock.getElapsedTime());
  _odometry_Blob.SetPosition(posData, opponentRobot);
  _odometry_Blob.SetVelocity(newVel, opponentRobot);
  _odometry_Heuristic.SetPosition(posData, opponentRobot);
  _odometry_Heuristic.SetVelocity(newVel, opponentRobot);
  _odometry_LKFlow.SetPosition(posData, opponentRobot);
  _odometry_LKFlow.SetVelocity(newVel, opponentRobot);
  _odometry_opencv.SetPosition(posData, opponentRobot);
}

bool RobotOdometry::Run(OdometryAlg algorithm) {
  if (algorithm == OdometryAlg::Human) {
    return _odometry_Human.Run() && _odometry_Human_Heuristic.Run();
  }
  std::cout << "Running algorithm: " << OdometryAlgToString(algorithm)
            << std::endl;
  OdometryBase *p = _algorithms.at(algorithm);
  return p != nullptr && p->Run();
}

bool RobotOdometry::Stop(OdometryAlg algorithm) {
  if (algorithm == OdometryAlg::Human) {
    return _odometry_Human.Stop() && _odometry_Human_Heuristic.Stop();
  }
  OdometryBase *p = _algorithms.at(algorithm);
  return p != nullptr && p->Stop();
}

bool RobotOdometry::IsRunning(OdometryAlg algorithm) {
  if (algorithm == OdometryAlg::Human) {
    return _odometry_Human.IsRunning() && _odometry_Human_Heuristic.IsRunning();
  }
  OdometryBase *p = _algorithms.at(algorithm);
  return p != nullptr && p->IsRunning();
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

ManualOverrideOdometry &RobotOdometry::GetManualOverrideOdometry() {
  return _odometry_Override;
}

OpenCVTracker &RobotOdometry::GetOpenCVOdometry() { return _odometry_opencv; }

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

  debugImage = cv::Mat::zeros(debugImage.size(), debugImage.type());

  const int leftX = 10 + offset.x;

  int yLeft = 20 + offset.y;
  printText("Final Robot Data:", debugImage, yLeft, leftX);
  _dataRobot.GetDebugImage(debugImage, cv::Point(leftX + 10, yLeft + 14));

  yLeft = 20 + offset.y;

  printText("Final Opp Data:", debugImage, yLeft, leftX + 230);
  _dataOpponent.GetDebugImage(debugImage,
                              cv::Point(leftX + 10 + 230, yLeft + 14));

  yLeft += 140;

  std::unique_lock<std::mutex> locker(_mutexDebugImage);

  printText(_debugString, debugImage, yLeft, offset.x);

  locker.unlock();
}
