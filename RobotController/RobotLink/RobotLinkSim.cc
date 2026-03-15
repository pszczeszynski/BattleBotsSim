#include "RobotLinkSim.h"

#include "../../Common/Communication.h"
#include "../Clock.h"
#include "../Globals.h"
#include "../MathUtils.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "../RobotStateParser.h"
#include "../SimulationState.h"

RobotLinkSim::RobotLinkSim() : serverSocket("11115") {}

void RobotLinkSim::Drive(DriverStationMessage &msg) {
  DriveCommand command;

  if (msg.type == DRIVE_COMMAND) {
    command = msg.driveCommand;
  }

  double opponentMoveAmount = 0;
  double opponentTurnAmount = 0;
  if (CONTROL_OPPONENT_ENABLED) {
    Gamepad &gamepad1 = RobotController::GetInstance().GetGamepad();
    opponentMoveAmount = gamepad1.GetRightStickY();
    opponentTurnAmount = -gamepad1.GetLeftStickX();
  }

  UnityDriveCommand message = {command.movement,
                               command.turn,
                               (double)command.frontWeaponPower,
                               (double)command.backWeaponPower,
                               false,
                               opponentMoveAmount,
                               opponentTurnAmount};

  serverSocket.reply_to_last_sender(RobotStateParser::serialize(message));
}

#define ACCELEROMETER_TO_PX_SCALER 1

std::vector<RobotMessage> RobotLinkSim::_ReceiveImpl() {
  static Clock lastCanDataClock;
  static Clock lastReceiveClock;
  static cv::Point2f lastRobotPosSim;
  static cv::Point2f lastOpponentPosSim;

  RobotMessage ret{RobotMessageType::INVALID};
  memset(&ret, 0, sizeof(ret));

  std::string received = "";
  while (received == "") {
    received = serverSocket.receive();
  }
  UnityRobotState message = RobotStateParser::parse(received);
  
  cv::Point2f robotPos = cv::Point2f((float)message.robot_position.x,
                                      (float)message.robot_position.z);
  cv::Point2f opponentPos = cv::Point2f{(float)message.opponent_position.x,
                                        (float)message.opponent_position.z};

  cv::Point2f mins = {-10, -10};
  cv::Point2f maxs = {10, 10};

  robotPos -= mins;
  robotPos.x /= maxs.x - mins.x;
  robotPos.y /= maxs.y - mins.y;
  robotPos.x = 1.0 - robotPos.x;
  robotPos *= WIDTH;

  opponentPos -= mins;
  opponentPos.x /= maxs.x - mins.x;
  opponentPos.y /= maxs.y - mins.y;
  opponentPos.x = 1.0 - opponentPos.x;
  opponentPos *= WIDTH;

  double deltaTime = lastReceiveClock.getElapsedTime();
  cv::Point2f robotVel = (robotPos - lastRobotPosSim) / deltaTime;
  cv::Point2f opponentVel = (opponentPos - lastOpponentPosSim) / deltaTime;
  
  lastRobotPosSim = robotPos;
  lastOpponentPosSim = opponentPos;
  lastReceiveClock.markStart();
  
  double opponentRotation = angle_wrap(message.opponent_rotation * TO_RAD + M_PI);
  double opponentRotationVel = message.opponent_rotation_velocity;
  double currentTime = Clock::programClock.getElapsedTime();

  SimulationState::GetInstance().SetRobotData(robotPos, robotVel, currentTime);
  SimulationState::GetInstance().SetOpponentData(opponentPos, opponentVel,
                                                   opponentRotation, opponentRotationVel,
                                                   currentTime);

  if (lastCanDataClock.getElapsedTime() < 0.5) {
    ret.type = RobotMessageType::IMU_DATA;
    ret.imuData.rotation = Angle(message.robot_rotation + M_PI);
    ret.imuData.rotationVelocity = message.robot_rotation_velocity;
  } else {
    ret.type = RobotMessageType::CAN_DATA;
    ret.canData.motorERPM[2] =
        (int)abs(message.spinner_1_RPM * RPM_TO_ERPM / ERPM_FIELD_SCALAR) * 9;
    ret.canData.motorERPM[3] =
        (int)abs(message.spinner_2_RPM * RPM_TO_ERPM / ERPM_FIELD_SCALAR) * 9;
    lastCanDataClock.markStart();
  }

  double receiveTime = Clock::programClock.getElapsedTime();
  _lastMessageReceiveTimes = {receiveTime};

  return {ret};
}

void RobotLinkSim::ResetSimulation() {
  UnityDriveCommand message = {0, 0, 0, 0, true};
  serverSocket.reply_to_last_sender(RobotStateParser::serialize(message));
}
