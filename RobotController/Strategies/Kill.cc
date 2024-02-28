#include "Kill.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotConfig.h"
#include "../RobotController.h"

Kill::Kill()
{
}

#define NUM_PREDICTION_ITERS 25

DriveCommand Kill::Execute(Gamepad &gamepad)
{
    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    OdometryData odoData =  RobotController::GetInstance().odometry.Robot();

    RobotSimState currentState;
    currentState.position = odoData.robotPosition;
    currentState.angle =  odoData.robotAngle;
    currentState.velocity = odoData.robotVelocity;
    double angleExtrapolate = KILL_ANGLE_EXTRAPOLATE_MS;
    currentState.angularVelocity = odoData.robotAngleVelocity * angleExtrapolate / POSITION_EXTRAPOLATE_MS;

    // predict where the robot will be in a couple milliseconds
    RobotSimState exState = robotSimulator.Simulate(currentState, POSITION_EXTRAPOLATE_MS / 1000.0, NUM_PREDICTION_ITERS);

    RobotMovement::DriveDirection direction = RobotMovement::DriveDirection::Auto;

    OdometryData opponentData =  RobotController::GetInstance().odometry.Opponent();

    DriveCommand ret;
    // drive directly to the opponent
    DriveCommand responseGoToPoint = RobotMovement::DriveToPosition(exState, opponentData.robotPosition, direction);
    ret.turn = responseGoToPoint.turn;
    ret.movement = gamepad.GetRightStickY() * abs(responseGoToPoint.movement);
    return ret;
}