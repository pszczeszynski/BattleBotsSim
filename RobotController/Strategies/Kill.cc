#include "Kill.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotConfig.h"

Kill::Kill()
{
}

#define NUM_PREDICTION_ITERS 25

DriveCommand Kill::Execute(Gamepad &gamepad)
{
    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    RobotSimState currentState;
    currentState.position = RobotOdometry::Robot().GetPosition();
    currentState.angle = RobotOdometry::Robot().GetAngle();
    currentState.velocity = RobotOdometry::Robot().GetVelocity();
    double angleExtrapolate = GTP_ANGLE_EXTRAPOLATE_MS;
    currentState.angularVelocity = RobotOdometry::Robot().GetAngleVelocity() * angleExtrapolate / POSITION_EXTRAPOLATE_MS;

    // predict where the robot will be in a couple milliseconds
    RobotSimState exState = robotSimulator.Simulate(currentState, POSITION_EXTRAPOLATE_MS / 1000.0, NUM_PREDICTION_ITERS);

    RobotMovement::DriveDirection direction = RobotMovement::DriveDirection::Auto;

    DriveCommand ret;
    // drive directly to the opponent
    DriveCommand responseGoToPoint = RobotMovement::DriveToPosition(exState, RobotOdometry::Opponent().GetPosition(), direction);
    ret.turn = responseGoToPoint.turn;
    ret.movement = gamepad.GetRightStickY() * abs(responseGoToPoint.movement);
    return ret;
}