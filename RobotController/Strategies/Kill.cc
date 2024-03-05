#include "Kill.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotConfig.h"
#include "../RobotController.h"

Kill::Kill()
{
}

#define NUM_PREDICTION_ITERS 25
#define MAX_PREDICTION_TIME 1.0
DriverStationMessage Kill::Execute(Gamepad &gamepad)
{
    RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    // extrapolate our position into the future
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();
    ourData.Extrapolate(Clock::programClock.getElapsedTime() + (POSITION_EXTRAPOLATE_MS / 1000.0));

    // extrapolate the opponent's position into the future
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();

    double distanceToOpponent = cv::norm(ourData.robotPosition - opponentData.robotPosition);

    double extrapolationTime = (distanceToOpponent / ORBIT_RADIUS) * (OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0);
    extrapolationTime += Clock::programClock.getElapsedTime();
    extrapolationTime = std::min(opponentData.time + MAX_PREDICTION_TIME, extrapolationTime);

    
    opponentData.Extrapolate(extrapolationTime);

    // draw a crosshair on the opponent
    cv::circle(RobotController::GetInstance().GetDrawingImage(), opponentData.robotPosition, 10, cv::Scalar(0, 0, 255), 2);

    // hold angle to the opponent
    DriverStationMessage ret = RobotMovement::HoldAngle(ourData.robotPosition,
                                                        opponentData.robotPosition,
                                                        KILL_ANGLE_EXTRAPOLATE_MS,
                                                        TURN_THRESH_1_DEG_KILL,
                                                        TURN_THRESH_2_DEG_KILL,
                                                        MAX_TURN_POWER_PERCENT_KILL,
                                                        MIN_TURN_POWER_PERCENT_KILL,
                                                        SCALE_DOWN_MOVEMENT_PERCENT_KILL,
                                                        direction);

    return ret;
}