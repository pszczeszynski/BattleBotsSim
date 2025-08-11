#include "Kill.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"

Kill::Kill()
{
}

DriverStationMessage Kill::Execute(Gamepad &gamepad)
{
    // RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    // extrapolate our position into the future
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();
    double extrapolationTimeSeconds = POSITION_EXTRAPOLATE_MS / 1000.0;
    ourData = ourData.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime() + extrapolationTimeSeconds, extrapolationTimeSeconds);

    
    // extrapolate the opponent's position into the future
    OdometryData opponentData = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS_KILL / 1000.0, MAX_OPP_EXTRAP_MS_KILL);

    // draw a crosshair on the opponent
    safe_circle(RobotController::GetInstance().GetDrawingImage(), opponentData.robotPosition, 10, cv::Scalar(0, 0, 255), 2);

    DriverStationMessage ret;
    // // hold angle to the opponent
    // DriverStationMessage ret = RobotMovement::HoldAngle(ourData.robotPosition,
    //                                                     opponentData.robotPosition,
    //                                                     KILL_KD_PERCENT,
    //                                                     TURN_THRESH_1_DEG_KILL,
    //                                                     TURN_THRESH_2_DEG_KILL,
    //                                                     MAX_TURN_POWER_PERCENT_KILL,
    //                                                     MIN_TURN_POWER_PERCENT_KILL,
    //                                                     SCALE_DOWN_MOVEMENT_PERCENT_KILL,
    //                                                     direction);

    return ret;
}