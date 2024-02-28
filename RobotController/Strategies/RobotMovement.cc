#include "RobotMovement.h"
#include "../Clock.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"

/**
 * Drive the robot to a specified position
 * @param targetPos The target position
 * @param direction The direction to drive in (forward, backward, or auto)
 */
DriveCommand RobotMovement::DriveToPosition(RobotSimState exState,
                                            const cv::Point2f &targetPos,
                                            DriveDirection direction)
{
    static Clock c;

    OdometryData odoData =  RobotController::GetInstance().odometry.Robot();

    cv::Point2f currPos = odoData.robotPosition;
    double currAngle = odoData.robotAngle;

    double deltaTime = c.getElapsedTime();
    c.markStart();

    double currAngleEx = exState.angle;
    cv::Point2f currPosEx = exState.position;

    double angleToTarget1 = atan2(targetPos.y - currPosEx.y, targetPos.x - currPosEx.x);
    double angleToTarget2 = angle_wrap(angleToTarget1 + M_PI);
    double deltaAngleRad1 = angle_wrap(angleToTarget1 - currAngleEx);
    double deltaAngleRad2 = angle_wrap(angleToTarget2 - currAngleEx);
    double deltaAngleRad1_noex = angle_wrap(angleToTarget1 - currAngle);
    double deltaAngleRad2_noex = angle_wrap(angleToTarget2 - currAngle);

    bool curr_direction = direction == DriveDirection::Forward ? true : false;
    // for auto, choose the direction that is closer to the target angle
    if (direction == DriveDirection::Auto)
    {
        curr_direction = abs(deltaAngleRad1_noex) < abs(deltaAngleRad2_noex);
        // put text on drawing image of curr_direction
        cv::putText(RobotController::GetInstance().GetDrawingImage(), curr_direction ? "1" : "2", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    double deltaAngleRad = curr_direction ? deltaAngleRad1 : deltaAngleRad2;

    DriveCommand response{0, 0};
    response.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                         TURN_THRESH_2_DEG * TO_RAD,
                                         MIN_TURN_POWER_PERCENT / 100.0,
                                         MAX_TURN_POWER_PERCENT / 100.0);

    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;
    // Slow down when far away from the target angle
    double drive_scale = max(0.0, 1.0 - abs(response.turn / (MAX_TURN_POWER_PERCENT / 100.0)) * scaleDownMovement) * 1.0;

    response.movement = curr_direction ? drive_scale : -drive_scale;
    response.turn *= -1;

    return response;
}
