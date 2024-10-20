#include "DriveToAngleSimulation.h"
#include <cmath>
// for max
#include <algorithm>
#include "../MathUtils.h"

#include "../UIWidgets/GraphWidget.h"

#define M_PI 3.14159265358979323846
#define TO_RAD (M_PI / 180.0)


/**
 * Drives the robot to a target angle given
 * 1. The current angle
 * 2. Our angular velocity (from the gyro)
 * 3. The target angle
 * 4. The target angular velocity, used to compute the D term
 * 5. The KD term as a percentage (may be over 100)
 * 6. The first threshold for turning in degrees
 * 7. The second threshold for turning in degrees
 * 8. The max power to turn at (100 is full power)
 * 9. The min power to turn at (100 is full power)
 * 10. The scale down movement percent (100 is full power)
*/
DriveCommand DriveToAngle(double currAngle,
                          double angularVelocity,
                          float targetAngle,
                          float targetAngleVelocity,
                          unsigned short KD_PERCENT,
                          short TURN_THRESH_1_DEG,
                          short TURN_THRESH_2_DEG,
                          short MAX_TURN_POWER_PERCENT,
                          short MIN_TURN_POWER_PERCENT,
                          short SCALE_DOWN_MOVEMENT_PERCENT)
{
    static GraphWidget derivativeWidget("Derivative of Angle Error", -10, 10, "rad/s");
    static GraphWidget targetAngleWidget("Target Angle Vel", -10, 10, "rad/s");
    static GraphWidget robotAngleWidget("Angle Vel", -10, 10, "rad/s");
    // Calculate the angle to the target
    double deltaAngleRad = angle_wrap(targetAngle - currAngle);

    // Calculate the power to turn at
    DriveCommand ret{0, 0, 0, 0, 0};
    ret.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                    TURN_THRESH_2_DEG * TO_RAD,
                                    MIN_TURN_POWER_PERCENT / 100.0,
                                    MAX_TURN_POWER_PERCENT / 100.0);

    // deriviative of error
    double deltaAngleVel = targetAngleVelocity - angularVelocity;
    derivativeWidget.AddData(deltaAngleVel);
    targetAngleWidget.AddData(targetAngleVelocity);
    robotAngleWidget.AddData(angularVelocity);
    ret.turn += deltaAngleVel * (KD_PERCENT / 100.0); // add the D term
    // clip turn
    ret.turn = std::max(-1.0f, std::min(1.0f, ret.turn));

    // Slow down when far away from the target angle
    double drive_scale = std::max(0.0, 1.0 - abs(ret.turn / (MAX_TURN_POWER_PERCENT / 100.0)) * (SCALE_DOWN_MOVEMENT_PERCENT / 100.0));
    ret.turn *= -1;

    return ret;
}
