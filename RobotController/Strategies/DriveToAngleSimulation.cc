#include "DriveToAngleSimulation.h"
#include <cmath>
// for max
#include <algorithm>
#include "../MathUtils.h"

#define M_PI 3.14159265358979323846
#define TO_RAD (M_PI / 180.0)

/**
 * Drives the robot to a target angle given
 * 1. The current angle
 * 2. The angular velocity
 * 3. The target angle
 * 4. The time to extrapolate the angle in ms
 * 5. The first threshold for turning in degrees
 * 6. The second threshold for turning in degrees
 * 7. The max power to turn at (100 is full power)
 * 8. The min power to turn at (100 is full power)
 * 9. The scale down movement percent (100 is full power)
*/
DriveCommand DriveToAngle(double currAngle,
                          double angularVelocity,
                          float targetAngle,
                          short ANGLE_EXTRAPOLATE_MS,
                          short TURN_THRESH_1_DEG,
                          short TURN_THRESH_2_DEG,
                          short MAX_TURN_POWER_PERCENT,
                          short MIN_TURN_POWER_PERCENT,
                          short SCALE_DOWN_MOVEMENT_PERCENT)
{
    // extrapolate the current angle using the angular velocity
    double currAngleEx = angle_wrap(currAngle + angularVelocity * (ANGLE_EXTRAPOLATE_MS / 1000.0));

    // Calculate the angle to the target
    double deltaAngleRad = angle_wrap(targetAngle - currAngleEx);

    // Calculate the power to turn at
    DriveCommand ret{0, 0, 0, 0, 0};
    ret.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                    TURN_THRESH_2_DEG * TO_RAD,
                                    MIN_TURN_POWER_PERCENT / 100.0,
                                    MAX_TURN_POWER_PERCENT / 100.0);

    // Slow down when far away from the target angle
    double drive_scale = std::max(0.0, 1.0 - abs(ret.turn / (MAX_TURN_POWER_PERCENT / 100.0)) * (SCALE_DOWN_MOVEMENT_PERCENT / 100.0));
    ret.turn *= -1;

    return ret;
}
