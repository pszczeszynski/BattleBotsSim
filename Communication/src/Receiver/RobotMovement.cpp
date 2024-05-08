#include "Receiver/RobotMovement.h"
#include "Communication.h"
#include <math.h>

#define M_PI 3.14159265358979323846
#define TO_RAD (M_PI / 180.0)

/**
 * Uses the error between a currentPos and targetPos
 * and returns a power to drive towards the target at (with 2 thresholds)
 * 
 * SHOULD BE THE SAME METHOD AS IN THE DRIVER STATION
 *
 * @param error The target position
 * @param threshold1 The first threshold (full power)
 * @param threshold2 The second threshold (min power)
 * @param minPower The minimum power to drive at
 */
double DoubleThreshToTarget(double error,
                            double threshold1, double threshold2,
                            double minPower, double maxPower)

{
    double distance = std::abs(error);
    double ret = 0;

    if (distance >= threshold1)
    {
        // Move towards the target with full power
        ret = maxPower;
    }
    else if (distance < threshold1 && distance > threshold2)
    {
        // Scale linearly from maxPower to minPower
        ret = ((distance - threshold2) / (threshold1 - threshold2)) * (maxPower - minPower) + minPower;
    }
    else
    {
        // Scale linearly from minPower to 0
        ret = (distance / threshold2) * minPower;
    }

    // invert if we need to
    if (error < 0)
    {
        ret *= -1;
    }

    return ret;
}

double angle_wrap(double angle_rad)
{
    angle_rad = fmod(angle_rad, 2 * M_PI);
    if (angle_rad <= -M_PI)
    {
        angle_rad += 2 * M_PI;
    }
    else if (angle_rad > M_PI)
    {
        angle_rad -= 2 * M_PI;
    }
    return angle_rad;
}

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
    static double lastTime = millis();
    double currTime = millis();
    double deltaTimeSeconds = (currTime - lastTime) / 1000.0;
    lastTime = currTime;

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
