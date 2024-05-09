#include "Communication.h"
#include <Arduino.h>

double DoubleThreshToTarget(double error,
                            double threshold1, double threshold2,
                            double minPower, double maxPower);


DriveCommand DriveToAngle(double currAngle,
                          double angularVelocity,
                          float targetAngle,
                          short ANGLE_EXTRAPOLATE_MS,
                          short TURN_THRESH_1_DEG,
                          short TURN_THRESH_2_DEG,
                          short MAX_TURN_POWER_PERCENT,
                          short MIN_TURN_POWER_PERCENT,
                          short SCALE_DOWN_MOVEMENT_PERCENT);
