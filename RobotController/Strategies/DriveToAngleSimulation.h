#pragma once
#include "../../Common/Communication.h"

DriveCommand DriveToAngle(double currAngle,
                          double angularVelocity,
                          float targetAngle,
                          float targetAngleVelocity,
                          unsigned short KD_PERCENT,
                          short TURN_THRESH_1_DEG,
                          short TURN_THRESH_2_DEG,
                          short MAX_TURN_POWER_PERCENT,
                          short MIN_TURN_POWER_PERCENT,
                          short SCALE_DOWN_MOVEMENT_PERCENT);
