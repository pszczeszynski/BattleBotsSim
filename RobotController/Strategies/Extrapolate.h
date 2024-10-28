#pragma once
#include "../RobotController.h"
#include "../Odometry/OdometryBase.h"
#include "../RobotConfig.h"



#define MAX_PREDICTION_TIME 0.8

OdometryData ExtrapolateOpponentPos();