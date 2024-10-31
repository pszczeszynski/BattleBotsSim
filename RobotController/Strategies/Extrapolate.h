#pragma once
#include "../RobotController.h"
#include "../Odometry/OdometryBase.h"
#include "../RobotConfig.h"



#define MAX_PREDICTION_TIME 0.8

OdometryData ExtrapolateOpponentPos(double seconds, double max_prediction_time = MAX_PREDICTION_TIME);