#include <opencv2/opencv.hpp>
#include "../../Common/Communication.h"
#include "../Extrapolator.h"

namespace RobotMovement
{
    enum class DriveDirection
    {
        Forward,  // Drive forward
        Backward, // Drive backwards
        Auto      // Automatically choose the direction based on the shortest angle
    };

    DriverStationMessage HoldAngle(cv::Point2f currPos,
                                   cv::Point2f targetPos,
                                   int ANGLE_EXTRAPOLATE_MS,
                                   int TURN_THRESH_1_DEG,
                                   int TURN_THRESH_2_DEG,
                                   int MAX_TURN_POWER_PERCENT,
                                   int MIN_TURN_POWER_PERCENT,
                                   int SCALE_DOWN_MOVEMENT_PERCENT,
                                   DriveDirection direction);
};
