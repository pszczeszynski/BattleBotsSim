#include <opencv2/opencv.hpp>
#include "../../Communication/Communication.h"
#include "../Extrapolator.h"

namespace RobotMovement
{
    enum class DriveDirection
    {
        Forward,  // Drive forward
        Backward, // Drive backwards
        Auto      // Automatically choose the direction based on the shortest angle
    };
    DriveCommand DriveToPosition(RobotSimState exState,
                                 const cv::Point2f &targetPos,
                                 int TURN_THRESH_1_DEG,
                                 int TURN_THRESH_2_DEG,
                                 int MAX_TURN_POWER_PERCENT,
                                 int MIN_TURN_POWER_PERCENT,
                                 DriveDirection direction = DriveDirection::Auto);
};
