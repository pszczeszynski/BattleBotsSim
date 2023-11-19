#pragma once

// This class controls the path planning and strategy of the robot

#include <opencv2/opencv.hpp>

class MovementStrategy
{
public:
    MovementStrategy();

    cv::Point2f AvoidStrategy();
};