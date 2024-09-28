#pragma once
#include "imgui.h"
#include "imgui_internal.h"
#include <opencv2/opencv.hpp>
#include "../RobotConfig.h"

class ConfigWidget
{
public:
    ConfigWidget();
    void Draw();
    static cv::Point2f leftStart;
    static cv::Point2f rightStart;
};