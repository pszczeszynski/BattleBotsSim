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
    cv::Point2f leftStart = cv::Point2f(HEU_LEFTSTART_X, HEU_LEFTSTART_Y);
    cv::Point2f rightStart = cv::Point2f(HEU_RIGHTSTART_X, HEU_RIGHTSTART_Y);
};