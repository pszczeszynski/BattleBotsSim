#pragma once

#include "Clock.h"
#include "../Common/Communication.h"
#include <opencv2/core.hpp>

class SelfRighter
{
public:
    SelfRighter();
    void Move(float inputPower, DriveCommand& command, cv::Mat& drawingImage);
};