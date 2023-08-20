#pragma once

#include "Clock.h"
#include "../Communication/Communication.h"
// cv
#include <opencv2/core.hpp>

#define RANGE_SECONDS_FULL_POWER 3.0

class SelfRighter
{
public:
    SelfRighter();
    void Move(float inputPower, DriveCommand& command, cv::Mat& drawingImage);
private:
    Clock _updateClock;
    double _positionGuessPercent;
};