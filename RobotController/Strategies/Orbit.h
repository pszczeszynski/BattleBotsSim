#pragma once
#include "Strategy.h"
#include "../Input/Gamepad.h"
#include <opencv2/opencv.hpp>

class Orbit : public Strategy
{
public:
    Orbit();

    virtual DriveCommand Execute(Gamepad &gamepad) override;

private:
    double _CalculateOrbitRadius(cv::Point2f opponentPosEx,
                                 Gamepad &gamepad);
    cv::Point2f _NoMoreAggressiveThanTangent(Gamepad &gamepad,
                                             cv::Point2f ourPosition,
                                             cv::Point2f opponentPosEx,
                                             double orbitRadius,
                                             cv::Point2f currentTargetPoint,
                                             bool circleDirection);
};