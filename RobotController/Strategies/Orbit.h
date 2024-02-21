#pragma once
#include "Strategy.h"
#include "../Input/Gamepad.h"
#include <opencv2/opencv.hpp>
#include "../Extrapolator.h"

enum OrbitState
{
    LARGE_CIRCLE,
    GO_AROUND
};


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

    RobotSimState _ExtrapolateOurPos(double seconds_position, double seconds_angle);

    bool _IsPointOutOfBounds(cv::Point2f point);

    OrbitState orbitState = LARGE_CIRCLE;

};