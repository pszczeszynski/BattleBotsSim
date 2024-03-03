#pragma once
#include "Strategy.h"
#include "../Input/Gamepad.h"
#include <opencv2/opencv.hpp>
#include "../Extrapolator.h"



class Orbit : public Strategy
{
public:
    Orbit();

    virtual DriverStationMessage Execute(Gamepad &gamepad) override;
    void StartOrbit();
    void StopOrbit();


private:
    double _CalculateOrbitRadius(cv::Point2f opponentPosEx,
                                 Gamepad &gamepad);
    double _CalculatePurePursuitRadius(cv::Point2f ourPosition, cv::Point2f orbitCenter, double orbitRadius);

    cv::Point2f _NoMoreAggressiveThanTangent(Gamepad &gamepad,
                                             cv::Point2f ourPosition,
                                             cv::Point2f opponentPosEx,
                                             double orbitRadius,
                                             cv::Point2f currentTargetPoint,
                                             bool circleDirection);

    RobotSimState _ExtrapolateOurPos(double seconds_position, double seconds_angle);

    enum class OrbitState
    {
        IDLE = 0,
        LARGE_CIRCLE,
        GO_AROUND
    };

    OrbitState _orbitState = OrbitState::IDLE;
};