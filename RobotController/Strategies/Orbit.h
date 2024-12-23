#pragma once
#include "Strategy.h"
#include "../Input/Gamepad.h"
#include <opencv2/opencv.hpp>
#include "../Extrapolator.h"
#include "../RobotOdometry.h"


class Orbit : public Strategy
{
public:
    Orbit();

    virtual DriverStationMessage Execute(Gamepad &gamepad) override;
    void StartOrbit();
    void StopOrbit();


private:
    double _GetDangerLevel(double angleOpponentToPoint,
                           double opponentAngle,
                           bool orbitDirection);

    double _CalculateOrbitRadius(double angleOpponentToPoint,
                                 cv::Point2f robotPosEx,
                                 cv::Point2f opponentPosEx,
                                 double opponentAngle,
                                 Gamepad &gamepad,
                                 bool orbitDirection);

    double _CalculatePurePursuitRadius(cv::Point2f ourPosition,
                                       cv::Point2f orbitCenter,
                                       double orbitRadius);

    cv::Point2f _CalculatePathPoint(double angle,
                                    cv::Point2f robotPosEx,
                                    cv::Point2f opponentWeaponPosEx,
                                    cv::Point2f opponentCenterEx,
                                    double opponentAngleEx,
                                    bool circleDirection,
                                    double *outCurrRadius = nullptr,
                                    cv::Point2f *outCurrOrbitCenter = nullptr);

    void _CalcStartAndEndAngle();
    double _START_ANGLE = 0;
    double _END_ANGLE = 0;

    std::vector<cv::Point2f> _CalculateOrbitPath(cv::Point2f opponentWeaponPosEx,
                                                 cv::Point2f opponentCenterEx,
                                                 double opponentAngleEx,
                                                 cv::Point2f robotPosition,
                                                 bool direction,
                                                 bool draw = false);

    cv::Point2f _NoMoreAggressiveThanTangent(Gamepad &gamepad,
                                             cv::Point2f ourPosition,
                                             cv::Point2f opponentPosEx,
                                             double orbitRadius,
                                             cv::Point2f currentTargetPoint,
                                             bool circleDirection);

    RobotSimState _ExtrapolateOurPos(double seconds_position,
                                     double seconds_angle);

    cv::Point2f _GetOrbitFollowPoint(bool direction,
                                     double& outCost,
                                     bool draw = false);


    double _CalcSpiralAggressionPreset();

    double _startingOrbitRadius = 0;

    double _spiralAggression = 0; // 0 to 1, incremented with the triggers
};