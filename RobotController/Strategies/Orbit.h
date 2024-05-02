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
    double _GetDangerLevel(cv::Point2f robotPos,
                           cv::Point2f opponentWeaponPos,
                           double opponentAngle,
                           bool orbitDirection);

    double _CalculateOrbitRadius(cv::Point2f robotPos,
                                 cv::Point2f orbitWeaponPos,
                                 double opponentAngle,
                                 Gamepad &gamepad,
                                 bool orbitDirection);

    double _CalculatePurePursuitRadius(cv::Point2f ourPosition,
                                       cv::Point2f orbitCenter,
                                       double orbitRadius);

    cv::Point2f _CalculatePathPoint(double angle,
                                    cv::Point2f opponentWeaponPosEx,
                                    cv::Point2f opponentCenterEx,
                                    double opponentAngleEx,
                                    bool circleDirection,
                                    double *outCurrRadius = nullptr,
                                    cv::Point2f *outCurrOrbitCenter = nullptr);

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

    double _startingOrbitRadius = 0;
};