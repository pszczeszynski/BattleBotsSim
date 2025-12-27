#pragma once

#include <opencv2/core.hpp>
#include "../SafeDrawing.h"
#include "FilteredRobot.h"

class FollowPoint {

public:

    FollowPoint();
    FollowPoint(bool forward, bool CW, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath);

    cv::Point2f point; // XY point we're aiming for
    FilteredRobot opp; // opponent robot

    std::vector<cv::Point2f> orbSimPath; // simulated path for orb to the opponent if we were to kill rn
    std::vector<cv::Point2f> oppSimPath; // extrapolated path for opp for where they'll be if we were to kill rn

    bool forward; // if we're driving forwards (or backwards) to the point
    bool CW; // if this point is meant to go CW (or CCW) around the opp

    float controllerGain; // curvature controller gain to use when following
    float radius; // attack radius corresponding to this point, the XY point doesn't always lie on this (ex if orb is close to the opponent in front of their weapon)
    float driveAngle; // angle to drive towards the point

    int enforceTurnDirection; // 1 = always turn right, -1 left, 0 whatever way is less turn distance

private:

    

};