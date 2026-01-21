#pragma once

#include <opencv2/core.hpp>
#include "../SafeDrawing.h"
#include "FilteredRobot.h"

class FollowPoint {

public:

    FollowPoint();
    FollowPoint(bool forward, bool CW, bool turnRight, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath, bool exception);

    cv::Point2f point; // XY point we're aiming for
    FilteredRobot opp; // opponent robot

    std::vector<cv::Point2f> orbSimPath; // simulated path for orb to the opponent if we were to kill rn
    std::vector<cv::Point2f> oppSimPath; // extrapolated path for opp for where they'll be if we were to kill rn

    bool forward; // if we're driving forwards (or backwards) to the point
    bool CW; // if this point is meant to go CW (or CCW) around the opp
    bool turnRight; // if we're curving right (otherwise left) towards the point

    bool exception; // if there's a false turn around that's naturally triggered

    float radius; // attack radius corresponding to this point, the XY point doesn't always lie on this (ex if orb is close to the opponent in front of their weapon)
    float driveAngle; // angle to drive towards the point
    float oppETA; // opp time for this point
    float orbETA; // orb time to hit opp given this points parameters

    std::vector<float> directionScores; // score numbers for how good this follow point is, last entry is the total score

private:

};