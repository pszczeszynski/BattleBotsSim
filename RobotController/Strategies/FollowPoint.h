#pragma once

#include <opencv2/core.hpp>
#include "../SafeDrawing.h"
#include "FilteredRobot.h"

class FollowPoint {

public:

    FollowPoint();
    FollowPoint(bool forward, bool CW, bool turnAway, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath);

    cv::Point2f point; // XY point we're aiming for
    FilteredRobot opp; // opponent robot

    std::vector<cv::Point2f> orbSimPath; // simulated path for orb to the opponent if we were to kill rn
    std::vector<cv::Point2f> oppSimPath; // extrapolated path for opp for where they'll be if we were to kill rn

    std::vector<float> orbSimPathTimes; // how long it takes orb to get to each point in the sim path
    int inflectIndex; // sim path index at which the opp starts being on the correct side

    bool forward; // if we're driving forwards (or backwards) to the point
    bool CW; // if this point is meant to go CW (or CCW) around the opp
    bool turnAway; // if this point is making sure to turn away from opp at first

    float radius; // attack radius corresponding to this point, the XY point doesn't always lie on this (ex if orb is close to the opponent in front of their weapon)
    float driveAngle; // angle to drive towards the point
    float oppETA; // opp time for this point
    float orbETA; // orb time to hit opp given this points parameters
    float simRadGain; // the rad gain used in sim to generate the path for this point

    std::vector<float> directionScores; // score numbers for how good this follow point is, last entry is the total score

private:

};