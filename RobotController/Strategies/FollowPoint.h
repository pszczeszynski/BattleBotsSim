#pragma once

#include <opencv2/core.hpp>
#include "../SafeDrawing.h"
#include "FilteredRobot.h"

class FollowPoint {

public:

    FollowPoint();
    FollowPoint(bool forward, bool CW, bool turnAway, float simRadGain, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath);

    cv::Point2f point; // XY point we're aiming for
    FilteredRobot opp; // opponent robot

    std::vector<cv::Point2f> orbSimPath; // simulated path for orb to the opponent for this point's settings
    std::vector<float> orbSimPathTimes; // how long it takes orb to get to each point in the sim path
    std::vector<cv::Point2f> orbSimFollowPoints; // list of follow points used to generate the sim path
    std::vector<cv::Point2f> approach; // curve that defines the approach to opp
    std::vector<cv::Point2f> oppSimPath; // extrapolated path for opp for where they'll be if we were to kill rn

    // these variables are what make a follow point unique
    bool forward; // if we're driving forwards (or backwards) to the point
    bool CW; // if this point is meant to go CW (or CCW) around the opp
    bool turnAway; // if this point is making sure to turn away from opp at first
    float simRadGain; // the rad gain used in sim to generate the path for this point

    float driveAngle; // angle to drive when this point is active
    float oppETA; // opp time for this point
    float orbETA; // orb time to hit opp given this points parameters
    float endingAngle; // angle at which the approach path targets opp

    int enforceTurnDirection; // forces curvature control to turn a certain way, used for walls mainly

    std::vector<float> directionScores; // score numbers for how good this follow point is, last entry is the total score

private:

};