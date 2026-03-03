#pragma once

#include <opencv2/core.hpp>
#include <string>
#include "../SafeDrawing.h"
#include "FilteredRobot.h"
#include "Approach.h"



class FollowPoint {

public:

    FollowPoint();
    FollowPoint(bool forward, bool CW, bool turnAway, float endAngle, float collisionRad, FilteredRobot opp);

    cv::Point2f point; // XY point we're aiming for
    FilteredRobot opp; // opponent robot

    std::vector<cv::Point2f> orbSimPath; // simulated path for orb to the opponent for this point's settings
    std::vector<float> orbSimPathTimes; // how long it takes orb to get to each point in the sim path
    std::vector<cv::Point2f> oppSimPath; // extrapolated path for opp for where they'll be if we were to kill rn
    std::vector<cv::Point2f> wallScanPoints; // points generated in wall score algorithm

    // these variables are what make a follow point unique
    bool forward; // if we're driving forwards (or backwards) to the point
    bool CW; // if this point is meant to go CW (or CCW) around the opp
    bool turnAway; // if this point is making sure to turn away from opp at first
    Approach approachCurve; // defines the path to the opp to collide at desired angle

    float worstTimeMargin; // how much can the opp beat us to unsafe points on the path
    float driveAngle; // angle to drive when this point is active
    float oppETA; // opp time for this point
    float orbETA; // orb time to hit opp given this points parameters
    int enforceTurnDirection; // forces curvature control to turn a certain way, used for walls mainly
    bool crossesOppFront; // if we cross by the opp's front during the sim path

    std::vector<float> directionScores; // score numbers for how good this follow point is, last entry is the total score
    std::vector<std::string> directionScoreNames; // names of the scores for each direction

private:

};