#include "FollowPoint.h"



FollowPoint::FollowPoint() { }

FollowPoint::FollowPoint(bool forward, bool CW, bool turnAway, float endAngle, float collisionRad, FilteredRobot opp) {

    this->forward = forward;
    this->CW = CW;
    this->turnAway = turnAway;
    this->opp = opp;
    this->oppSimPath = oppSimPath;

    approachCurve = Approach(opp.position(), endAngle, collisionRad, CW);
    
    point = cv::Point2f(0, 0);
    driveAngle = 0;
    oppETA = 0;
    orbETA = 0;
    enforceTurnDirection = 0;
    worstTimeMargin = 0;
    crossesOppFront = false;

    directionScores = {};
    orbSimPath = {};
    orbSimPathTimes = {};
    directionScores = {};
    wallScanPoints = {};
    oppSimPath = {};
}