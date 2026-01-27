#include "FollowPoint.h"



FollowPoint::FollowPoint() { }

FollowPoint::FollowPoint(bool forward, bool CW, bool turnAway, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath) {

    this->forward = forward;
    this->CW = CW;
    this->turnAway = turnAway;
    this->opp = opp;
    this->oppSimPath = oppSimPath;
    
    radius = 0;
    point = cv::Point2f(0, 0);
    driveAngle = 0;
    oppETA = 0;
    orbETA = 0;
    inflectDistance = 0;
    simRadGain = 0;

    directionScores = {};
    orbSimPath = {};
}