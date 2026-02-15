#include "FollowPoint.h"



FollowPoint::FollowPoint() { }

FollowPoint::FollowPoint(bool forward, bool CW, bool turnAway, float simRadGain, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath) {

    this->forward = forward;
    this->CW = CW;
    this->turnAway = turnAway;
    this->opp = opp;
    this->oppSimPath = oppSimPath;
    this->simRadGain = simRadGain;
    
    point = cv::Point2f(0, 0);
    driveAngle = 0;
    oppETA = 0;
    orbETA = 0;
    enforceTurnDirection = 0;
    endingAngle = 0;
    crossesOppFront = false;
    hit = false;

    directionScores = {};
    orbSimPath = {};
    orbSimPathTimes = {};
    directionScores = {};
    orbSimFollowPoints = {};
    approach = {};
}