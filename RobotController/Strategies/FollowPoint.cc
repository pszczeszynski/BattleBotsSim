#include "FollowPoint.h"



FollowPoint::FollowPoint() { }

FollowPoint::FollowPoint(bool forward, bool CW, bool turnRight, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath, bool exception) {

    this->forward = forward;
    this->CW = CW;
    this->turnRight = turnRight;
    this->opp = opp;
    this->oppSimPath = oppSimPath;
    this->exception = exception;
    
    radius = 0; // default
    point = cv::Point2f(0, 0); // default
    driveAngle = 0; // default
    oppETA = 0;
    orbETA = 0;

    directionScores = {}; // default
    orbSimPath = {}; // default
}