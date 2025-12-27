#include "FollowPoint.h"



FollowPoint::FollowPoint() { }

FollowPoint::FollowPoint(bool forward, bool CW, FilteredRobot opp, std::vector<cv::Point2f> oppSimPath) {

    this->forward = forward;
    this->CW = CW;
    this->opp = opp;
    this->oppSimPath = oppSimPath;
    
    radius = 0; // default
    controllerGain = 1.0f; // default
    point = cv::Point2f(0, 0); // default
    enforceTurnDirection = 0; // default
    driveAngle = 0; // default
    
    std::vector<cv::Point2f> orbSimPath = {}; // default

}