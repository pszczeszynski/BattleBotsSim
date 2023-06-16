#pragma once
#include <vector>
#include "RobotTracker.h"
#include "OpponentProfile.h"
#include <opencv2/core.hpp>

class PathFinder
{
public:
    PathFinder();
    cv::Point2f PathFinder::GetMotionVector(cv::Point2f opponentPosition, double opponentAngle, OpponentProfile& opponentProfile, cv::Point2f pos);
    cv::Point2f PathFinder::GetMotionVectorConvex(cv::Point2f opponentPosition, double opponentAngle, OpponentProfile& opponentProfile, cv::Point2f pos, cv::Mat& drawingImage);

    cv::Point2f purePursuit(std::vector<cv::Point2f> &path, cv::Point2f robotPos, double lookaheadDistance, cv::Mat &image);

};