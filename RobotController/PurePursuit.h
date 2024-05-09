#pragma once
// PurePursuit.h
// Allows following a path of points using the pure pursuit algorithm. I.e. by intersecting a circle with the path
// and following the furthest along point.


#include <vector>
#include <opencv2/opencv.hpp>
#include "../Common/Communication.h"
#include "RobotOdometry.h"

namespace PurePursuit
{
    // returns the 2 intersections of the circle with the path
    std::vector<cv::Point2f> followPath(cv::Point2f robotPos, const std::vector<cv::Point2f>& pathPoints, double followRadius);
    void CalculateTangentPoints(std::vector<cv::Point2f> &pathPoints,
                                cv::Point2f robot,
                                cv::Point2f opponentWeaponPos,
                                cv::Point2f &outTangent1,
                                cv::Point2f &outTangent2);

    // returns the angle of the closest point on the path
    double GetAngleOfClosestPathSegment(const std::vector<cv::Point2f>& pathPoints, const cv::Point2f& robotPos);
};
