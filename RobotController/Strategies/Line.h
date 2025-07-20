#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <algorithm>
#include "../SafeDrawing.h"
#include <utility>
class Line {
public:
    /// Construct a segment from p1 to p2
    Line(const cv::Point2f& p1, const cv::Point2f& p2);

    /// Closest point on the segment to P
    cv::Point2f closestLinePoint(const cv::Point2f& P) const;

    /// Distance from P to the segment
    float howClosePoint(const cv::Point2f& P) const;

    /// Length of the segment
    float getLength() const;

    /// Does this segment intersect another?
    bool doesIntersectLine(const Line& other) const;

    /// Return the endpoints as a pair
    std::pair<cv::Point2f, cv::Point2f> getLinePoints() const;

private:
    cv::Point2f point1, point2;  // endpoints
    cv::Point2f _d;              // = point2 - point1
    float       _len2;           // squared length of _d
    float       length;          // = sqrt(_len2)
};
    