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
    // Constructor
    Line(const cv::Point2f& point1_, const cv::Point2f& point2_);
    float getLength();
    float howClosePoint(const cv::Point2f& testPoint);
    bool doesIntersectLine(const Line& otherLine);
    std::pair<cv::Point2f, cv::Point2f> getLinePoints();


private:

    cv::Point2f point1; // start of the line
    cv::Point2f point2; // end of the line

    // min and max coords on each dimension
    float minX;
    float maxX;
    float minY;
    float maxY;

    float length; // length of line

    // saves for exception cases
    bool isVertical;
    bool isHorizontal;

    bool doesIntersectVerticalVertical(Line line1, Line line2);
    bool doesIntersectHorizontalHorizontal(Line line1, Line line2);
    bool doesIntersectVerticalHorizontal(Line verticalLine, Line horizontalLine);
    bool doesIntersectVerticalDiagonal(Line verticalLine, Line diagonalLine);
    bool doesIntersectHorizontalDiagonal(Line horizontalLine, Line diagonalLine);
    bool doesIntersectDiagonalDiagonal(Line line1, Line line2);
    bool inYRange(float yValue);
    bool inXRange(float xValue);
};

