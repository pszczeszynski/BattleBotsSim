#include "Line.h"


// constructor with float points
Line::Line(const cv::Point2f& point1_, const cv::Point2f& point2_) {
    point1 = point1_;
    point2 = point2_;
    length = cv::norm(point2 - point1);

    minX = std::min(point1.x, point2.x);
    maxX = std::max(point1.x, point2.x);
    minY = std::min(point1.y, point2.y);
    maxY = std::max(point1.y, point2.y);

    isVertical = point1.x == point2.x;
    isHorizontal = point1.y == point2.y;
}


// how close does the inputted point get
float Line::howClosePoint(const cv::Point2f& testPoint) {
    
    // Initialize intersection coordinates
    cv::Point2f intersect(0, 0);
    bool onLine;

    if (point1.x == point2.x) {  // Vertical line case
        intersect.x = point1.x;
        intersect.y = testPoint.y;
        onLine = intersect.y > minY && intersect.y < maxY;

    } else if (point1.y == point2.y) {  // Horizontal line case
        intersect.y = point1.y;
        intersect.x = testPoint.x;
        onLine = intersect.x > minX && intersect.x < maxX;

    } else {
        // General case: Calculate slopes
        float slope = (float) (point2.y - point1.y) / (point2.x - point1.x);
        float slope_perp = -1.0f / slope;

        // Intersection of the line between begin and end points and the perpendicular line from the obstacle point
        intersect.x = (slope * point1.x - point1.y - slope_perp * testPoint.x + testPoint.y) / (slope - slope_perp);
        intersect.y = slope * (intersect.x - point1.x) + point1.y;

        // if it's more vertical, use the y coord, otherwise use x coord
        if (slope > 1.0f) { onLine = intersect.y > minY && intersect.y < maxY; }
        else { intersect.x > minX && intersect.x < maxX; }
    }

    // Distance from point to intersection
    float pointDistance;
    
    // if it's not on the line, distance is the distance to the closest start/end point
    if (onLine) { pointDistance = cv::norm(intersect - testPoint); }
    else { pointDistance = std::min(cv::norm(testPoint - point1), cv::norm(testPoint - point2)); }
    
    return pointDistance;
}


// get length for easier calc
float Line::getLength() {
    return length;
}


// does another line intersect this line, covers all cases and exceptions
bool Line::doesIntersectLine(const Line& otherLine) {

    // if this line is vertical
    if (isVertical) {
        if (otherLine.isVertical) { return doesIntersectVerticalVertical(*this, otherLine); }
        if (otherLine.isHorizontal) { return doesIntersectVerticalHorizontal(*this, otherLine); }
        return doesIntersectVerticalDiagonal(*this, otherLine);
    }

    // if this line is horizontal
    if (isHorizontal) {
        if (otherLine.isVertical) { return doesIntersectVerticalHorizontal(otherLine, *this); }
        if (otherLine.isHorizontal) { return doesIntersectHorizontalHorizontal(*this, otherLine); }
        else doesIntersectHorizontalDiagonal(*this, otherLine);
    }

    // otherwise this line is diagonal
    if (otherLine.isVertical) { return doesIntersectVerticalDiagonal(otherLine, *this); }
    if (otherLine.isHorizontal) { return doesIntersectHorizontalDiagonal(otherLine, *this); }
    return doesIntersectDiagonalDiagonal(*this, otherLine); 
}


// do 2 vertical lines intersect
bool Line::doesIntersectVerticalVertical(Line line1, Line line2) {
    bool aligned = line1.point1.x == line2.point1.x;
    if (!aligned) { return false; } // they don't intersect if not aligned

    bool point1Intersect = line2.inYRange(line1.point1.y);
    bool point2Intersect = line2.inYRange(line1.point2.y);
    bool point3Intersect = line1.inYRange(line2.point1.y);
    bool point4Intersect = line1.inYRange(line2.point2.y);

    return point1Intersect || point2Intersect || point3Intersect || point4Intersect;
}


// do 2 horizontal lines intersect
bool Line::doesIntersectHorizontalHorizontal(Line line1, Line line2) {
    bool aligned = line1.point1.y == line2.point1.y;
    if (!aligned) { return false; } // they don't intersect if not aligned

    bool point1Intersect = line2.inXRange(line1.point1.x);
    bool point2Intersect = line2.inXRange(line1.point2.x);
    bool point3Intersect = line1.inXRange(line2.point1.x);
    bool point4Intersect = line1.inXRange(line2.point2.x);

    return point1Intersect || point2Intersect || point3Intersect || point4Intersect;
}


// do a vertical and horizontal line intersect
bool Line::doesIntersectVerticalHorizontal(Line verticalLine, Line horizontalLine) {

    // intersection is defined by the lines directly
    float intersectX = verticalLine.point1.x;
    float intersectY = horizontalLine.point1.y;

    return horizontalLine.inXRange(intersectX) && verticalLine.inYRange(intersectY);
}


// do a vertical and diagonal line intersect
bool Line::doesIntersectVerticalDiagonal(Line verticalLine, Line diagonalLine) {

    float intersectX = verticalLine.point1.x;

    float diagonalSlope = (diagonalLine.point2.y - diagonalLine.point1.y) / (diagonalLine.point2.x - diagonalLine.point1.x);
    float intersectY = diagonalSlope*(intersectX - diagonalLine.point1.x) + diagonalLine.point1.y;

    bool onVertical = verticalLine.inYRange(intersectY);
    bool onDiagonal;

    // if mostly vertical, compare using y component, otherwise x
    if (abs(diagonalSlope) > 1.0f) {
        onDiagonal = diagonalLine.inYRange(intersectY);
    }
    else {
        onDiagonal = diagonalLine.inXRange(intersectX);
    }

    return onVertical && onDiagonal;
}


// do a horizontal and diagonal line intersect
bool Line::doesIntersectHorizontalDiagonal(Line horizontalLine, Line diagonalLine) {

    float intersectY = horizontalLine.point1.y;

    float diagonalSlope = (diagonalLine.point2.y - diagonalLine.point1.y) / (diagonalLine.point2.x - diagonalLine.point1.x);
    float intersectX = ((intersectY - horizontalLine.point1.y) / diagonalSlope) + diagonalLine.point1.x;

    bool onHorizontal = horizontalLine.inXRange(intersectX);
    bool onDiagonal = diagonalLine.inYRange(intersectY) && diagonalLine.inXRange(intersectX);

    return onHorizontal && onDiagonal;
}


// do two diagonal lines intersect
bool Line::doesIntersectDiagonalDiagonal(Line line1, Line line2) {

    // calculate slopes of each
    float slope1 = (line1.point2.y - line1.point1.y) / (line1.point2.x - line1.point1.x);
    float slope2 = (line2.point2.y - line2.point1.y) / (line2.point2.x - line2.point1.x);

    // if they're parallel
    if (slope1 == slope2) {
        bool aligned = line2.point1.y - line1.point1.y == slope1*(line2.point1.x - line1.point1.x);

        // if not aligned, they don't intersect
        if (!aligned) { return false; }

        // if mostly vertical, use y coords to compare, otherwise x
        if (abs(slope1) > 1.0f) {
            bool point1Intersect = line2.inYRange(line1.point1.y);
            bool point2Intersect = line2.inYRange(line1.point2.y);
            bool point3Intersect = line1.inYRange(line2.point1.y);
            bool point4Intersect = line1.inYRange(line2.point2.y);
            return point1Intersect || point2Intersect || point3Intersect || point4Intersect;
        }
        else {
            bool point1Intersect = line2.inXRange(line1.point1.x);
            bool point2Intersect = line2.inXRange(line1.point2.x);
            bool point3Intersect = line1.inXRange(line2.point1.x);
            bool point4Intersect = line1.inXRange(line2.point2.x);
            return point1Intersect || point2Intersect || point3Intersect || point4Intersect;
        }
    }

    // calculate intersect coordiantes, parallel line has already been checked
    float intersectX = (slope1*line1.point1.x - line1.point1.y - slope2*line2.point1.x + line2.point1.y) / (slope1 - slope2);
    float intersectY = slope1*(intersectX - line1.point1.x) + line1.point1.y;


    // check if the intersect point is on the lines
    bool onLine1;
    bool onLine2;

    if (abs(slope1) > 1.0f) { onLine1 = line1.inYRange(intersectY); }
    else { onLine1 = line1.inXRange(intersectX); }

    if (abs(slope2) > 1.0f) { onLine2 = line2.inYRange(intersectY); }
    else { onLine2 = line2.inXRange(intersectX); }

    return onLine1 && onLine2;
}


// is a y value in range of this lines Y
bool Line::inYRange(float yValue) {
    return yValue >= minY && yValue <= maxY;
}


// is an x value in range of this lines X
bool Line::inXRange(float xValue) {
    return xValue >= minX && xValue <= maxX;
}


// returns the line points
std::pair<cv::Point2f, cv::Point2f> Line::getLinePoints() {
    return std::pair<cv::Point2f, cv::Point2f>(point1, point2);
}