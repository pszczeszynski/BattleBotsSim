#include "Field.h"



// default constructor
Field::Field() {

    // set everything to defaults
    resetBoundsToDefault();

}



// does a test line intersect any boundary line
bool Field::intersectsAnyBound(Line testLine) {
    for(int i = 0; i < boundLines.size(); i++) {
        float howCloseFirstPoint = boundLines[i].howClosePoint(testLine.getLinePoints().first);
        float howCloseSecondPoint = boundLines[i].howClosePoint(testLine.getLinePoints().second);
        if(boundLines[i].doesIntersectLine(testLine) || howCloseFirstPoint < 0.1f || howCloseSecondPoint < 0.1f) { // add tolerance in case point is exactly on line (clipped points often are)
            return true;
        }
    }
    return false;
}



// finds the closest point that's on a boundary line
cv::Point2f Field::closestBoundPoint(cv::Point2f point) {

    std::pair<float, int> closestFieldBound = closestFromLineList(boundLines, point);
    std::pair<cv::Point2f, cv::Point2f> closestLinePoints = boundLines[closestFieldBound.second].getLinePoints();

    cv::Point2f closestWallPoint = boundLines[closestFieldBound.second].closestLinePoint(point);
    return closestWallPoint;
}



// distance to the closest line from the list
std::pair<float, int> Field::closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point) {

    float closest = 999999999.9f; // will always be further away than anything else
    int index = -1;

    // check the whole list and record closest distance
    for (int i = 0; i < lineList.size(); i++) {
        float distance = lineList[i].howClosePoint(point);
        if (distance < closest) { 
            closest = distance; 
            index = i;
        }
    }
    return std::pair<float, int>(closest, index);
}



// if the point is inside the shape of the field
bool Field::insideFieldBounds(cv::Point2f point) {

    // define horizontal line from point
    Line testLine(point, cv::Point2f(point.x + 999999.0f, point.y));

    // count the number of intersections with the bounds
    int intersections = 0;
    for(int i = 0; i < boundLines.size(); i++) {
        if(testLine.doesIntersectLine(boundLines[i])) { intersections++; }
    }

    // point is in the field if there's an odd number of intersections
    return intersections % 2 != 0;
}



// clips a point to be fully in bounds if needed
cv::Point2f Field::clipPointInBounds(cv::Point2f testPoint) {

    cv::Point2f returnPoint = testPoint; 
    if(!insideFieldBounds(returnPoint)) { 
        cv::Point2f onBoundStart = closestBoundPoint(testPoint);

        // 8 surrounding points
        double increment = 0.1f;
        std::vector<cv::Point2f> increments = {
            cv::Point2f(increment, increment),   cv::Point2f(increment, 0),
            cv::Point2f(increment, -increment),  cv::Point2f(0, -increment),
            cv::Point2f(-increment, -increment), cv::Point2f(-increment, 0.0),
            cv::Point2f(-increment, increment),  cv::Point2f(0.0, increment)};

        // see whatever point is in bounds
        for (int i = 0; i < increments.size(); i++) {
          cv::Point2f inBound = cv::Point2f(onBoundStart.x + increments[i].x, onBoundStart.y + increments[i].y);

          // choose whatever point is inside the bounds
          if (insideFieldBounds(inBound)) { returnPoint = inBound; }
        }
    }
    return returnPoint;
}



// lets UI set the boundary points
void Field::setBoundPoints(const std::vector<cv::Point2f>& points) {
    boundPoints = points;
    generateBoundLines();
}



// resets the field lines to whatever the points are
void Field::generateBoundLines() {

    boundLines.clear();
    
    // Regenerate lines from connected adjacent points
    for(int i = 0; i < boundPoints.size() - 1; i++) {
        Line addLine = Line(boundPoints[i], boundPoints[i + 1]);
        boundLines.emplace_back(addLine);
    }
}



// sets boundaries to default values
void Field::resetBoundsToDefault() {

    // default point values
    float minX = 65.0f;
    float maxX = 675.0f;
    float minY = 55.0f;
    float maxY = 655.0f;
    float shelfStartX = 210.0f;
    float shelfEndX = 525.0f;
    float shelfY = 215.0f;
    float screwX1 = 110.0f;
    float screwX2 = 620.0f;
    float screwY1 = 140.0f;
    float screwY2 = 220.0f;
    float screwY3 = 470.0f;
    float screwY4 = 560.0f;
    float gateY = 615.0f;
    float gateX1 = 115.0f;
    float gateX2 = 620.0f;

    std::vector<cv::Point2f> defaultPoints = {
        cv::Point2f(screwX1, shelfY),
        cv::Point2f(screwX2, shelfY),
        cv::Point2f(screwX2, screwY3),
        cv::Point2f(maxX, screwY4),
        cv::Point2f(maxX, gateY),
        cv::Point2f(gateX2, maxY),
        cv::Point2f(gateX1, maxY),
        cv::Point2f(minX, gateY),
        cv::Point2f(minX, screwY4),
        cv::Point2f(screwX1, screwY3),
        cv::Point2f(screwX1, shelfY), // include first point again to complete last line
    };
    
    setBoundPoints(defaultPoints); // set the actual list and lines
}



std::vector<cv::Point2f>& Field::getBoundPoints() { return boundPoints; };
std::vector<Line> Field::getBoundLines() { return boundLines; };