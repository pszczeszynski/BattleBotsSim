#include "ScoreRegion.h"
#include <cmath>
#include <string>
#include "../MathUtils.h"

// Constructor: Initializes the grid with the given region
ScoreRegion::ScoreRegion()
{
    return;
}

ScoreRegion::ScoreRegion(std::vector<Line> lines_)
{
    lines = lines_;
}


// translates lines to field coords and returns
std::vector<Line> ScoreRegion::fieldLines(const cv::Point2f& position, const float& theta) {

    // return vector to fill
    std::vector<Line> fieldLines = {};

    for (int i = 0; i < lines.size(); i++) {

        // get line points, these are robot centric
        std::pair<cv::Point2f, cv::Point2f> linePoints = lines[i].getLinePoints();

        cv::Point2f fieldPoint1 = pointRobotToField(linePoints.first, position, theta);
        cv::Point2f fieldPoint2 = pointRobotToField(linePoints.second, position, theta);

        // add the new line to the list
        fieldLines.push_back(Line(fieldPoint1, fieldPoint2));
    }

    return fieldLines;
}



// converts a robot centric point to a field centric point
cv::Point2f ScoreRegion::pointRobotToField(const cv::Point2f& point, const cv::Point2f& robotPos, const float& robotAngle) {

    float fieldX = robotPos.x + point.x * cos(robotAngle - M_PI / 2) - point.y * sin(robotAngle - M_PI / 2);
    float fieldY = robotPos.y + point.x * sin(robotAngle - M_PI / 2) + point.y * cos(robotAngle - M_PI / 2);

    return cv::Point2f(fieldX, fieldY);
}
