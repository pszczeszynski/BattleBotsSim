#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
#include "ScoreRegion.h"
#include "AStar.h"
#include "../PurePursuit.h"
#include <algorithm>
#include <cmath>
#include "Line.h"

class AStarAttack : public Strategy
{
public:
    AStarAttack();

    virtual DriverStationMessage Execute(Gamepad& gamepad) override;


private:

    float oppMoveVelFilter = 0.0f;
    float oppTurnVelFilter = 0.0f;
    float orbMoveVelFilter = 0.0f;
    float previousDistanceToOpp = 0.0f;
    float speedToOppFilter = 0.0f;

    std::vector<cv::Point2f> orbPath; // tracks where orb is
    float orbPathSpacing; // how far apart are the points in the orb path
    float orbPathLength; // how long the total tracked path is

    std::vector<Line> fieldBounds; // list of lines that defines the outline of the field
    std::vector<cv::Point2f> convexPoints; // list of field bound convex points we could hit while driving



    void displayPoints(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayLines(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayLineList(std::vector<Line>& lines, cv::Scalar color);
    void displayFieldBoundIndices(std::vector<int> indices, cv::Scalar color);
    void displayFieldBounds();

    std::vector<cv::Point2f> arcPointsFromOrigin(float radius, float angle, float pointSpacing);
    std::vector<cv::Point2f> arcPointsFromCenter(float radius, float angle, float pointSpacing);
    void transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle);
    std::vector<cv::Point2f> angleLineFromPoint(cv::Point2f start, float length, float angle);
    std::vector<cv::Point2f> addPaths(std::vector<cv::Point2f> path1, std::vector<cv::Point2f> path2);
    std::vector<cv::Point2f> reversePath(std::vector<cv::Point2f> path);
    float angle(cv::Point2f point1, cv::Point2f point2);
    std::pair<std::vector<cv::Point2f>, float> generatePath(bool clockwise, cv::Point2f oppPos, float oppAngle, cv::Point2f orbPos, float closingSpeed, float ppRadius, float vMax, float wMax);
    float arcTime(float radius, float theta, float vMax, float wMax);

    cv::Point2f tangentPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW);
    cv::Point2f ppPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW, float ppRadius, float rejoinAngle, float distanceToOpp);
    cv::Point2f clipInBounds(cv::Point2f point, float xMin, float xMax, float yMin, float yMax);
    std::pair<float, int> closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point);
    cv::Point2f closestBoundPoint(cv::Point2f point);
    bool insideFieldBounds(cv::Point2f point);
    bool intersectsAnyBound(Line testLine);
    int vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint);

};