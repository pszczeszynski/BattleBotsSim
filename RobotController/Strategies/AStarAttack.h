#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
#include "ScoreRegion.h"
#include "AStar.h"
#include "../PurePursuit.h"
#include <algorithm>
#include <cmath>
#include "Line.h"
#include "FilteredRobot.h"

class AStarAttack : public Strategy
{
public:
    AStarAttack();
    virtual DriverStationMessage Execute(Gamepad& gamepad) override;


private:

    // filtered data for orb and opp
    FilteredRobot orbFiltered;
    FilteredRobot oppFiltered;

    // for simulating drift
    float oppAngleDrift;
    float oppAnglePreviousPerfect;

    // tracking stats
    float maxOppTurnAccel;
    float maxOppTurnSpeed;


    // bruh
    float previousMetricCW = 0.0f;
    float previousMetricCCW = 0.0f;
    float previousAngleToOrb = 0.0f;


    std::vector<cv::Point2f> fieldBoundPoints; // list of points that define the field bound lines
    std::vector<Line> fieldBoundLines; // list of lines that defines the outline of the field
    std::vector<cv::Point2f> convexPoints; // list of field bound convex points we could hit while driving

    bool leadingWithBar; // what we're currently leading with



    void displayPoints(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayLines(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayLineList(std::vector<Line>& lines, cv::Scalar color);
    void displayFieldBoundIndices(std::vector<int> indices, cv::Scalar color);

    std::vector<cv::Point2f> arcPointsFromCenter(float radius, float angle, float pointSpacing);
    void transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle);
    float angle(cv::Point2f point1, cv::Point2f point2);
    cv::Point2f tangentPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW);
    cv::Point2f ppPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW, float ppRadius, float rejoinAngle);
    std::pair<float, int> closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point);
    cv::Point2f closestBoundPoint(cv::Point2f point);
    bool insideFieldBounds(cv::Point2f point);
    bool intersectsAnyBound(Line testLine);
    int vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint);
    float calculateMovePercent(cv::Point2f orbPos, float orbAngle, cv::Point2f followPoint, float angleThresh1, float angleThresh2, bool leadingWithBar);
    float radiusEquation(float deltaTime, float ppRadius, bool CW);
    cv::Point2f followPointDirection(float deltaTime, float ppRadius, bool CW, bool display);
    cv::Point2f clipPointInBounds(cv::Point2f testPoint);
    float closestWallDistance(bool CW);
    float metric(bool CW, bool draw);
    cv::Point2f turnCorrectWay(cv::Point2f followPoint);
};