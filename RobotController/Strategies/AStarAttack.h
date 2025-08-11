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
    
    // Field boundary editing interface
    std::vector<cv::Point2f>& GetFieldBoundaryPoints();
    void SetFieldBoundaryPoints(const std::vector<cv::Point2f>& points);
    void RegenerateFieldBoundaryLines();
    void ResetFieldBoundariesToDefault();
    
    // Radius curve parameter interface
    void GetRadiusCurvePoints(float radiusCurveX[3], float radiusCurveY[3]);
    void SetRadiusCurvePoints(const float radiusCurveX[3], const float radiusCurveY[3]);
    void ResetRadiusCurveToDefault();
    
    static AStarAttack* GetInstance();

private:

    // filtered data for orb and opp
    FilteredRobot orbFiltered;
    FilteredRobot oppFiltered;



    std::vector<cv::Point2f> fieldBoundPoints; // list of points that define the field bound lines
    std::vector<Line> fieldBoundLines; // list of lines that defines the outline of the field
    std::vector<cv::Point2f> convexPoints; // list of field bound convex points we could hit while driving

    bool currForward; // what we're currently leading with
    bool CW; // which way we're currently going around the circle
    float prevAngleError;
    
    // Note: Radius curve parameters are now in RobotConfig.h as:
    // RADIUS_CURVE_X0, RADIUS_CURVE_X1, RADIUS_CURVE_X2
    // RADIUS_CURVE_Y0, RADIUS_CURVE_Y1, RADIUS_CURVE_Y2
    
    static AStarAttack* _instance;



    void displayPathPoints(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayPathLines(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayLineList(std::vector<Line>& lines, cv::Scalar color);
    void displayFieldBoundIndices(std::vector<int> indices, cv::Scalar color);
    void displayPathTangency(FilteredRobot robot, cv::Scalar color);

    std::vector<cv::Point2f> arcPointsFromCenter(float radius, float angle, float pointSpacing);
    void transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle);
    float angle(cv::Point2f point1, cv::Point2f point2);
    cv::Point2f tangentPoint(float radius, bool CW);
    cv::Point2f ppPoint(float radius, bool CW, float ppRadius);
    std::pair<float, int> closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point);
    cv::Point2f closestBoundPoint(cv::Point2f point);
    bool insideFieldBounds(cv::Point2f point);
    bool intersectsAnyBound(Line testLine);
    int vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint);
    float calculateMovePercent(cv::Point2f followPoint, float angleThresh1, float angleThresh2, bool forward);
    float radiusEquation(bool forward, bool CW);
    cv::Point2f followPointDirection(bool CW, bool forward);
    cv::Point2f clipPointInBounds(cv::Point2f testPoint);
    float wallScore(bool CW);
    cv::Point2f turnCorrectWay(cv::Point2f followPointRaw, bool forward);
    float ppRad();
    cv::Point2f avoidBounds(cv::Point2f rawFollowPoint);
    float directionScore(cv::Point2f followPoint, bool CW, bool forward);
    cv::Point2f followPointInsideCircle(float radius, float ppRadius, bool CW, bool forward, float collisionRadius);
    cv::Point2f chooseBestPoint(std::vector<cv::Point2f> followPoints, std::vector<bool> pointsCW, std::vector<bool> pointsForward, bool& CW, bool& forward);
    cv::Point2f predictDriftStop(bool forward);
    float piecewise(std::vector<cv::Point2f> points, float x);
    std::vector<float> curvatureController(cv::Point2f followPoint, float moveSpeed, float deltaTime);
};