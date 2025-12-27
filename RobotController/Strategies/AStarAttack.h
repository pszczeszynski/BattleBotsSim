#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
#include "../PurePursuit.h"
#include <algorithm>
#include <cmath>
#include "Line.h"
#include "FilteredRobot.h"
#include "FollowPoint.h"


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

    FilteredRobot orbFiltered; // orb robot
    FilteredRobot oppFiltered; // opp robot



    std::vector<cv::Point2f> fieldBoundPoints; // list of points that define the field bound lines
    std::vector<Line> fieldBoundLines; // list of lines that defines the outline of the field
    std::vector<cv::Point2f> convexPoints; // list of field bound convex points we could hit while driving

    
    // Note: Radius curve parameters are now in RobotConfig.h as:
    // RADIUS_CURVE_X0, RADIUS_CURVE_X1, RADIUS_CURVE_X2
    // RADIUS_CURVE_Y0, RADIUS_CURVE_Y1, RADIUS_CURVE_Y2
    
    static AStarAttack* _instance;




    std::vector<cv::Point2f> arcPointsFromCenter(float radius, float angle, float pointSpacing);
    void transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle);
    float angle(cv::Point2f point1, cv::Point2f point2);
    void tangentPoint(FollowPoint &follow);
    cv::Point2f ppPoint(FollowPoint follow);
    std::pair<float, int> closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point);
    cv::Point2f closestBoundPoint(cv::Point2f point);
    bool insideFieldBounds(cv::Point2f point);
    bool intersectsAnyBound(Line testLine);
    int vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint);
    void radiusEquation(FollowPoint &follow);
    FollowPoint followPointDirection(FilteredRobot opp, float deltaTime, bool CW, bool forward, std::vector<cv::Point2f> oppSimPath);
    cv::Point2f clipPointInBounds(cv::Point2f testPoint);
    float wallScore(FollowPoint follow);
    float wallScorePinch(FollowPoint follow);
    float turnScore(FollowPoint follow);
    void turnAwayFromOpp(FollowPoint &follow);
    float ppRad();
    float ppRadWall();
    void avoidBoundsVector(FollowPoint &follow);
    float directionScore(FollowPoint follow, float deltaTime, bool forwardInput);
    void followPointInsideCircle(FollowPoint &follow);
    FollowPoint chooseBestPoint(std::vector<FollowPoint> follows, bool forwardInput, float deltaTime);
    float piecewise(std::vector<cv::Point2f> points, float x);
    int sign(float num);
    void commitToTarget(FollowPoint &follow, double deltaTime, float targetTime);
    float driveAngle(FollowPoint follow);
    void display(FollowPoint follow);
    bool willTurnPastOpp(FollowPoint follow);

};