#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
#include "../PurePursuit.h"
#include <algorithm>
#include <cmath>
#include "Line.h"
#include "FilteredRobot.h"
#include "FollowPoint.h"
#include "Field.h"


class AStarAttack : public Strategy
{
public:
    AStarAttack();
    DriverStationMessage Execute(Gamepad& gamepad, double rightStickY);
    // Call this version if if you want to force the stick y
    DriverStationMessage Execute(Gamepad& gamepad) override {
        return Execute(gamepad, gamepad.GetRightStickY());
    }
    
    // Field boundary editing interfac
    std::vector<cv::Point2f>& GetFieldBoundaryPoints();
    void SetFieldBoundaryPoints(const std::vector<cv::Point2f>& points);
    void ResetFieldBoundariesToDefault();
    void RegenerateFieldBoundaryLines();
    
    // Radius curve parameter interface
    void GetRadiusCurvePoints(float radiusCurveX[4], float radiusCurveY[4]);
    void SetRadiusCurvePoints(const float radiusCurveX[4], const float radiusCurveY[4]);
    void ResetRadiusCurveToDefault();
    
    // Get the last computed followPoints (for debugging/display)
    const std::vector<FollowPoint>& GetFollowPoints() const;
    
    static AStarAttack* GetInstance();

private:

    FilteredRobot orbFiltered; // orb robot
    FilteredRobot oppFiltered; // opp robot


    Field field; // field object
    
    std::vector<FollowPoint> _lastFollowPoints; // last computed follow points (for debugging/display)


    
    // Note: Radius curve parameters are now in RobotConfig.h as:
    // RADIUS_CURVE_X0, RADIUS_CURVE_X1, RADIUS_CURVE_X2, RADIUS_CURVE_X3
    // RADIUS_CURVE_Y0, RADIUS_CURVE_Y1, RADIUS_CURVE_Y2, RADIUS_CURVE_Y3
    
    static AStarAttack* _instance;




    std::vector<cv::Point2f> arcPointsFromCenter(float radius, float angle, float pointSpacing);
    void transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle);
    float angle(cv::Point2f point1, cv::Point2f point2);
    cv::Point2f tangentPoint(float radius, cv::Point2f center, cv::Point2f point, bool CW);
    cv::Point2f ppPoint(FollowPoint follow);
    std::pair<float, int> closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point);
    int vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint);
    void radiusEquation(FollowPoint &follow);
    FollowPoint createFollowPoint(bool CW, bool forward, bool turnAway, float deltaTime);
    void generatePath(FollowPoint &follow);
    float wallScore(FollowPoint follow);
    float turnScore(FollowPoint follow);
    float ppRad();
    float ppRadWall();
    // void avoidBoundsVector(FollowPoint &follow);
    void directionScore(FollowPoint &follow, bool forwardInput);
    void followPointInsideCircle(FollowPoint &follow);
    FollowPoint chooseBestPoint(std::vector<FollowPoint>& follows, bool forwardInput);
    float piecewise(std::vector<cv::Point2f> points, float x);
    int sign(float num);
    void commitToTarget(FollowPoint &follow, double deltaTime, float targetTime);
    void driveAngle(FollowPoint &follow);
    void display(FollowPoint follow);
    bool willTurnPastOpp(FollowPoint follow);
    float switchPointScore(FollowPoint follow);

};