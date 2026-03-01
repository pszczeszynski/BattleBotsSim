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
    FilteredRobot orbVirtual; // virtual orb to compare model


    Field field; // field object
    
    std::vector<FollowPoint> _lastFollowPoints; // last computed follow points (for debugging/display)


    std::vector<float> inputs; // real orb's initial state from last update
    std::vector<float> trueOutputs; // real orb's state going into this update
    std::vector<float> previousGamepad; // what gamepad inputs were last time

    
    // Note: Radius curve parameters are now in RobotConfig.h as:
    // RADIUS_CURVE_X0, RADIUS_CURVE_X1, RADIUS_CURVE_X2, RADIUS_CURVE_X3
    // RADIUS_CURVE_Y0, RADIUS_CURVE_Y1, RADIUS_CURVE_Y2, RADIUS_CURVE_Y3
    
    static AStarAttack* _instance;



    float angle(cv::Point2f point1, cv::Point2f point2);
    FollowPoint createFollowPoint(float deltaTime, bool forwardInput, std::vector<bool> enable, std::vector<FollowPoint>& follows, std::vector<FollowPoint>& followsFocussed);
    std::vector<float> generateEndAngles(bool CW);
    void orbToOppPath(FollowPoint &follow);
    void oppToOrbETA(FollowPoint &follow);
    float wallScore(FollowPoint &follow);
    float ppRad(float speed);
    float ppRadWall();
    void avoidBoundsVector(FollowPoint &follow);
    void followScore(FollowPoint &follow, bool forwardInput);
    int sign(float num);
    void driveAngle(FollowPoint &follow);
    void display(FollowPoint follow, std::vector<FollowPoint> follows, std::vector<FollowPoint> followsFocussed);
    void controlOrbVirtual(bool autoTune, bool resetState, bool resetModel);

};