#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
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
    
    // Get the last computed followPoints (for debugging/display)
    const std::vector<FollowPoint>& GetFollowPoints() const;
    
    static AStarAttack* GetInstance();


private:

    FilteredRobot orbFiltered; // orb robot
    FilteredRobot oppFiltered; // opp robot
    FilteredRobot orbVirtual; // virtual orb to compare model


    Field field; // field object
    
    FollowPoint prevFollow; // previous follow point we were tracking
    std::vector<FollowPoint> _lastFollowPoints; // last computed follow points (for debugging/display)


    std::vector<float> inputs; // real orb's initial state from last update
    std::vector<float> trueOutputs; // real orb's state going into this update
    std::vector<float> previousGamepad; // what gamepad inputs were last time

    static AStarAttack* _instance;

    float angle(cv::Point2f point1, cv::Point2f point2);
    FollowPoint createFollowPoint(float deltaTime, bool forwardInput, std::vector<bool> enable, std::vector<FollowPoint>& follows, std::vector<FollowPoint>& followsFocussed);
    FollowPoint circleMode(float deltaTime, bool forwardInput);
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
    void controlOrbVirtual(Gamepad& gamepad);

};