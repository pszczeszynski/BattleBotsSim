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



    void displayPathPoints(std::vector<cv::Point2f>& path, cv::Scalar color);
    void displayPathLines(std::vector<cv::Point2f>& path, cv::Scalar color);

    std::vector<cv::Point2f> arcPointsFromOrigin(float radius, float angle, float pointSpacing);
    void transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle);
    std::vector<cv::Point2f> angleLineFromPoint(cv::Point2f start, float length, float angle);
    std::vector<cv::Point2f> addPaths(std::vector<cv::Point2f> path1, std::vector<cv::Point2f> path2);
    std::vector<cv::Point2f> reversePath(std::vector<cv::Point2f> path);
    float angle(cv::Point2f point1, cv::Point2f point2);
    std::pair<std::vector<cv::Point2f>, float> generatePath(bool clockwise, cv::Point2f oppPos, float oppAngle, cv::Point2f orbPos, float closingSpeed, float ppRadius, float vMax, float wMax);
    float arcTime(float radius, float theta, float vMax, float wMax);

};