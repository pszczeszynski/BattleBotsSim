#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <cmath>
#include <opencv2/core.hpp>
#include <iostream>
#include "ScoreRegion.h"
#include <utility>
#include <algorithm>

class AStar {
public:
    //AStar(const Field& field);
    AStar(int width_, int height_);
    void setWeaponPoints(const cv::Point2f& center, const float& angle);
    void setBoundaryPoints(const std::vector<cv::Point>& boundaryPointList);
    void setStartParams(const cv::Point2f& startF_, const float& orbMoveVelocity, const cv::Point2f& oppPositionF_, const float& oppAngle_, const float& oppMoveVelocity_, const float& oppTurnVelocity_);
    std::vector<cv::Point> generatePath(float robotAngle);
    std::vector<cv::Point> getBoundaryPoints();
    std::vector<cv::Point> getWeaponPoints();

private:
    //Field field;
    cv::Point start;
    cv::Point2f oppPositionF;
    float oppAngle;
    float oppMoveVelocity, oppTurnVelocity;
    float orbVelocity; // tiles per second

    int width, height; // size of grid

    ScoreRegion oppWeapon; // opponent weapon shape


    std::vector<cv::Point> weaponPoints; // opponent weapon points
    std::vector<cv::Point> boundaryPoints; // walls/shelf/screws

    std::vector<cv::Point> openSet; // unorganized list of open set points
    std::vector<cv::Point> closedSet; // unorganized lsit of closed set points

    // lists of data associated with each node
    std::vector<std::vector<cv::Point>> cameFrom;
    std::vector<std::vector<std::pair<cv::Point2f, float>>> oppPosWhenHere;
    std::vector<std::vector<std::pair<float, float>>> oppVelWhenHere;


    // keeps track of g and f scores for every point
    std::vector<std::vector<float>> gScore;
    std::vector<std::vector<float>> fScore;


    float heuristic(const cv::Point& current, const cv::Point& goal);
    float wrapAngle(const float& angle);
    bool doesIntersectPoint(const cv::Point& begin, const cv::Point& endIn, const cv::Point& point, float howClose, const bool& ignoreStart);
    bool doesIntersectList(const std::vector<cv::Point>& pointList, const cv::Point& begin, const cv::Point& endIn, const float& howClose, const bool& ignoreStart);
    std::vector<cv::Point> ringPoints(const int& innerRad, const int& outerRad);
    std::vector<cv::Point> getNeighbors(const cv::Point& point);
    float closestFromList(const std::vector<cv::Point>& pointList, const cv::Point& point);
    float angle(const cv::Point& point1, const cv::Point& point2);
    std::vector<cv::Point> reconstructPath(const cv::Point& current);
    std::vector<cv::Point> difference(const std::vector<cv::Point>& list1, const std::vector<cv::Point>& list2);
    std::pair<int, cv::Point> getOpenSetLowestFScore();
    void extrapolateOpp(cv::Point2f& position, float& angle, float& moveVelocity, float& turnVelocity, float time);
    void constantVelExtrapolate(cv::Point2f& position, float& angle, const float& moveVel, const float& turnVel, const float& time);
    cv::Point floatPointToInt(cv::Point2f point);
    float predictNeighborTime(const cv::Point& current, const cv::Point& neighbor, const float& currentVel, const float& currentAngle);

};

#endif 
