#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <cmath>
#include "Field.h"
#include <opencv2/core.hpp>

class AStar {
public:
    //AStar(const Field& field);
    AStar(int width, int height);
    void setWeaponPoints(const std::vector<cv::Point>& weaponPointList);
    void setBoundaryPoints(const std::vector<cv::Point>& boundaryPointList);
    void setStartGoal(const cv::Point& start_, const cv::Point& goal_);
    std::vector<cv::Point> generatePath();
    std::vector<cv::Point> getBoundaryPoints();

private:
    //Field field;
    cv::Point start, goal;
    int width, height;

    std::vector<cv::Point> weaponPoints; // weapon points (wants to keep distance from them)
    std::vector<cv::Point> boundaryPoints; // walls/shelf/screws (can't intersect but can go close)

    std::vector<cv::Point> openSet; // unorganized list of open set points
    std::vector<cv::Point> closedSet; // unorganized lsit of closed set points

    // used to retrace final path
    std::vector<std::vector<cv::Point>> cameFrom;

    // keeps track of g and f scores for every point
    std::vector<std::vector<float>> gScore;
    std::vector<std::vector<float>> fScore;


    float heuristic(const cv::Point& current, const cv::Point& goal);
    float wrapAngle(const float& angle);
    bool doesIntersectPoint(const cv::Point& start, const cv::Point& end, const cv::Point& point, const float& howClose);
    bool doesIntersectList(std::vector<cv::Point> pointList, const cv::Point& start, const cv::Point& end, const float& howClose);
    std::vector<cv::Point> getCirclePoints(const int& radius);
    std::vector<cv::Point> getNeighbors(const cv::Point& point);
    float closestFromList(const std::vector<cv::Point>& pointList, const cv::Point& point);
    float angle(const cv::Point& point1, const cv::Point& point2);
    std::vector<cv::Point> reconstructPath(const cv::Point& current);
    std::vector<cv::Point> difference(const std::vector<cv::Point>& list1, const std::vector<cv::Point>& list2);
    std::pair<int, cv::Point> getOpenSetLowestFScore();
};

#endif 
