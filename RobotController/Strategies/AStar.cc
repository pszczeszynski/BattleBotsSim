#include "AStar.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include "PairHash.h"
#include "../Clock.h"

#define M_PI 3.14159

AStar::AStar(int width, int height)
    : width(width), height(height) {

    // build the vectors
    for (int y = 0; y < height; ++y) {
        gScore.push_back(std::vector<float>());
        fScore.push_back(std::vector<float>());
        cameFrom.push_back(std::vector<cv::Point>());

        // fill the arrays with default values
        for (int x = 0; x < width; ++x) {
            float score = std::numeric_limits<double>::infinity();
            gScore[y].push_back(score);
            fScore[y].push_back(score);

            cv::Point silly(-1, -1);
            cameFrom[y].push_back(silly);
        }
    }
}


// sets the start and goal points
void AStar::setStartGoal(const cv::Point& start_, const cv::Point& goal_) {
    start = start_;
    goal = goal_;


    // init all locations with gScore and fScore of infinity
    // init all locations with came from something just for memes
    for (int y = 0; y < height; ++y) {

        // fill the arrays with default values
        for (int x = 0; x < width; ++x) {
            float score = std::numeric_limits<double>::infinity();
            gScore[y][x] = score;
            fScore[y][x] = score;

            cv::Point silly(-1, -1);
            cameFrom[y][x] = silly;
        }
    }

    // cost to get to start node is nothing
    gScore[start.x][start.y] = 0;

    // therefore, total score of start is just heuristic
    fScore[start.x][start.y] = heuristic(start, goal);

    // add start to the open set to initialize the search
    openSet = {};
    openSet.emplace_back(start);

    closedSet = {};
}


// retrieves point with lowest F score in open list
std::pair<int, cv::Point> AStar::getOpenSetLowestFScore() {
    float lowestScore = std::numeric_limits<double>::infinity();
    cv::Point lowestPoint(0, 0);
    int lowestIndex = -1;
    for(int i = 0; i < openSet.size(); i++) {
        cv::Point openSetPoint = openSet[i];
        float pointFScore = fScore[openSetPoint.x][openSetPoint.y];
        if(pointFScore < lowestScore) {
            lowestScore = pointFScore;
            lowestPoint = openSetPoint;
            lowestIndex = i;
        }
    }
    // organize point and index into pair for return
    std::pair<int, cv::Point> pair(lowestIndex, lowestPoint);
    return pair;
}

// fills the obstacle list with the inputted list
void AStar::setWeaponPoints(const std::vector<cv::Point>& obstacleList) {
    weaponPoints = obstacleList;
}

// fills the boundary list with the inputted list
void AStar::setBoundaryPoints(const std::vector<cv::Point>& boundaryPointList) {
    boundaryPoints = boundaryPointList;
}

float AStar::heuristic(const cv::Point& current, const cv::Point& goal) {
    return cv::norm(current - goal);
}

float AStar::wrapAngle(const float& angle) {
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

// does a point get too close to a line segment connecting two nodes
bool AStar::doesIntersectPoint(const cv::Point& start, const cv::Point& end, const cv::Point& point, const float& howClose) {

    // if the point is out of the square then it def doesn't intersect
    float minX = std::min(start.x, end.x) - howClose;
    float maxX = std::max(start.x, end.x) + howClose;
    float minY = std::min(start.y, end.y) - howClose;
    float maxY = std::max(start.y, end.y) + howClose;

    if (point.x < minX || point.x > maxX || point.y < minY || point.y > maxY) {
        //std::cout << "wut" << std::endl;
        return false;
    }

    // Initialize intersection coordinates
    cv::Point intersect(0, 0);

    if (start.x == end.x) {  // Vertical line case
        intersect.x = start.x;
        intersect.y = point.y;
    } else if (start.y == end.y) {  // Horizontal line case
        intersect.y = start.y;
        intersect.x = point.x;
    } else {
        // General case: Calculate slopes
        float slope = (end.y - start.y) / (end.x - start.x);
        float slope_perp = -1.0f / slope;

        // Intersection of the line between start and end points and the perpendicular line from the obstacle point
        intersect.x = (slope * start.x - start.y - slope_perp * point.x + point.y) / (slope - slope_perp);
        intersect.y = slope * (intersect.x - start.x) + start.y;
    }

    // Distance from point to intersection
    double point_distance = cv::norm(point - intersect);

    // Check for intersection condition
    if (point_distance < howClose) {
        return true;
    }

    return false;
}

// do any obstacles from the point list get too close to the line between two points
bool AStar::doesIntersectList(std::vector<cv::Point> pointList, const cv::Point& start, const cv::Point& end, const float& howClose) {

    for (int i = 0; i < pointList.size(); i++) {
        if (doesIntersectPoint(start, end, pointList[i], howClose)) {
            return true;
        }
    }
    return false;
}


// generate a list of pixelated circle points
std::vector<cv::Point> AStar::getCirclePoints(const int& radius) {

    std::vector<cv::Point> points;
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            if (dx * dx + dy * dy <= radius * radius) {
                cv::Point pointToAdd(dx, dy);
                points.emplace_back(pointToAdd);
            }
        }
    }
    return points;
}


// Function to filter points that are in the first list but not in the second
std::vector<cv::Point> AStar::difference(const std::vector<cv::Point>& list1, const std::vector<cv::Point>& list2) {
    std::vector<cv::Point> result;

    // Iterate through the first list
    for (const auto& point : list1) {
        // Check if the point is not in the second list
        if (std::find(list2.begin(), list2.end(), point) == list2.end()) {
            result.push_back(point);  // Add point to result if it's not found in list2
        }
    }
    return result;
}



// finds valid/non colliding neighbors to existing node
std::vector<cv::Point> AStar::getNeighbors(const cv::Point& point) {

    // Calculate the distance to the goal
    float distanceToGoal = cv::norm(goal - point);

    // Determine outer and inner radii
    int outerRad = std::round(distanceToGoal * 0.6);
    outerRad = std::min(std::max(outerRad, 2), 6); // minimum of 2, max of 6
    int innerRad = std::max(outerRad - 2, 0);

    // outerRad = 5;
    // innerRad = 3;

    // Generate the outer and inner circles
    // TODO: make this calc directly
    std::vector<cv::Point> outerCircle = getCirclePoints(outerRad);
    std::vector<cv::Point> innerCircle = getCirclePoints(innerRad);
    std::vector<cv::Point> ring = difference(outerCircle, innerCircle);

    //std::cout << "ring = " << ring << std::endl;

    // Vector to store valid neighbors
    std::vector<cv::Point> neighbors;

    // Iterate through the ring to find valid neighbors
    for (const auto& delta : ring) {
        cv::Point neighbor(point.x + delta.x, point.y + delta.y);

        //std::cout << "test neighbor = " << neighbor << ", does intersect = " << doesIntersectList(point, neighbor, 1);
        // if (doesIntersectList(point, neighbor, 1)) {
        //     std::cout << "intersection" << std::endl;
        // }

        //std::cout << "boundary check = " << doesIntersectList(boundaryPoints, point, neighbor, 1) << std::endl;
        std::cout << "boundary points = " << boundaryPoints << std::endl;

        // Check if the neighbor is within bounds and doesn't intersect weapon or boudaries when line connected
        if (neighbor.x >= 0 && neighbor.x < width && neighbor.y >= 0 && neighbor.y < height && !doesIntersectList(weaponPoints, point, neighbor, 0.51) ) {
            neighbors.emplace_back(neighbor);
            //std::cout << "adding" << std::endl;
        }
    }
    //std::cout << "return neighbors: " << neighbors << std::endl;
    return neighbors;
}

// distance to closest point in the inputted list
float AStar::closestFromList(const std::vector<cv::Point>& pointList, const cv::Point& point) {

    float closest = width + height; // will always be further away than any other point

    // loop through all obstacles, record the closest distance
    for (int i = 0; i < pointList.size(); i++) {
        double distance = cv::norm(point - pointList[i]);
        if (distance < closest) {
            closest = distance;
        }
    }
    return closest;
}

// absolute angle made by points, referenced from horizontal at point1
float AStar::angle(const cv::Point& point1, const cv::Point& point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}

// returns boundary points
std::vector<cv::Point> AStar::getBoundaryPoints() {
    return boundaryPoints;
}

// A* path recreation
std::vector<cv::Point> AStar::reconstructPath(const cv::Point& current) {
    std::vector<cv::Point> path;
    cv::Point curr = current;
    while (cameFrom[curr.x][curr.y] != cameFrom[start.x][start.y]) {
        path.push_back(curr);
        curr = cameFrom[curr.x][curr.y];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// run AStar algorithm to find path
std::vector<cv::Point> AStar::generatePath() {
    //std::cout << "start = " << start.x << ", " << start.y << "      goal = " << goal.x << ", " << goal.y << std::endl;
    
    if (openSet.empty()) {
        std::cout << "starting empty" << std::endl;
    }
    //Clock totalTime;

    while (!openSet.empty()) {

        // retrieve lowest F score point from the open list
        std::pair<int, cv::Point> lowestFScore = getOpenSetLowestFScore();
        int currentIndex = lowestFScore.first;
        cv::Point current = lowestFScore.second;
        openSet.erase(openSet.begin() + currentIndex); // remove point from open set since now we're exploring it

        // if we've found the goal then the path is done
        if (current == goal) {
            //std::cout << "reconstructing at " << current.x << ", " << current.y << std::endl;

            std::vector<cv::Point> ret = reconstructPath(current);
            //std::cout << "while loop total time: " << totalTime.getElapsedTime() << std::endl;
            return ret;
        }

        // add current node to closed set to mark it as explored
        closedSet.emplace_back(current);

        //std::cout << "neighbors = " << getNeighbors(current) << std::endl;

        for (const auto& neighbor : getNeighbors(current)) {

            // if the neighbor is in the closed set already, skip this iteration
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) { 
                continue;
            }

            // lower score = preferred
            float tentativeGScore = gScore[current.x][current.y] + 
            (10 / pow(closestFromList(weaponPoints, neighbor), 1.2) * pow(cv::norm(current - neighbor), 0.9)) + // opponent weapon weight
            (1 / closestFromList(boundaryPoints, neighbor) * cv::norm(current - neighbor)); // boundaries weight


            if (tentativeGScore < gScore[neighbor.x][neighbor.y]) {
                cameFrom[neighbor.x][neighbor.y] = current;
                //std::cout << "here" << std::endl;
                gScore[neighbor.x][neighbor.y] = tentativeGScore;
                fScore[neighbor.x][neighbor.y] = tentativeGScore + heuristic(neighbor, goal);

                // add neighbor to open set so it's explored later
                openSet.emplace_back(neighbor);
            }

            
        }
    }

    std::cout << "no path" << std::endl;
    return {}; // No path found
}