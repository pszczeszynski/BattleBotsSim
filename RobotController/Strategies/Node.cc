#include "Node.h"
#include <functional>
#include <cmath>

// Constructor
Node::Node() {

    // values here don't matter, just want to initialize to something
    cameFrom = cv::Point(-1, -1);
    oppPosWhenHere = std::pair<cv::Point2f, Angle>(cv::Point2f(0.0f, -0.0f), 0.0f);
    oppVelWhenHere = std::pair<float, float>(0.0f, 0.0f);
    orbMoveVelWhenHere = 0.0f;
    timeToTraverse = 0.0f;

    resetNode();
}



// anything that needs to be reset to an intial value at the start of A* goes here
void Node::resetNode() {
    gScore = std::numeric_limits<double>::infinity();
    fScore = std::numeric_limits<double>::infinity();
}



