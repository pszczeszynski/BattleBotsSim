#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <opencv2/core.hpp>
#include "../MathUtils.h"

class Node {
private:

public:
    Node();
    void resetNode();

    cv::Point cameFrom; // which point precedes this point in the path order
    std::pair<cv::Point2f, Angle> oppPosWhenHere; // where is the opponent when we're at this node
    std::pair<float, float> oppVelWhenHere; // what is the opps move and turn velocity when we're at this node
    float orbMoveVelWhenHere; // what is orbs velocity when we're at this node
    float timeToTraverse; // how long does it take to get here from where we came from

    // for A*
    float gScore;
    float fScore;
};

#endif // NODE_H
