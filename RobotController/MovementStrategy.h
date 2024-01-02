#pragma once

// This class controls the path planning and strategy of the robot

#include <opencv2/opencv.hpp>

class MovementStrategy
{
public:
    MovementStrategy();

    cv::Point2f AvoidStrategy();

    /**
     * Represents a possible node in the path
    */
    struct Node
    {
        cv::Point2f position;
        cv::Point2f velocity;
        Node *parent;
        float gCost;                                  // Cost from start to the current node
        float hCost;                                  // Heuristic cost from the current node to the end
        float estimatedTotalCost() const { return gCost + hCost; } // Total cost
    };

    std::vector<cv::Point2f> PathFindingAStar(cv::Point2f start,
                                              cv::Point2f goal,
                                              cv::Point2f initialMomentum,
                                              cv::Point2f targetApproachDirection,
                                              cv::Mat &wallsMap);

private:
    double EstimateCost(Node *a, Node *b);
    std::vector<Node *> GetNeighbors(Node *node,
                                     Node *goalNode,
                                     std::vector<std::vector<Node>> &nodeGrid,
                                     cv::Mat &wallsMap,
                                     cv::Point2f targetApproachDirection);
};