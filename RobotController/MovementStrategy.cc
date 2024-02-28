#include "MovementStrategy.h"
#include "RobotController.h"
#include "UIWidgets/ImageWidget.h"

// ctor
MovementStrategy::MovementStrategy()
{

}

/*
Requirements:

1. Don't touch the opponent
2. Don't touch walls
3. Preserve momentum as much as possible
4. iteratively build solution => step by step
5. if at desireable distance away, pace and mess with them to promote attack
*/

/**
 * Plans where the robot should go to avoid the opponent
*/
cv::Point2f MovementStrategy::AvoidStrategy()
{
    // size of each grid cell (pixel in strategy graphic)
    const int STEP_SIZE = 10;
    static ImageWidget widget{"Strategy Graphic", false};
    static cv::Mat strategyGraphic{WIDTH / STEP_SIZE, HEIGHT / STEP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0)};

    cv::Point2f ret = cv::Point2f(0, 0);

    // 1. get as far away from opponent as possible (within limits)

    // get drawing image from rc
    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();

    // clear strategy graphic
    cv::Mat strategyGraphicSmall{WIDTH / STEP_SIZE, HEIGHT / STEP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0)};

    // get our position
    cv::Point2f ourPos = RobotController::GetInstance().odometry.Robot().robotPosition / STEP_SIZE;
    
    // get opponent position
    cv::Point2f opponentPos = RobotController::GetInstance().odometry.Opponent().robotPosition / STEP_SIZE;

    // go through every point in the drawing image with step size of 10
    for (int i = 0; i < strategyGraphicSmall.size().height; i += 1)
    {
        for (int j = 0; j < strategyGraphicSmall.size().width; j += 1)
        {
            // draw a circle at the point. More red means closer to opponent

            // get the point
            cv::Point2f point = cv::Point2f(j, i);

            // get the distance to the opponent
            float distToOpponent = cv::norm(point - opponentPos);

            // 1. get as far away from opponent as possible (within limits)
            double opponentProximityPenalty = 1.0 - distToOpponent / (strategyGraphicSmall.size().width * 0.5);
            opponentProximityPenalty = max(0.0, opponentProximityPenalty);
            opponentProximityPenalty *= opponentProximityPenalty;

            // 2. don't touch walls
            // compute dist to nearest edge
            int distToEdge = min(min(point.x, point.y), min(strategyGraphicSmall.size().width - point.x,
                                                            strategyGraphicSmall.size().height - point.y));
            // compute penalty
            double wallProximityPenalty = 1.0 - distToEdge / (strategyGraphicSmall.size().width * 0.5);

            // square
            wallProximityPenalty *= wallProximityPenalty;

            // put pixel
            strategyGraphicSmall.at<cv::Vec3b>(i, j) = cv::Vec3b(255 * wallProximityPenalty, 0, 100 * opponentProximityPenalty);
        }
    }




    ///////////////// PATH FINDING //////////////////////

    // convert strategy graphic small to CV_32FC1
    cv::Mat strategyGraphicSmallFloat;
    // convert to grayscale
    cv::cvtColor(strategyGraphicSmall, strategyGraphicSmallFloat, cv::COLOR_BGR2GRAY);
    // convert to float
    strategyGraphicSmallFloat.convertTo(strategyGraphicSmallFloat, CV_32FC1);
    // scale by 1 / 255
    strategyGraphicSmallFloat *= 1.0 / 255.0;
#ifdef ASTAR
    // get path from current position to opponent position
    std::vector<cv::Point2f> path = PathFindingAStar(ourPos, opponentPos, strategyGraphicSmallFloat);

    // draw the path on the strategy graphic
    for (int i = 0; i < path.size(); i++)
    {
        cv::Point2f point = path[i];
        strategyGraphicSmall.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(0, 255, 0);
    }
#endif
    ///////////////// END PATH FINDING //////////////////////


    // resize up
    cv::resize(strategyGraphicSmall, strategyGraphic, cv::Size(WIDTH, HEIGHT));

    // // add weighted with drawing image
    // cv::addWeighted(drawingImage, 0.2, strategyGraphic, 0.8, 0, strategyGraphic);

    widget.UpdateMat(strategyGraphic);

    // plot the point to go to
    cv::circle(drawingImage, ret, 5, cv::Scalar(0, 255, 0), 2);

    // return the point to go to
    return ret;
}


// for convenience
typedef MovementStrategy::Node Node;

// Define constants
const float INITIAL_G_COST = 100000000.0f;

/**
* Used to estimate the cost of moving from one node to another
* @param a The first node
* @param b The second node
*/
double MovementStrategy::EstimateCost(Node* a, Node* b)
{
    return cv::norm(a->position - b->position);
}

/**
 * Gets the lowest estimated cost node from a set of nodes
 *
 * @param nodes The set of nodes to search
 * @return The node with the lowest estimated cost
 */
Node *GetBestLookingNode(std::set<Node *> &nodes)
{
    // Sanity check: Ensure the set of nodes is not empty
    if (nodes.empty())
    {
        std::cerr << "Error: The set of nodes is empty." << std::endl;
        return nullptr;
    }

    Node *lowestCostNode = nullptr;

    // For each node in the set
    for (Node *node : nodes)
    {
        // Sanity check: Ensure the node is valid
        if (node == nullptr)
        {
            std::cerr << "Error: Encountered a null node in the set." << std::endl;
            continue;  // Skip this iteration if the node is null
        }

        // If the lowestCostNode is null or the node has a lower estimated cost than the lowestCostNode
        if (!lowestCostNode || node->estimatedTotalCost() < lowestCostNode->estimatedTotalCost())
        {
            // Update the lowestCostNode
            lowestCostNode = node;
        }
    }

    // Check if no valid node was found
    if (lowestCostNode == nullptr)
    {
        std::cerr << "Error: No valid node found in the set." << std::endl;
    }

    // Return the lowestCostNode
    return lowestCostNode;
}

/**
 * Reconstructs the path from the goal node to the start node
 * 
 * @param goalNode The goal node
 * @return The path from the start node to the goal node as a vector of points
*/
std::vector<cv::Point2f> ReconstructPath(Node *goalNode)
{
    // sanity check
    if (goalNode == nullptr)
    {
        std::cerr << "ERROR: goalNode is nullptr" << std::endl;
        return std::vector<cv::Point2f>();
    }

    std::vector<cv::Point2f> path;
    Node *currentNode = goalNode;
    while (currentNode->parent != nullptr)
    {
        path.push_back(currentNode->position);
        currentNode = currentNode->parent;
    }

    // Reverse the path since we started from the goal
    std::reverse(path.begin(), path.end());

    // Return the path
    return path;
}

/**
 * Calculates the gCost of moving from current to neighbor
*/
float CalculateGCost(Node* current, Node* neighbor, cv::Point2f targetApproachDirection)
{
    const float MOMENTUM_COST_WEIGHT = 0.5f;
    const float DIRECTIONAL_COST_WEIGHT = 0.5f;
    const float DISTANCE_COST_WEIGHT = 0.5f;

    float momentumCost = cv::norm(current->velocity - neighbor->velocity);
    float directionalCost = 0/* Calculate cost based on how much the path deviates from targetApproachDirection */;
    float distanceCost = cv::norm(current->position - neighbor->position);

    return momentumCost * MOMENTUM_COST_WEIGHT +
           directionalCost * DIRECTIONAL_COST_WEIGHT +
           distanceCost * DISTANCE_COST_WEIGHT;
}

/**
 * Gets the neighbors of a node
 * 
 * @param node The node to get the neighbors of
 * @param goalNode The goal node
 * @param nodeGrid The grid of nodes
 * @param wallsMap The cost map
 * @param targetApproachDirection The direction to approach the target from
 * @return A vector of node ptrs that are the neighbors of the node
*/
std::vector<Node*> MovementStrategy::GetNeighbors(Node* node, Node* goalNode, std::vector<std::vector<Node>>& nodeGrid, cv::Mat& wallsMap, cv::Point2f targetApproachDirection)
{
    std::vector<Node*> neighbors;
    std::array<int, 8> dx = {-1, 0, 1, -1, 1, -1, 0, 1};
    std::array<int, 8> dy = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int i = 0; i < 8; i++)
    {
        int newX = node->position.x + dx[i];
        int newY = node->position.y + dy[i];

        // Check if the neighbor is within the bounds of the cost map
        if (newX >= 0 && newX < wallsMap.cols && newY >= 0 && newY < wallsMap.rows)
        {
            neighbors.push_back(&nodeGrid[newY][newX]);
        }
    }

    // Update the gCost and hCost for each neighbor
    for (Node* neighbor : neighbors)
    {
        // Update hCost for the neighbor
        neighbor->hCost = EstimateCost(neighbor, goalNode);

        // Update gCost for the neighbor considering momentum and target approach direction
        neighbor->gCost = CalculateGCost(node, neighbor, targetApproachDirection);
    }

    return neighbors;
}

/**
 * Checks if a point is reachable (i.e. not a wall)
 * Walls have a cost of 1.0
 * 
 * @param point The point to check
 * @param wallsMap The cost map
 * @return True if the point is reachable, false otherwise
*/
bool IsReachable(cv::Point2f point, cv::Mat &wallsMap)
{
    const double COST_THRESHOLD = 1.0;

    // Check if the point is within the bounds of the cost map
    if (point.x < 0 || point.y < 0 || point.x >= wallsMap.cols || point.y >= wallsMap.rows)
    {
        return false;
    }

    // Check if the cost at the point is below the threshold
    // wallsMap is a single-channel floating-point image
    return wallsMap.at<float>(point.y, point.x) < COST_THRESHOLD;
}

/**
 * Plans a path from start to end using A*
 *
 * @param start The starting point
 * @param goal The goal point
 * @param initialMomentum The initial momentum of the robot
 * @param targetApproachDirection The direction to approach the target from
 * @param wallsMap The cost map
 */
std::vector<cv::Point2f> MovementStrategy::PathFindingAStar(cv::Point2f start,
                                                            cv::Point2f goal,
                                                            cv::Point2f initialMomentum,
                                                            cv::Point2f targetApproachDirection,
                                                            cv::Mat &wallsMap)
{
    // validation: wallsMap must be CV_32FC1
    if (wallsMap.type() != CV_32FC1)
    {
        std::cerr << "ERROR: wallsMap must be CV_32FC1 (" << CV_32FC1 << ")" << std::endl;
        // print the actual type
        std::cerr << "Actual type: " << wallsMap.type() << std::endl;
        return std::vector<cv::Point2f>();
    }

    // validation: start and goal must be within the bounds of the cost map
    if (start.x < 0 || start.y < 0 || start.x >= wallsMap.cols || start.y >= wallsMap.rows ||
        goal.x < 0 || goal.y < 0 || goal.x >= wallsMap.cols || goal.y >= wallsMap.rows)
    {
        std::cerr << "ERROR: start and goal must be within the bounds of the cost map" << std::endl;
        return std::vector<cv::Point2f>();
    }

    // Create a 2D grid of nodes
    std::vector<std::vector<Node>> nodeGrid(wallsMap.rows, std::vector<Node>(wallsMap.cols));
    for (int y = 0; y < wallsMap.rows; ++y)
    {
        for (int x = 0; x < wallsMap.cols; ++x)
        {
            nodeGrid[y][x].position = cv::Point2f(x, y);
            nodeGrid[y][x].gCost = INITIAL_G_COST;
            nodeGrid[y][x].hCost = 0;
            nodeGrid[y][x].parent = nullptr;
        }
    }

    Node* startNode = &nodeGrid[start.y][start.x];
    startNode->parent = nullptr;
    startNode->gCost = 0;
    startNode->hCost = 0;
    startNode->velocity = initialMomentum;

    Node* goalNode = &nodeGrid[goal.y][goal.x];
    goalNode->parent = nullptr;
    goalNode->gCost = 0;
    goalNode->hCost = 0;

    // estimate the cost from the start to the goal
    startNode->hCost = EstimateCost(startNode, goalNode);

    // nodes to be evaluated
    std::set<Node*> nodesToExplore;
    // nodes already evaluated
    std::set<Node*> exploredNodes;

    // Add the start node to nodesToExplore
    nodesToExplore.insert(startNode);

    // While there are nodes to explore
    while (!nodesToExplore.empty())
    {
        // Get the node with the lowest estimated cost
        Node* currentNode = GetBestLookingNode(nodesToExplore);

        // Check if we reached the goal
        if (currentNode->position == goalNode->position)
        {
            return ReconstructPath(currentNode);
        }

        // move the chosen node from nodesToExplore to exploredNodes
        nodesToExplore.erase(currentNode);
        exploredNodes.insert(currentNode);

        // Get neighbors of the current node
        std::vector<Node *> neighbors = GetNeighbors(currentNode, goalNode,
                                                     nodeGrid, wallsMap,
                                                     targetApproachDirection);

        for (Node *neighbor : neighbors)
        {
            // If the neighbor is already explored or is not reachable, skip it
            if (exploredNodes.find(neighbor) != exploredNodes.end() ||
                !IsReachable(neighbor->position, wallsMap))
            {
                continue;
            }

            // Calculate the new gCost for the neighbor
            float newGCost = currentNode->gCost + cv::norm(currentNode->position - neighbor->position);

            // If the new gCost is lower than the neighbor's gCost or the neighbor is not in nodesToExplore
            if (newGCost < neighbor->gCost || nodesToExplore.find(neighbor) == nodesToExplore.end())
            {
                // Update the neighbor's gCost and parent
                neighbor->gCost = newGCost;
                neighbor->parent = currentNode;

                // Add the neighbor to nodesToExplore
                nodesToExplore.insert(neighbor);
            }
        }
    }

    // Return an empty path if no path found
    return std::vector<cv::Point2f>();
}
