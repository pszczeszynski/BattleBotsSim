#include "AStar.h"
#include <limits>
#include "../Clock.h"

#define M_PI 3.14159

AStar::AStar(int width_, int height_)
{

    width = width_;
    height = height_; 

    float weaponWidth = 0.16f * width; // 0.12
    float weaponHeight = 0.01f * width; // 0.06
    float weaponForwardOffset = 0.04f * width; // 0.07

    // opponent weapon shape
    std::vector<Line> weaponLines = {
        Line(cv::Point2f(-weaponWidth/2, weaponForwardOffset), cv::Point2f(weaponWidth/2, weaponForwardOffset)), // bottom
        Line(cv::Point2f(0.0f, weaponForwardOffset + weaponHeight), cv::Point2f(-weaponWidth/2, weaponForwardOffset)), // left diagonal
        Line(cv::Point2f(weaponWidth/2, weaponForwardOffset), cv::Point2f(0.0f, weaponForwardOffset + weaponHeight)) // right diagonal
    };

    

    oppWeapon = ScoreRegion(weaponLines);
    

    // build the vectors
    for (int y = 0; y < height; ++y) {
        // gScore.push_back(std::vector<float>());
        // fScore.push_back(std::vector<float>());
        // cameFrom.push_back(std::vector<cv::Point>());
        // oppPosWhenHere.push_back(std::vector<std::pair<cv::Point2f, float>>());
        // oppVelWhenHere.push_back(std::vector<std::pair<float, float>>());
        // orbVelWhenHere.push_back(std::vector<float>());

        nodeData.push_back(std::vector<Node>());

        // fill the arrays with default values
        for (int x = 0; x < width; ++x) {
            // float score = std::numeric_limits<double>::infinity();
            // gScore[y].push_back(score);
            // fScore[y].push_back(score);

            // // fill in came from list with arbitrary points, value doesn't matter
            // cv::Point silly(-1, -1);
            // cameFrom[y].push_back(silly);

            // // fill in opponent pos list with arbitrary values, values don't matter
            // std::pair<cv::Point2f, float> silly2(cv::Point2f(-1, -1), 0.0f);
            // oppPosWhenHere[y].push_back(silly2);

            // // fill in opponent vel list with arbitrary values, values don't matter
            // std::pair<float, float> silly3(0.0f, 0.0f);
            // oppVelWhenHere[y].push_back(silly3);
            // orbVelWhenHere[y].push_back(0.0f);

            nodeData[y].push_back(Node());
        }
    }
}

// converts a Point2f to a Point
cv::Point AStar::floatPointToInt(cv::Point2f point) {
    return cv::Point(round(point.x), round(point.y));
}


// sets the nitial values for path parameters
void AStar::setStartParams(const cv::Point2f& startF_, const float& orbMoveVelocity, const cv::Point2f& oppPositionF_, const float& oppAngle_, const float& oppMoveVelocity_, const float& oppTurnVelocity_) {
    start = floatPointToInt(startF_);
    oppPositionF = oppPositionF_;
    oppStartAngle = oppAngle_;
    oppMoveVelocity = oppMoveVelocity_;
    oppTurnVelocity = oppTurnVelocity_;

    
    // reset all nodes
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            nodeData[x][y].resetNode();
        }
    }

    // init start parameters
    nodeData[start.x][start.y].gScore = 0.0f;
    nodeData[start.x][start.y].fScore = heuristic(start, floatPointToInt(oppPositionF));
    nodeData[start.x][start.y].timeToTraverse = 0.0f;
    nodeData[start.x][start.y].oppPosWhenHere = std::pair<cv::Point2f, Angle>(oppPositionF, oppStartAngle);
    nodeData[start.x][start.y].oppVelWhenHere = std::pair<float, float>(oppMoveVelocity, oppTurnVelocity);
    nodeData[start.x][start.y].orbMoveVelWhenHere = orbMoveVelocity;


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
        float pointFScore = nodeData[openSetPoint.x][openSetPoint.y].fScore;
        //fScore[openSetPoint.x][openSetPoint.y];
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
void AStar::setWeaponLines(const cv::Point2f& center, const float& angle) {
    oppWeaponLines = oppWeapon.fieldLines(center, angle);

    // update weapon points too
    oppWeaponPoints = {};
    for (int i = 0; i < oppWeaponLines.size(); i++) {
        oppWeaponPoints.push_back(oppWeaponLines[i].getLinePoints().first);
    }
}

// fills the boundary list with the inputted list
void AStar::setBoundaryLines(const std::vector<Line>& boundaryLineList) {
    boundaryLines = boundaryLineList;
}

float AStar::heuristic(const cv::Point& current, const cv::Point& goal) {
    return cv::norm(current - goal) / 3.0;
}

float AStar::wrapAngle(const float& angle) {

    float mod = std::fmod(angle + M_PI, 2 * M_PI);
    if (mod < 0) { mod += 2*M_PI; }
    return mod - M_PI;
}

// does a point get too close to a line segment connecting two nodes
bool AStar::doesIntersectPoint(const cv::Point& begin, const cv::Point& endIn, const cv::Point2f& point, float howClose, const bool& ignoreStart) {

    bool firstOfPath = (begin == start);    
    if (howClose >= 1.0f) {
        // exception for the final opp point, just needs to be 1 tile away, another exception for start position
        cv::Point2f oppPos = nodeData[begin.x][begin.y].oppPosWhenHere.first;
        //oppPosWhenHere[begin.x][begin.y].first;
        if (endIn == cv::Point(round(oppPos.x), round(oppPos.y)) || firstOfPath) {
        //if (endIn == cv::Point(round(oppPos.x), round(oppPos.y)) && howClose >= 1.0f) {
            howClose = 0.99f;
        }
    }
    

    // if the point is out of the square then it def doesn't intersect
    float minX = std::min(begin.x, endIn.x) - howClose;
    float maxX = std::max(begin.x, endIn.x) + howClose;
    float minY = std::min(begin.y, endIn.y) - howClose;
    float maxY = std::max(begin.y, endIn.y) + howClose;

    if ((float) point.x < minX || (float) point.x > maxX || (float) point.y < minY || (float) point.y > maxY) {
        return false;
    }

    // ignore if the begin point is too close and we're not ignoring the start
    cv::Point2f beginF(begin.x, begin.y);
    if (cv::norm(beginF - point) < howClose && (ignoreStart || firstOfPath)) { 
    //if (cv::norm(begin - point) < howClose && ignoreStart) { 
        return false;
    }

    // // ignore if the end point is close to the goal
    // if (cv::norm(point - goal) < howClose) {
    //     return false;
    // }

    // Initialize intersection coordinates
    cv::Point2f intersect(0, 0);

    if (begin.x == endIn.x) {  // Vertical line case
        intersect.x = begin.x;
        intersect.y = point.y;
    } else if (begin.y == endIn.y) {  // Horizontal line case
        intersect.y = begin.y;
        intersect.x = point.x;
    } else {
        // General case: Calculate slopes
        float slope = (float) (endIn.y - begin.y) / (endIn.x - begin.x);
        float slope_perp = -1.0f / slope;

        // Intersection of the line between begin and end points and the perpendicular line from the obstacle point
        intersect.x = (slope * begin.x - begin.y - slope_perp * point.x + point.y) / (slope - slope_perp);
        intersect.y = slope * (intersect.x - begin.x) + begin.y;
    }

    // Distance from point to intersection
    float point_distance = sqrt(pow(intersect.x - point.x, 2) + pow(intersect.y - point.y, 2));

    if (point_distance < howClose) {
        return true;
    }

    return false;
}

// do any obstacles from the point list get too close to the line between two points
bool AStar::doesIntersectList(const std::vector<cv::Point2f>& pointList, const cv::Point& begin, const cv::Point& endIn, const float& howClose, const bool& ignoreStart) {

    // check all the points on the list, if any intersect return true
    for (int i = 0; i < pointList.size(); i++) {
        if (doesIntersectPoint(begin, endIn, pointList[i], howClose, ignoreStart)) {
            return true;
        }
    }
    return false;
}

// does a line you're trying to drive through intersect any lines from the inputted list (boundaries or weapon)
bool AStar::doesIntersectLineList(std::vector<Line> lineList, Line testLine) {

    // check all lines in the list, if any intersect return true
    for (int i = 0; i < lineList.size(); i++) {
        if (testLine.doesIntersectLine(lineList[i])) {
            return true;
        }
    }
    return false;
}


// generate a list of pixelated circle points
std::vector<cv::Point> AStar::ringPoints(const int& innerRad, const int& outerRad) {

    std::vector<cv::Point> points;
    for (int dx = -outerRad; dx <= outerRad; ++dx) {
        for (int dy = -outerRad; dy <= outerRad; ++dy) {

            // if it's inside the outer circle and outside the inner circle
            if (pow(dx, 2) + pow(dy, 2) <= pow(outerRad, 2) && pow(dx, 2) + pow(dy, 2) >= pow(innerRad, 2)) {
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
    float distanceToGoal = cv::norm(floatPointToInt(nodeData[point.x][point.y].oppPosWhenHere.first) - point);

    // // Determine outer and inner radii
    // int outerRad = round(distanceToGoal * 0.6);
    // outerRad = min(max(outerRad, 2), 12); // minimum of 2, max of 6
    // int innerRad = max(outerRad - 2, 0); // minimum of 0

    int outerRad = 4;
    int innerRad = 0;

    // generate ring points
    std::vector<cv::Point> ring = ringPoints(innerRad, outerRad);

    // Iterate through the ring to find valid neighbors
    std::vector<cv::Point> neighbors;
    for (const auto& delta : ring) {
        cv::Point neighbor(point.x + delta.x, point.y + delta.y);

        // Check if the neighbor is within bounds and doesn't intersect weapon or boudaries when line connected
        if (neighbor.x >= 0 && neighbor.x < width && neighbor.y >= 0 && neighbor.y < height && !doesIntersectLineList(oppWeaponLines, Line(point, neighbor)) && !doesIntersectLineList(boundaryLines, Line(point, neighbor))) {
            neighbors.emplace_back(neighbor);
        }
    }
    return neighbors;
}


// distance to closest point in the inputted list
float AStar::closestFromList(const std::vector<cv::Point2f>& pointList, const cv::Point2f& point) {

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


// distance to the closest line from the list
float AStar::closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point) {

    float closest = width + height; // will always be further away than anything else

    // check the whole list and record closest distance
    for (int i = 0; i < lineList.size(); i++) {
        float distance = lineList[i].howClosePoint(point);
        if (distance < closest) { closest = distance; }
    }

    return closest;
}


// absolute angle made by points, referenced from horizontal at point1
float AStar::angle(const cv::Point& point1, const cv::Point& point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}

// absolute angle made by pointsF
float AStar::angleF(const cv::Point2f& point1, const cv::Point2f& point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}

// returns boundary points
std::vector<Line> AStar::getBoundaryLines() {
    return boundaryLines;
}


// A* path recreation
std::vector<cv::Point> AStar::reconstructPath(const cv::Point& current) {

    std::vector<cv::Point> path; // path is a list of points
    float pathTime = 0.0f;

    cv::Point curr = current;

    //std::cout << "final opp angle = " << nodeData[current.x][current.y].oppPosWhenHere.second << std::endl;

    for (int i = 0; i < width*height; i++) {
        path.push_back(curr);
        pathTime += nodeData[curr.x][curr.y].timeToTraverse;
        //std::cout << "node time = " << nodeData[curr.x][curr.y].timeToTraverse << std::endl;
        if (curr == start) { break; } // if we've reached and added the start node, the path is done

        curr = nodeData[curr.x][curr.y].cameFrom;
    }
    std::cout << "path time = " << pathTime << std::endl;
    std::reverse(path.begin(), path.end());
    return path;

}

// returns the wepaon point list
std::vector<Line> AStar::getWeaponLines() {
    return oppWeaponLines;
}


// does an arc extrapolation of constant input velocities
void AStar::constantVelExtrapolate(cv::Point2f& position, float& angle, const float& moveVel, const float& turnVel, const float& time) {

    float dX;
    float dY;
    float dTheta = turnVel * time;

    // calculate deltas
    if (turnVel == 0) {
        dX = moveVel * cos(angle) * time;
        dY = moveVel * sin(angle) * time;
    }
    else {
        dX = (moveVel/turnVel) * (sin(turnVel*time + angle) - sin(angle));
        dY = (-moveVel/turnVel) * (cos(turnVel*time + angle) - cos(angle)); 
    }
    
    // write new positions, check for wall clipping
    position = cv::Point2f(std::max(std::min(position.x + dX, (float) width - 2), 1.0f), std::max(std::min(position.y + dY, (float) height - 2), 1.0f));
    angle = wrapAngle(angle + dTheta);

    if (abs(angle) > M_PI) {
        std::cout << "over pi2 = " << angle << std::endl;
    }
}


// predicts time required for a neighbor node, rough approximation
float AStar::predictNeighborTime(const cv::Point& current, const cv::Point& neighbor, float& vel, const float& currentAngle) {

    float angleToNeighbor = angle(current, neighbor);
    float turnAngle = abs(wrapAngle(angleToNeighbor - currentAngle));
    if (current == start) {
        turnAngle = std::min(turnAngle, 60.0f*(3.14159f/180.0f));
    }
    float neighborDistance = cv::norm(neighbor - current);

    float time = (neighborDistance / vel) * (1 + 2.5*std::min(pow(turnAngle/M_PI, 2), 1.0));

    // basic proportional guess at turn radius
    float radiusGuess = neighborDistance / turnAngle;

    if (turnAngle < 30.0f * (M_PI/180.0f)) {
        vel *= 1.0;
    }
    else {
        vel *= 1.0f;
    }
    vel = std::min(vel, width*0.6f);


    return time;
}


// updates opponent position assuming constant decel parameters by reference
void AStar::extrapolateOpp(cv::Point2f& oppPosition, const cv::Point2f& orbPosition, float& oppAngle, float& moveVelocity, float& turnVelocity, float time) {

    int timeSteps = 10;
    float oppMoveAccelMag = 1.3f * width; // tiles/sec^2
    float oppTurnAccelMag = 50; // rad/sec^2
    float pointShootMoveVel = 0.05f * width;
    float pointShootTurnVel = 50.0f * (M_PI/180.0f); // rad/sec^2
    float maxExtropolateAngle = 100.0f * (M_PI/180.0f); // max point and shoot turn angle itll predict, lets the path solve

    cv::Point2f previousPosition;
    float previousAngle;
    float previousMoveVel;
    float previousTurnVel;

    // simulate through every timestep
    for (int i = 0; i < timeSteps; i++) {

        // save in case a collision is found
        previousPosition = oppPosition;
        previousAngle = oppAngle;
        previousMoveVel = moveVelocity;
        previousTurnVel = turnVelocity;


        float angleToOrb = wrapAngle(angleF(oppPosition, orbPosition) - oppAngle);        
        int orbDirection = 1;
        if (angleToOrb < 0) { orbDirection = -1; }

        // if (orbPosition == cv::Point2f(start.x, start.y)) {
        //     std::cout << "orb angle = " << angleToOrb << std::endl;
        // } 

        if (abs(oppAngle) > M_PI) {
            std::cout << "over pi = " << oppAngle << std::endl;
        }

        // goal is to turn at this velocity in the point and shoot direction
        float targetTurnVel = pointShootTurnVel * orbDirection;
        float targetMoveVel = pointShootMoveVel;

        // move and turn velocities will increment by this much
        float turnVelIncrement = oppTurnAccelMag*(time/timeSteps);
        float moveVelIncrement = oppMoveAccelMag*(time/timeSteps);

        // if the velocities are closer than the increments, just set them directly
        // otherwise, add the accel in the right direction
        if (abs(targetTurnVel - turnVelocity) < turnVelIncrement) { turnVelocity = targetTurnVel; }
        else if (turnVelocity < targetTurnVel) { turnVelocity += turnVelIncrement; }
        else { turnVelocity -= turnVelIncrement; }

        if (abs(targetMoveVel - moveVelocity) < moveVelIncrement) { moveVelocity = targetMoveVel; }
        else if (moveVelocity < targetMoveVel) { moveVelocity += moveVelIncrement; }
        else { moveVelocity -= moveVelIncrement; }

        // just turn towards him bro
        turnVelocity = pointShootTurnVel*orbDirection;
        moveVelocity = pointShootMoveVel;

            




        // extrapolate in an arc with current velocities
        constantVelExtrapolate(oppPosition, oppAngle, moveVelocity, turnVelocity, time/timeSteps);

        float angleToOrbAfter = wrapAngle(angleF(oppPosition, orbPosition) - oppAngle); 

        // if opp turned too far, then just set the angle directly on orb
        if (angleToOrb * angleToOrbAfter < 0) { oppAngle = wrapAngle(oppAngle + angleToOrbAfter); }

        // if (oppAngle - (oppStartAngle + maxExtropolateAngle) > 0.0f) { 
        //     oppAngle = wrapAngle(oppStartAngle + maxExtropolateAngle); 
        //     std::cout << "too far left" << std::endl;
        // }
        // if (oppAngle - (oppStartAngle - maxExtropolateAngle) < 0.0f) { 
        //     oppAngle = wrapAngle(oppStartAngle - maxExtropolateAngle); 
        //     std::cout << "too far right" << std::endl;
        // }

        // see where the weapon would be at this position
        std::vector<Line> potentialWeapon = oppWeapon.fieldLines(oppPosition, oppAngle);
        
        float closestPotentialWeapon = width + height; // will always be higher
        for (int i = 0; i < potentialWeapon.size(); i++) {
            cv::Point2f startF(start.x, start.y);
            float distanceToStart = potentialWeapon[i].howClosePoint(startF);
            // if a weapon point is too close then stop extrapolating
            if (distanceToStart < 0.06f * width) {
                oppPosition = previousPosition;
                oppAngle = previousAngle;
                moveVelocity = previousMoveVel;
                turnVelocity = previousTurnVel;
                continue;
            }
        }
    }
}

// run AStar algorithm to find path
std::vector<cv::Point> AStar::generatePath(float robotAngle) {

    while (!openSet.empty()) {

        // retrieve lowest F score point from the open list
        std::pair<int, cv::Point> lowestFScore = getOpenSetLowestFScore();
        int currentIndex = lowestFScore.first;
        cv::Point current = lowestFScore.second;
        openSet.erase(openSet.begin() + currentIndex); // remove point from open set since now we're exploring it

        // set weapon for current node
        setWeaponLines(nodeData[current.x][current.y].oppPosWhenHere.first, nodeData[current.x][current.y].oppPosWhenHere.second);

        // if we've found the goal then the path is done
        if (current == floatPointToInt(nodeData[current.x][current.y].oppPosWhenHere.first)) { 
            return reconstructPath(current);
        }

        

        // add current node to closed set to mark it as explored
        closedSet.emplace_back(current);

        for (const auto& neighbor : getNeighbors(current)) {

            // if the neighbor is in the closed set already, skip this iteration
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) { 
                continue;
            }

            float neighborDistance = cv::norm(current - neighbor);
                        
            // find the closest weapon distance
            float closestWeapon = width + height;
            Line neighborLine(current, neighbor);
            for (int i = 0; i < oppWeaponPoints.size(); i++) {
                float distance = neighborLine.howClosePoint(oppWeaponPoints[i]);
                if (distance < closestWeapon) {
                    closestWeapon = distance;
                }
            }


            float angleChange = 0;
            float previousAngle = 0; // use the robot's angle as a default
            float angleWeight = 0;
            float neighborAngle = angle(current, neighbor);
            if (current == start) {
                previousAngle = robotAngle;
                angleChange = std::min(abs(wrapAngle(neighborAngle - previousAngle)), 45.0f*(3.14159f/180.0f));
            }
            else { // if it's not the start node, look back for previous angle
                //cameFrom[current.x][current.y]
                previousAngle = angle(nodeData[current.x][current.y].cameFrom, current); // angle of previous line
                angleChange = abs(wrapAngle(neighborAngle - previousAngle));             
            }


            angleWeight = 3.0f * angleChange * neighborDistance;



            float neighborOrbVel = nodeData[current.x][current.y].orbMoveVelWhenHere; //orbVelWhenHere[current.x][current.y]
            float neighborTime = predictNeighborTime(current, neighbor, neighborOrbVel, previousAngle);
            //float tentativeGScore = nodeData[current.x][current.y].gScore + (10 / pow(closestWeapon, 0.6)) * pow(neighborTime, 1.0);
            float tentativeGScore = nodeData[current.x][current.y].gScore + neighborDistance + angleWeight;


            // if the score is better than commit the node
            if (tentativeGScore < nodeData[neighbor.x][neighbor.y].gScore) {

                nodeData[neighbor.x][neighbor.y].cameFrom = current;
                nodeData[neighbor.x][neighbor.y].gScore = tentativeGScore;
                nodeData[neighbor.x][neighbor.y].fScore = tentativeGScore + heuristic(neighbor, floatPointToInt(nodeData[current.x][current.y].oppPosWhenHere.first));
                nodeData[neighbor.x][neighbor.y].timeToTraverse = neighborTime;


                // save the predicted positions and velocities at this neighbor
                cv::Point2f oppPredictedPosition = nodeData[current.x][current.y].oppPosWhenHere.first;
                float oppPredictedAngle = nodeData[current.x][current.y].oppPosWhenHere.second;
                float oppPredictedMoveVelocity = nodeData[current.x][current.y].oppVelWhenHere.first;
                float oppPredictedTurnVelocity = nodeData[current.x][current.y].oppVelWhenHere.second;
                extrapolateOpp(oppPredictedPosition, neighbor, oppPredictedAngle, oppPredictedMoveVelocity, oppPredictedTurnVelocity, neighborTime);
                nodeData[neighbor.x][neighbor.y].oppPosWhenHere = std::pair<cv::Point2f, Angle>(oppPredictedPosition, oppPredictedAngle);
                nodeData[neighbor.x][neighbor.y].oppVelWhenHere = std::pair<float, float>(oppPredictedMoveVelocity, oppPredictedTurnVelocity);
                nodeData[neighbor.x][neighbor.y].orbMoveVelWhenHere = neighborOrbVel;

                // add neighbor to open set so it's explored later
                openSet.emplace_back(neighbor);
            }

            
        }
    }

    std::cout << "no path" << std::endl;
    return {}; // No path found
}