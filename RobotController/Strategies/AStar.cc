#include "AStar.h"
#include <limits>
#include "../Clock.h"

#define M_PI 3.14159

AStar::AStar(int width_, int height_)
{

    width = width_;
    height = height_; 

    float weaponWidth = 0.12f * width;
    float weaponHeight = 0.05f * width;
    float weaponForwardOffset = 0.07f * width;

    std::vector<Line> weaponLines = {
        Line(cv::Point2f(-weaponWidth/2, weaponForwardOffset), cv::Point2f(weaponWidth/2, weaponForwardOffset)), // bottom
        Line(cv::Point2f(-weaponWidth/2, weaponForwardOffset), cv::Point2f(0.0f, weaponForwardOffset + weaponHeight)), // left diagonal
        Line(cv::Point2f(weaponWidth/2, weaponForwardOffset), cv::Point2f(0.0f, weaponForwardOffset + weaponHeight)) // right diagonal
    };

    oppWeapon = ScoreRegion(weaponLines);
    

    // build the vectors
    for (int y = 0; y < height; ++y) {
        gScore.push_back(std::vector<float>());
        fScore.push_back(std::vector<float>());
        cameFrom.push_back(std::vector<cv::Point>());
        oppPosWhenHere.push_back(std::vector<std::pair<cv::Point2f, float>>());
        oppVelWhenHere.push_back(std::vector<std::pair<float, float>>());
        orbVelWhenHere.push_back(std::vector<float>());

        // fill the arrays with default values
        for (int x = 0; x < width; ++x) {
            float score = std::numeric_limits<double>::infinity();
            gScore[y].push_back(score);
            fScore[y].push_back(score);

            // fill in came from list with arbitrary points, value doesn't matter
            cv::Point silly(-1, -1);
            cameFrom[y].push_back(silly);

            // fill in opponent pos list with arbitrary values, values don't matter
            std::pair<cv::Point2f, float> silly2(cv::Point2f(-1, -1), 0.0f);
            oppPosWhenHere[y].push_back(silly2);

            // fill in opponent vel list with arbitrary values, values don't matter
            std::pair<float, float> silly3(0.0f, 0.0f);
            oppVelWhenHere[y].push_back(silly3);
            orbVelWhenHere[y].push_back(0.0f);
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
    oppAngle = oppAngle_;
    oppMoveVelocity = oppMoveVelocity_;
    oppTurnVelocity = oppTurnVelocity_;

    
    // init all locations with gScore and fScore of infinity
    // init all locations with came from something just for memes
    // init oppPositions to random stuff to avoid confusion
    for (int y = 0; y < height; ++y) {

        // fill the arrays with default values
        for (int x = 0; x < width; ++x) {
            float score = std::numeric_limits<double>::infinity();
            gScore[x][y] = score;
            fScore[x][y] = score;
        }
    }

    // cost to get to start node is nothing
    gScore[start.x][start.y] = 0;

    // therefore, total score of start is just heuristic
    fScore[start.x][start.y] = heuristic(start, floatPointToInt(oppPositionF));


    // opp positions are what they currently are at start node
    oppPosWhenHere[start.x][start.y] = std::pair<cv::Point2f, float>(oppPositionF, oppAngle);
    oppVelWhenHere[start.x][start.y] = std::pair<float, float>(oppMoveVelocity, oppTurnVelocity);
    orbVelWhenHere[start.x][start.y] = orbMoveVelocity;

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
void AStar::setWeaponLines(const cv::Point2f& center, const float& angle) {
    oppWeaponLines = oppWeapon.fieldLines(center, angle);
    //std::cout << "center = " << center << std::endl;
}

// fills the boundary list with the inputted list
void AStar::setBoundaryLines(const std::vector<Line>& boundaryLineList) {
    boundaryLines = boundaryLineList;
}

float AStar::heuristic(const cv::Point& current, const cv::Point& goal) {
    //return 0.0f;
    return cv::norm(current - goal) / 1.0;
}

float AStar::wrapAngle(const float& angle) {
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

// does a point get too close to a line segment connecting two nodes
bool AStar::doesIntersectPoint(const cv::Point& begin, const cv::Point& endIn, const cv::Point2f& point, float howClose, const bool& ignoreStart) {

    bool firstOfPath = (begin == start);    
    if (howClose >= 1.0f) {
        // exception for the final opp point, just needs to be 1 tile away, another exception for start position
        cv::Point2f oppPos = oppPosWhenHere[begin.x][begin.y].first;
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
    float distanceToGoal = cv::norm(floatPointToInt(oppPosWhenHere[point.x][point.y].first) - point);

    // // Determine outer and inner radii
    // int outerRad = round(distanceToGoal * 0.6);
    // outerRad = min(max(outerRad, 2), 12); // minimum of 2, max of 6
    // int innerRad = max(outerRad - 2, 0); // minimum of 0

    int outerRad = 5;
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

// returns boundary points
std::vector<Line> AStar::getBoundaryLines() {
    return boundaryLines;
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
    angle += dTheta;
}


// predicts time required for a neighbor node, rough approximation
float AStar::predictNeighborTime(const cv::Point& current, const cv::Point& neighbor, float& vel, const float& currentAngle) {

    float angleToNeighbor = angle(current, neighbor);
    float turnAngle = abs(wrapAngle(angleToNeighbor - currentAngle));
    float neighborDistance = cv::norm(neighbor - current);

    float time = (neighborDistance / vel) * (1 + 10*std::min(pow(turnAngle/M_PI, 4), 1.0));

    vel = neighborDistance / time; // update the velocity with an estimated number

    return time;
}


// updates opponent position assuming constant decel parameters by reference
void AStar::extrapolateOpp(cv::Point2f& position, float& angle, float& moveVelocity, float& turnVelocity, float time) {

    int timeSteps = 10;
    float oppMoveAccel = 25; // tiles/sec^2
    float oppTurnAccel = 100; // rad/sec^2
    float pointAndShootVel = 1; // rad/sec

    cv::Point2f previousPosition;
    float previousAngle;
    float previousMoveVel;
    float previousTurnVel;

    // simulate through every timestep
    for (int i = 0; i < timeSteps; i++) {

        // // if we're moving super slow already then just return
        // if (moveVelocity < 0.05f && turnVelocity < 0.05f) {
        //     return;
        // }

        // save in case a collision is found
        previousPosition = position;
        previousAngle = angle;
        previousMoveVel = moveVelocity;
        previousTurnVel = turnVelocity;

        // extrapolate in an arc with current velocities
        constantVelExtrapolate(position, angle, moveVelocity, turnVelocity, time/timeSteps);

        int moveVelSign = 1;
        if (moveVelocity < 0) { moveVelSign = -1; }
        int turnVelSign = 1;
        if (turnVelocity < 0) { turnVelSign = -1; }

        // decrease move/turn vel magnitudes as a predicted decay
        moveVelocity = moveVelSign * std::max(abs(moveVelocity) - oppMoveAccel*(time/timeSteps), 0.0f);
        turnVelocity = turnVelSign * std::max(abs(turnVelocity) - oppTurnAccel*(time/timeSteps), 0.0f);

        // see where the weapon would be at this position
        std::vector<Line> potentialWeapon = oppWeapon.fieldLines(position, angle);
        
        float closestPotentialWeapon = width + height; // will always be higher
        for (int i = 0; i < potentialWeapon.size(); i++) {
            cv::Point2f startF(start.x, start.y);
            float distanceToStart = potentialWeapon[i].howClosePoint(startF);
            // if a weapon point is too close then stop extrapolating
            if (distanceToStart < 0.06f * width) {
                position = previousPosition;
                angle = previousAngle;
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

        // if we've found the goal then the path is done
        if (current == floatPointToInt(oppPosWhenHere[current.x][current.y].first)) { 
            return reconstructPath(current);
        }

        setWeaponLines(oppPosWhenHere[current.x][current.y].first, oppPosWhenHere[current.x][current.y].second);

        // add current node to closed set to mark it as explored
        closedSet.emplace_back(current);

        for (const auto& neighbor : getNeighbors(current)) {

            // if the neighbor is in the closed set already, skip this iteration
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) { 
                continue;
            }

            float neighborDistance = cv::norm(current - neighbor);
                        
            float closestWeapon = closestFromLineList(oppWeaponLines, neighbor);


            float angleChange = 0;
            float previousAngle = 0; // use the robot's angle as a default
            float angleWeight = 0;
            float neighborAngle = angle(current, neighbor);
            if (current == start) {
                previousAngle = robotAngle;
                angleChange = abs(wrapAngle(neighborAngle - previousAngle));
                //angleWeight = 1.0 * angleChange;
            }
            else { // if it's not the start node, look back for previous angle
                previousAngle = angle(cameFrom[current.x][current.y], current); // angle of previous line
                angleChange = abs(wrapAngle(neighborAngle - previousAngle));

                //angleWeight = 0;                
            }

            // if (angleChange > 95 * (M_PI / 180.0)) {
            //     angleWeight = 10 * pow(angleChange, 2);
            // } 

            angleWeight = 30.0f * angleChange * neighborDistance;
            //angleWeight = 0.0f;


            // lower score = preferred
            // float tentativeGScore = gScore[current.x][current.y] + 
            // (10 / pow(closestWeapon, 1.2) * pow(neighborDistance, 0.9)) + // opponent weapon weight.+, 1.2, 0.9
            // angleWeight + // angle change weight
            // (0.5 / pow(closestFromList(boundaryPoints, neighbor), 3) * cv::norm(current - neighbor)); // boundaries weight

            float neighborOrbVel = orbVelWhenHere[current.x][current.y];
            float neighborTime = predictNeighborTime(current, neighbor, neighborOrbVel, previousAngle);
            float tentativeGScore = gScore[current.x][current.y] + (0.1 / pow(closestWeapon, 1.0) * pow(neighborDistance, 1.0)) + 0.0*neighborDistance + 0.0*angleWeight;
            //float tentativeGScore = gScore[current.x][current.y] + 0.0*neighborTime + 0.0*neighborDistance + (1/neighborOrbVel)*(neighborDistance)*pow(closestWeapon, -1.0) + angleWeight;
            //float tentativeGScore = gScore[current.x][current.y] + pow(closestWeapon, -1.5)*pow(neighborDistance, 1.0) + 0*neighborTime + 0.0*neighborDistance;


            // if the score is better than commit the node
            if (tentativeGScore < gScore[neighbor.x][neighbor.y]) {

                cameFrom[neighbor.x][neighbor.y] = current;
                gScore[neighbor.x][neighbor.y] = tentativeGScore;
                fScore[neighbor.x][neighbor.y] = tentativeGScore + heuristic(neighbor, floatPointToInt(oppPosWhenHere[current.x][current.y].first));


                // save the predicted positions and velocities at this neighbor
                cv::Point2f oppPredictedPosition = oppPosWhenHere[current.x][current.y].first;
                float oppPredictedAngle = oppPosWhenHere[current.x][current.y].second;
                float oppPredictedMoveVelocity = oppVelWhenHere[current.x][current.y].first;
                float oppPredictedTurnVelocity = oppVelWhenHere[current.x][current.y].second;
                extrapolateOpp(oppPredictedPosition, oppPredictedAngle, oppPredictedMoveVelocity, oppPredictedTurnVelocity, neighborTime);
                oppPosWhenHere[neighbor.x][neighbor.y] = std::pair<cv::Point2f, float>(oppPredictedPosition, oppPredictedAngle);
                oppVelWhenHere[neighbor.x][neighbor.y] = std::pair<float, float>(oppPredictedMoveVelocity, oppPredictedTurnVelocity);
                orbVelWhenHere[neighbor.x][neighbor.y] = neighborOrbVel;

                // add neighbor to open set so it's explored later
                openSet.emplace_back(neighbor);
            }

            
        }
    }

    std::cout << "no path" << std::endl;
    return {}; // No path found
}