#include "FilteredRobot.h"

#define M_PI 3.14159



// CONSTRUCTOR
FilteredRobot::FilteredRobot() { }
FilteredRobot::FilteredRobot(float pathSpacing, float pathLength) {

    this->pathSpacing = pathSpacing;
    this->pathLength = pathLength;
}




// updates the position filters
void FilteredRobot::updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta) {

    // clip to field wtf
    visionPos = clipInBounds(visionPos);


    // find travel direction and magnitude
    float speed = cv::norm(moveVelFiltered);
    float travelDirection = atan2(moveVelFiltered.y, moveVelFiltered.x);

    // update previous belief with pretend odometry
    float newXBelief = posFiltered.x + (moveVelFiltered.x * deltaTime);
    float newYBelief = posFiltered.y + (moveVelFiltered.y * deltaTime);

    // if we're not going in a straight line use the arc equations
    if(angleFiltered != 0.0f) {
        newXBelief = posFiltered.x + (speed / turnVelFiltered) * (sin(travelDirection + turnVelFiltered*deltaTime) - sin(travelDirection));
        newYBelief = posFiltered.y + (-speed / turnVelFiltered) * (cos(travelDirection + turnVelFiltered*deltaTime) - cos(travelDirection));
    }

    float newThetaBelief = angleFiltered + (turnVelFiltered * deltaTime);
    

    // correct using delta to vision position
    float deltaX = visionPos.x - newXBelief;
    float deltaY = visionPos.y - newYBelief;
    float deltaTheta = wrapAngle(visionTheta - newThetaBelief);


    // how much to include the delta
    float visionPosGain = deltaTime * 30.0f; // 40
    float visionThetaGain = deltaTime * 30.0f; // 40


    if(visionPosGain > 1.0f) { visionPosGain = 1.0f; }
    if(visionThetaGain > 1.0f) { visionThetaGain = 1.0f; }


    // add percentages of deltas
    float newX = newXBelief + (visionPosGain * deltaX);
    float newY = newYBelief + (visionPosGain * deltaY);
    cv::Point2f newPos = clipInBounds(cv::Point2f(newX, newY));

    float newTheta = wrapAngle(newThetaBelief + (visionThetaGain * deltaTheta));

    // calculate velocities
    float moveVelXRaw = (newPos.x - posFiltered.x) / deltaTime;
    float moveVelYRaw = (newPos.y - posFiltered.y) / deltaTime;
    float turnVelRaw = wrapAngle(newTheta - angleFiltered) / deltaTime;

    float moveVelGain = deltaTime * 30.0f; if(moveVelGain > 1.0f) { moveVelGain = 1.0f; }
    float turnVelGain = deltaTime * 30.0f; if(turnVelGain > 1.0f) { turnVelGain = 1.0f; }

    moveVelFiltered.x += (moveVelXRaw - moveVelFiltered.x) * moveVelGain;
    moveVelFiltered.y += (moveVelYRaw - moveVelFiltered.y) * moveVelGain;
    turnVelFiltered += (turnVelRaw - turnVelFiltered) * turnVelGain;
    

    // write new positions
    posFiltered = newPos;
    angleFiltered = newTheta;
}


// updates the path tracking vector
void FilteredRobot::updatePath() {

    int pathSize = round(pathLength / pathSpacing);

    if(path.size() == 0) { path.emplace_back(posFiltered); }

    if(cv::norm(posFiltered - path[path.size() - 1]) > pathSpacing) {
        if(path.size() < pathSize) { 
            path.emplace_back(posFiltered);
        }

        else {
            for(int i = 0; i < path.size() - 1; i++) {
                path[i] = path[i + 1];
            }
            path[path.size() - 1] = posFiltered;
        }
    }
}


// get robot data
cv::Point2f FilteredRobot::pos() { return posFiltered; }
cv::Point2f FilteredRobot::moveVel() { return moveVelFiltered; }
float FilteredRobot::angle() { return angleFiltered; }
float FilteredRobot::turnVel() { return turnVelFiltered; }
std::vector<cv::Point2f> FilteredRobot::getPath() { return path; }




// wraps an angle
float FilteredRobot::wrapAngle(float angle) {

    float mod = std::fmod(angle + M_PI, 2 * M_PI);
    if (mod < 0) { mod += 2*M_PI; }
    return mod - M_PI;
}


// clips the inputted point to the inputted bounds
cv::Point2f FilteredRobot::clipInBounds(cv::Point2f point) {

    float resultX = point.x;
    if(resultX < fieldMin) { resultX = fieldMin; }
    else if(resultX > fieldMax) { resultX = fieldMax; }

    float resultY = point.y;
    if(resultY < fieldMin) { resultY = fieldMin; }
    else if(resultY > fieldMax) { resultY = fieldMax; }

    return cv::Point2f(resultX, resultY);
}