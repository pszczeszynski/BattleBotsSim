#include "FilteredRobot.h"
#include "../MathUtils.h"
#include <iostream>
#include <cstdlib>


// CONSTRUCTOR
FilteredRobot::FilteredRobot() { }
FilteredRobot::FilteredRobot(float pathSpacing, float pathLength, float moveSpeed, float moveAccel, float turnSpeed, float turnAccel) {

    this->pathSpacing = pathSpacing;
    this->pathLength = pathLength;
    this->maxMoveSpeed = moveSpeed;
    this->maxMoveAccel = moveAccel;
    this->maxTurnSpeed = turnSpeed;
    this->maxTurnAccel = turnAccel;

    // visionAngleOffset = 0.0f;

    // initialize
    posFiltered = {0, 0, 0};
    velFiltered = {0, 0, 0};
    accFiltered = {0, 0, 0};

    velFilteredSlow = {0, 0, 0};
}


// hard sets the pos
void FilteredRobot::setPos(std::vector<float> pos) {
    posFiltered = pos;
    velFiltered = {0, 0, 0};
    accFiltered = {0, 0, 0};
}



// incrementally extrapolates with constant accel
std::vector<std::vector<float>> FilteredRobot::constAccExtrap(float time) {

    // start at current values
    std::vector<float> extrapolatedPos = posFiltered;
    std::vector<float> extrapolatedVel = velFiltered;
    std::vector<float> extrapolatedAcc = accFiltered;

    int increments = 50.0f;
    float timeIncrement = time / increments;

    for(int i = 0; i < increments; i++) {

        // save so we can see the increment
        float previousAngle = extrapolatedPos[2];

        // extrapolate the increment at the current angle
        for(int i = 0; i < 3; i++) { extrapolatedPos[i] += (extrapolatedVel[i] * timeIncrement) + 0.5f*(extrapolatedAcc[i] * pow(timeIncrement, 2)); }
        for(int i = 0; i < 3; i++) { extrapolatedVel[i] += (extrapolatedAcc[i] * timeIncrement); }
        extrapolatedPos[2] = angle_wrap(extrapolatedPos[2]);

        // how much the angle incremented
        float angleIncrement = angle_wrap(extrapolatedPos[2] - previousAngle);

        // new directions are incremented by the angle increment
        float newVelDirection = angle_wrap(atan2(extrapolatedVel[1], extrapolatedVel[0]) + angleIncrement);
        float newAccDirection = angle_wrap(atan2(extrapolatedAcc[1], extrapolatedAcc[0]) + angleIncrement);

        float velMag = cv::norm(cv::Point2f(extrapolatedVel[0], extrapolatedVel[1]));
        float accMag = cv::norm(cv::Point2f(extrapolatedAcc[0], extrapolatedAcc[1]));

        // rotate the magnitudes by the new new angle
        extrapolatedVel = {velMag * cos(newVelDirection), velMag * sin(newVelDirection), extrapolatedVel[2]};
        extrapolatedAcc = {accMag * cos(newAccDirection), accMag * sin(newAccDirection), extrapolatedAcc[2]};
    }

    // return final values
    return std::vector<std::vector<float>> {extrapolatedPos, extrapolatedVel, extrapolatedAcc};
}



// incrementally extrapolates with constant vel (0 accel)
std::vector<std::vector<float>> FilteredRobot::constVelExtrap(float time) {

    // start at current values
    std::vector<float> extrapolatedPos = posFiltered;
    std::vector<float> extrapolatedVel = velFiltered;

    int increments = 50.0f;
    float timeIncrement = time / increments;

    for(int i = 0; i < increments; i++) {

        // save so we can see the increment
        float previousAngle = extrapolatedPos[2];

        // extrapolate the increment at the current angle
        for(int i = 0; i < 3; i++) { extrapolatedPos[i] += extrapolatedVel[i] * timeIncrement; }
        extrapolatedPos[2] = angle_wrap(extrapolatedPos[2]);

        // how much the angle incremented
        float angleIncrement = angle_wrap(extrapolatedPos[2] - previousAngle);

        // new directions are incremented by the angle increment
        float newVelDirection = angle_wrap(atan2(extrapolatedVel[1], extrapolatedVel[0]) + angleIncrement);
        float velMag = cv::norm(cv::Point2f(extrapolatedVel[0], extrapolatedVel[1]));

        // rotate the magnitudes by the new new angle
        extrapolatedVel = {velMag * cos(newVelDirection), velMag * sin(newVelDirection), extrapolatedVel[2]};
    }

    // return final values
    return std::vector<std::vector<float>> {extrapolatedPos, extrapolatedVel};
}



// updates the position filters
void FilteredRobot::updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta) {

    if(visionPos.x < fieldMin || visionPos.x > fieldMax || std::isnan(visionPos.x)) { return; }
    if(visionPos.y < fieldMin || visionPos.y > fieldMax || std::isnan(visionPos.y)) { return; }


    // clip to camera frame
    // visionPos = clipInBounds(visionPos);

    // extrapolate each state for new beliefs
    std::vector<std::vector<float>> outputs = constAccExtrap(deltaTime);
    std::vector<float> posBelief = outputs[0];
    std::vector<float> velBelief = outputs[1];
    std::vector<float> accBelief = outputs[2];

    // for(int i = 0; i < 3; i++) { posBelief[i] = posFiltered[i] + (velFiltered[i] * deltaTime) + 0.5f*(accFiltered[i] * pow(deltaTime, 2)); }
    // for(int i = 0; i < 3; i++) { velBelief[i] = velFiltered[i] + (accFiltered[i] * deltaTime); }
    // accBelief = accFiltered;

 
    // from measurement, find deltas to pos
    std::vector<float> posDelta = {visionPos.x - posBelief[0], visionPos.y - posBelief[1], angle_wrap(visionTheta - posBelief[2])};



    // what even are covariances, determines percentage of delta to use
    float positionKalmanGain = std::clamp(deltaTime * 15.0f, 0.05f, 1.0f);
    float turnKalmanGain = std::clamp(deltaTime * 40.0f, 0.05f, 1.0f);
    std::vector<float> kalmanGain = {positionKalmanGain, positionKalmanGain, turnKalmanGain};


    std::vector<float> posFilteredNew = {0, 0, 0};
    std::vector<float> velFilteredNew = {0, 0, 0};
    std::vector<float> accFilteredNew = {0, 0, 0};

    float maxVel = 1000.0f;
    float maxAccel = 100000.0f;

    for(int i = 0; i < 3; i++) { 
        // update pos
        posFilteredNew[i] = posBelief[i] + (kalmanGain[i] * posDelta[i]); 
        if(i == 0 || i == 1) { posFilteredNew[i] = std::clamp(posFilteredNew[i], fieldMin, fieldMax); }
        if(i == 2) { posFilteredNew[i] = angle_wrap(posFilteredNew[i]); }

        // update vel
        velFilteredNew[i] = (posFilteredNew[i] - posFiltered[i]) / deltaTime; if(i == 2) { velFilteredNew[i] = angle_wrap(posFilteredNew[i] - posFiltered[i]) / deltaTime; }
        velFilteredNew[i] = velFiltered[i] + (velFilteredNew[i] - velFiltered[i]) * (1.0f - (pow(1.0f - kalmanGain[i], 2)));
        velFilteredNew[i] = std::clamp(velFilteredNew[i], -maxVel, maxVel);

        // update acc
        accFilteredNew[i] = (velFilteredNew[i] - velFiltered[i]) / deltaTime;
        accFilteredNew[i] = accFiltered[i] + (accFilteredNew[i] - accFiltered[i]) * (1.0f - (pow(1.0f - kalmanGain[i], 3)));
        accFilteredNew[i] = std::clamp(accFilteredNew[i], -maxAccel, maxAccel);
    }
    
    // write new data
    posFiltered = posFilteredNew;
    velFiltered = velFilteredNew;
    accFiltered = accFilteredNew;

    // update slow filters
    for(int i = 0; i < 3; i++) { velFilteredSlow[i] += std::clamp(deltaTime * 5.0f, 0.0f, 0.2f) * (velFiltered[i] - velFilteredSlow[i]); }
}


// how long to collide with a robot using current velocity and assumed turn speed
float FilteredRobot::collideETA(FilteredRobot& opp, bool forward, float collisionRadius) {
    // how much orb velocity towards the opp do we actually count
    float angleThresh1 = 50.0f*TO_RAD; // 60.0f
    float angleThresh2 = 100.0f*TO_RAD; // 160.0f

    float velAngle = atan2(velFiltered[1], velFiltered[0]); // angle we're actually moving in
    float angleToOppAbs = atan2(opp.position().y - posFiltered[1], opp.position().x - posFiltered[0]);
    float angleToOppVel = angle_wrap(angleToOppAbs - velAngle);
    if(cv::norm(moveVel()) == 0.0f) { angleToOppVel = 0.0f; }
    float angleToOpp = angleToOppVel;

    // how good can our velocity be used to drive directly to the opponent if we turn directly towards them
    float velFactor = 1.0f - std::clamp((abs(angleToOpp) - angleThresh1) / (angleThresh2 - angleThresh1), 0.0f, 1.0f);    
    int velSign = sign(tangentVel(forward)); // measure direction of main drive velocity
    float orbUsableSpeed = cv::norm(moveVel()) * velSign * velFactor;

    // estimate time to turn to opp
    float turnTime = turnTimeMin(opp.position(), 0.0f, 30.0f*TO_RAD, forward, false);

    // estimate time to drive to opp
    float distanceToCollision = std::max(cv::norm(position() - opp.position()) - collisionRadius, 0.0);
    float moveTime = moveETASim(distanceToCollision, orbUsableSpeed, false);

    // total time is sum of turning and driving
    return moveTime + turnTime;
}


// signed velocity that's actually in the driving direction, positive means it's in the direction of orientation
float FilteredRobot::tangentVel(bool forward) {
    float velMag = cv::norm(moveVel());
    float velAngle = atan2(velFiltered[1], velFiltered[0]);
    float velAngleOffset = angle_wrap(velAngle - posFiltered[2]);
    int sign = 1; if(!forward) { sign = -1; }
    return velMag * cos(velAngleOffset) * sign;
}



// simulates to find the estimated drive time considering max speed and accel
// simulates to find estimated point time
float FilteredRobot::moveETASim(float distance, float startVel, bool print) {
    constexpr uint64_t kMaxIterations = 3000;

    float timeIncrement = 0.001f;
    float timeSim = 0.0f;

    float velSim = startVel;
    float distanceTraveled = 0.0f;

    // until we've driven the full distance
    uint64_t i = 0;
    for (i = 0; i < kMaxIterations; i++) {
        if(distanceTraveled >= distance) { break; } // if we've traveled far enough, break  
        
        velSim += maxMoveAccel * timeIncrement; // increment velocity by accel
        velSim = std::min(maxMoveSpeed, velSim); // clip velocity to the max value
        distanceTraveled += velSim * timeIncrement; // increment distance traveled by speed

        timeSim += timeIncrement; // increment time
    }

    // make sure we didn't reach the max iterations
    if (i == kMaxIterations) {
        std::cout << "Warning: moveETASim reached max iterations" << std::endl;
    }

    return timeSim; // return the final time
}




// simulates to find estimated point time
float FilteredRobot::pointETASim(cv::Point2f point, float lagTime, float turnCW, float angleMargin, bool forward, bool print) {
    constexpr uint64_t kMaxIterations = 3000;
    float timeIncrement = 0.001f;
    float timeSim = 0.0f;

    std::vector<float> posSim = posFiltered;
    std::vector<float> velSim = velFiltered;
    std::vector<float> accSim = accFiltered;

    float targetAngle = atan2(point.y - posSim[1], point.x - posSim[0]); // angle we're trying to point to
    if(!forward) { targetAngle = angle_wrap(targetAngle + M_PI); }
    if(!turnCW) { angleMargin *= -1; }
    int previousAngleSign = sign(angle_wrap(targetAngle - posSim[2] - angleMargin));


    // until we're pointing
    uint64_t i = 0;
    for (i = 0; i < kMaxIterations; i++) {
        float angleDistance = angle_wrap(targetAngle - posSim[2]);
        int angleSign = sign(angle_wrap(targetAngle - posSim[2] - angleMargin));
        if ((abs(angleDistance) < abs(angleMargin)) ||
            (previousAngleSign == 1 && angleSign == -1 && turnCW) ||
            (previousAngleSign == -1 && angleSign == 1 && !turnCW)) {
          break;
        }  // if we're pointed already, break the loop
        previousAngleSign = angleSign;

        int signCW = 1; if(turnCW) { signCW = -1; } // invert accel direction based on desired turn direction
        accSim[2] = signCW*0.2f*maxTurnAccel; // be default, assume they're acceling slowly 
        if(timeSim > lagTime) { accSim[2] = maxTurnAccel*signCW; } // if we're past the lag time, the human inputs correctly

        posSim[2] = angle_wrap(posSim[2] + velSim[2]*timeIncrement); // increment position
        velSim[2] += accSim[2]; // increment velocity if we're past the lag time
        velSim[2] = std::clamp(velSim[2], maxTurnSpeed, -maxTurnSpeed);

        timeSim += timeIncrement;
    }

    if (i == kMaxIterations) {
        std::cout << "Warning: pointETASim reached max iterations" << std::endl;
    }

    return timeSim; // return the final time
}

// takes the minimum of both turn directions
float FilteredRobot::turnTimeMin(cv::Point2f point, float lagTime,
                                 float angleMargin, bool forward, bool print) {
  return std::min(pointETASim(point, 0.0f, true, angleMargin, forward, print),
                  pointETASim(point, 0.0f, false, angleMargin, forward, print));
}

// returns the sign
int FilteredRobot::sign(float num) {
    int returnSign = 1;
    if(num < 0) { returnSign = -1; }
    return returnSign;
}


// angle to the inputted point from the front of the robot
float FilteredRobot::angleTo(cv::Point2f point, bool forward) {
    float rawAngle = atan2(point.y - posFiltered[1], point.x - posFiltered[0]);
    float offset = 0.0f; if(!forward) { offset = M_PI; } // turn by 180 if its backwards
    return angle_wrap(rawAngle - posFiltered[2] + offset);
}


// distance to the inputted point
float FilteredRobot::distanceTo(cv::Point2f point) {
    return cv::norm(cv::Point2f(posFiltered[0], posFiltered[1]) - point);
}



// updates the path tracking vector
void FilteredRobot::updatePath() {
  int pathSize = round(pathLength / pathSpacing);

  cv::Point2f position(posFiltered[0], posFiltered[1]);
  if (path.size() == 0) {
    path.emplace_back(position);
  }

  // if you've moved far from the last point, add a new point
  if (cv::norm(position - path[path.size() - 1]) > pathSpacing) {
    if (path.size() < pathSize) {
      path.emplace_back(position);
    }
    else {
      // shift all the points down
      for (int i = 0; i < path.size() - 1; i++) {
        path[i] = path[i + 1];
      }
      // override the last with us
      path[path.size() - 1] = position;
    }
  }
}

// // runs additional path tangency stuff for opp
// void FilteredRobot::updateFiltersOpp(float deltaTime, cv::Point2f visionPos, float visionTheta) {

//     // add our increment
//     pathTangency(deltaTime, visionTheta);
//     float actualThetaMeasured = angle_wrap(visionTheta + visionAngleOffset);

//     updateFilters(deltaTime, visionPos, actualThetaMeasured);
// }


// // averages in path tangency
// void FilteredRobot::pathTangency(float deltaTime, float visionTheta) {

//     // what direction are we driving
//     float velAngle = atan2(velFilteredSlow[1], velFilteredSlow[0]);
//     float rawOffset = angle_wrap(velAngle - visionTheta);


//     // how much we factor in the path tangency
//     float velMag = cv::norm(moveVel());
//     float factor = std::clamp((velMag / std::max(abs(velFilteredSlow[2]), 0.01f) - 250.0f) * deltaTime * 0.0001f, 0.0f, 0.1f); // 0.03


//     // weighted average in the vision angle, clips to nearest 180deg in case they drive backwards
//     visionAngleOffset = angle_wrap(visionAngleOffset + factor * 0.5f * angle_wrap(2.0f * (rawOffset - visionAngleOffset)));
//     // posFiltered[2] = angle_wrap(visionTheta + visionAngleOffset);
// }



// get robot data
cv::Point2f FilteredRobot::position() { return cv::Point2f(posFiltered[0], posFiltered[1]); }
cv::Point2f FilteredRobot::moveVel() { return cv::Point2f(velFiltered[0], velFiltered[1]); }

float FilteredRobot::angle(bool forward) { 
    int offset = 0.0f; if(!forward) { offset = M_PI; } // offset by 180 if not forwards
    return angle_wrap(posFiltered[2] + offset); 
}

float FilteredRobot::turnVel() { return velFiltered[2]; }
float FilteredRobot::getMaxTurnSpeed() { return maxTurnSpeed; }
float FilteredRobot::moveAccel() { return cv::norm(cv::Point2f(accFiltered[0], accFiltered[1])); }
float FilteredRobot::turnAccel() { return accFiltered[2]; }
std::vector<cv::Point2f> FilteredRobot::getPath() { return path; }




// clips the inputted point to the field bounds
cv::Point2f FilteredRobot::clipInBounds(cv::Point2f point) {
    float resultX = std::clamp(point.x, fieldMin, fieldMax);
    float resultY = std::clamp(point.y, fieldMin, fieldMax);
    
    return cv::Point2f(resultX, resultY);
}


std::vector<float> FilteredRobot::getPosFiltered() {
    return posFiltered;
}
std::vector<float> FilteredRobot::getVelFiltered() {
    return velFiltered;
}
std::vector<float> FilteredRobot::getAccFiltered() {
    return accFiltered;
}