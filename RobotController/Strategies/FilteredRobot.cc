#include "FilteredRobot.h"
#include <iostream>
#include <cstdlib>
#include <cmath>

#define M_PI 3.14159



// CONSTRUCTOR
FilteredRobot::FilteredRobot() { }
FilteredRobot::FilteredRobot(float pathSpacing, float pathLength, float turnSpeed, float turnAccel) {

    this->pathSpacing = pathSpacing;
    this->pathLength = pathLength;
    this->maxTurnSpeed = turnSpeed;
    this->maxTurnAccel = turnAccel;

    visionAngleOffset = 0.0f;

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
        extrapolatedPos[2] = wrapAngle(extrapolatedPos[2]);

        // how much the angle incremented
        float angleIncrement = wrapAngle(extrapolatedPos[2] - previousAngle);

        // new directions are incremented by the angle increment
        float newVelDirection = wrapAngle(atan2(extrapolatedVel[1], extrapolatedVel[0]) + angleIncrement);
        float newAccDirection = wrapAngle(atan2(extrapolatedAcc[1], extrapolatedAcc[0]) + angleIncrement);

        float velMag = cv::norm(cv::Point2f(extrapolatedVel[0], extrapolatedVel[1]));
        float accMag = cv::norm(cv::Point2f(extrapolatedAcc[0], extrapolatedAcc[1]));

        // rotate the magnitudes by the new new angle
        extrapolatedVel = {velMag * cos(newVelDirection), velMag * sin(newVelDirection), extrapolatedVel[2]};
        extrapolatedAcc = {accMag * cos(newAccDirection), accMag * sin(newAccDirection), extrapolatedAcc[2]};
    }

    // return final values
    return std::vector<std::vector<float>> {extrapolatedPos, extrapolatedVel, extrapolatedAcc};
}



// incrementally extrapolates with constant vel
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
        extrapolatedPos[2] = wrapAngle(extrapolatedPos[2]);

        // how much the angle incremented
        float angleIncrement = wrapAngle(extrapolatedPos[2] - previousAngle);

        // new directions are incremented by the angle increment
        float newVelDirection = wrapAngle(atan2(extrapolatedVel[1], extrapolatedVel[0]) + angleIncrement);
        float velMag = cv::norm(cv::Point2f(extrapolatedVel[0], extrapolatedVel[1]));

        // rotate the magnitudes by the new new angle
        extrapolatedVel = {velMag * cos(newVelDirection), velMag * sin(newVelDirection), extrapolatedVel[2]};
    }

    // return final values
    return std::vector<std::vector<float>> {extrapolatedPos, extrapolatedVel};
}



// updates the position filters
void FilteredRobot::updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta) {

    // clip to camera frame
    visionPos = clipInBounds(visionPos);

    // epic noise
    float randomValue1 = static_cast<float>(rand()) / RAND_MAX;
    float randomValue2 = static_cast<float>(rand()) / RAND_MAX;
    // visionPos.x += 7.0f * 2.0f * (randomValue1 - 0.5f);
    // visionPos.y += 7.0f * 2.0f * (randomValue2 - 0.5f);


    // extrapolate each state for new beliefs
    std::vector<std::vector<float>> outputs = constAccExtrap(deltaTime);
    std::vector<float> posBelief = outputs[0];
    std::vector<float> velBelief = outputs[1];
    std::vector<float> accBelief = outputs[2];

    for(int i = 0; i < 3; i++) { posBelief[i] = posFiltered[i] + (velFiltered[i] * deltaTime) + 0.5f*(accFiltered[i] * pow(deltaTime, 2)); }
    for(int i = 0; i < 3; i++) { velBelief[i] = velFiltered[i] + (accFiltered[i] * deltaTime); }
    accBelief = accFiltered;

 

    // from measurement, find deltas to pos
    std::vector<float> posDelta = {visionPos.x - posBelief[0], visionPos.y - posBelief[1], wrapAngle(visionTheta - posBelief[2])};

    // std::cout << "     delta = " << posDelta[2] << "     ";
    // std::cout << "     vision theta = " << visionTheta << "     ";

    // what even are covariances, determines percentage of delta to use
    float positionKalmanGain = std::clamp(deltaTime * 15.0f, 0.05f, 1.0f);
    float turnKalmanGain = std::clamp(deltaTime * 40.0f, 0.05f, 1.0f);
    std::vector<float> kalmanGain = {positionKalmanGain, positionKalmanGain, turnKalmanGain};

    // std::cout << "    gain = " << positionKalmanGain << ",     ";



    std::vector<float> posFilteredNew = {0, 0, 0};
    std::vector<float> velFilteredNew = {0, 0, 0};
    std::vector<float> accFilteredNew = {0, 0, 0};

    float maxVel = 1000.0f;
    float maxAccel = 1000000000.0f;

    for(int i = 0; i < 3; i++) { 
        // update pos
        posFilteredNew[i] = posBelief[i] + (kalmanGain[i] * posDelta[i]); 
        if(i == 2) { posFilteredNew[i] = wrapAngle(posFilteredNew[i]); }

        // update vel
        velFilteredNew[i] = (posFilteredNew[i] - posFiltered[i]) / deltaTime; if(i == 2) { velFilteredNew[i] = wrapAngle(posFilteredNew[i] - posFiltered[i]) / deltaTime; }
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
float FilteredRobot::collideETA(FilteredRobot opp) {

    float TO_RAD = 3.14159f / 180.0f;

    float collisionRadius = 70.0f; // radius at which the robots basically collide
    float distanceToCollision = std::max(cv::norm(position() - opp.position()) - collisionRadius, 0.0);

   
    // how much orb velocity towards the opp do we actually count
    float angleThresh1 = 70.0f*TO_RAD;
    float angleThresh2 = 160.0f*TO_RAD;
    float velFactor = 1.0f - std::clamp((abs(angleTo(opp.position())) - angleThresh1) / (angleThresh2 - angleThresh1), 0.0f, 1.0f);

    
    float assumedMinSpeed = 100.0f; // 200
    float orbUsableSpeed = std::max(cv::norm(moveVel()), (double) assumedMinSpeed) * velFactor;
    float oppSpeed = cv::norm(opp.moveVel());
    float oppSpeedAngle = atan2(opp.moveVel().y, opp.moveVel().x);
    float angleToOrbAbs = atan2(position().y - opp.position().y, position().x - opp.position().x);
    float oppSpeedTowards = oppSpeed * cos(angleToOrbAbs - oppSpeedAngle);
    // float collisionSpeed = oppSpeedTowards + orbUsableSpeed;
    float collisionSpeed = 2.0f*std::min(oppSpeedTowards, 0.0f) + orbUsableSpeed;


    float angleMargin = 10.0f*TO_RAD;
    // float turnTime = std::max(abs(angleTo(opp.position())) - 30.0f*TO_RAD, 0.0f) / maxTurnSpeed;
    // float turnTime = pointETA(opp.position(), 0.0f, angleMargin, angleMargin, true);
    // float turnTimeCCW = pointETA(opp.position(), 0.0f, angleMargin, angleMargin, false);
    // if(turnTimeCCW < turnTime) { turnTime = turnTimeCCW; }
    float turnTime = std::min(pointETASim(opp.position(), 0.0f, true, angleMargin), pointETASim(opp.position(), 0.0f, false, angleMargin));

    float travelTime = distanceToCollision / std::max(collisionSpeed, 100.0f);


    return travelTime + turnTime;
}



// how long to turn towards the point using the assumed speed
// angleMargin1 is the margin in the direction of orbit rotation, so it should be smaller than angleMargin2
float FilteredRobot::pointETA(cv::Point2f point, float lagTime, float angleMargin1, float angleMargin2, bool CW) {

    std::vector<std::vector<float>> extrapolated = constAccExtrap(lagTime); // extrapolate robot based on lag time
    std::vector<float> posExtrap = extrapolated[0];
    std::vector<float> velExtrap = extrapolated[1];


    float TO_RAD = 3.14159f / 180.0f;
    float angle = atan2(point.y - posExtrap[1], point.x - posExtrap[0]); // absolute angle the opp has to face to face us directly

    float averageAngleMargin = 0.5f * (angleMargin1 + angleMargin2);

    if(!CW) {
        angleMargin1 *= -1.0f;
        angleMargin2 *= -1.0f;
    }

    float angleOffset = 0.5f * (angleMargin1 - angleMargin2);


    // cutoff angles for where we don't want to hit
    float angle1 = wrapAngle(angle - angleMargin1);
    float angle2 = wrapAngle(angle + angleMargin2);


    // how far we have to turn to the point exactly
    float turnDistance = wrapAngle(angle - (posExtrap[2] + angleOffset)); 
    if(abs(turnDistance) < averageAngleMargin) { return 0; } // if opp is already facing us within the margin


    // otherwise return the shortest time required to get us into the angle margin
    float time = lagTime + std::min(timeToTurnToAngle(posExtrap, velExtrap, angle1, true), std::min(timeToTurnToAngle(posExtrap, velExtrap, angle1, false), std::min(timeToTurnToAngle(posExtrap, velExtrap, angle2, true), timeToTurnToAngle(posExtrap, velExtrap, angle2, false))));
    if(time == lagTime) { time = 0.0f; }
    return time;
}


// simulates to find estimated point time
float FilteredRobot::pointETASim(cv::Point2f point, float lagTime, float turnCW, float angleMargin) {

    float timeIncrement = 0.01f;
    float timeSim = 0.0f;

    std::vector<float> posSim = posFiltered;
    std::vector<float> velSim = velFiltered;
    std::vector<float> accSim = accFiltered;

    float targetAngle = atan2(point.y - posSim[1], point.x - posSim[0]); // angle we're trying to point to
    if(!turnCW) { angleMargin *= -1; }
    int previousAngleSign = sign(wrapAngle(targetAngle - posSim[2] - angleMargin));

    // if(angleMargin == 0) { std::cout << "angle sign = " << previousAngleSign << std::endl; }


    // until we're pointing
    while(true) {
        float angleDistance = wrapAngle(targetAngle - posSim[2]);
        int angleSign = sign(wrapAngle(targetAngle - posSim[2] - angleMargin));
        if((abs(angleDistance) < abs(angleMargin)) || (previousAngleSign == 1 && angleSign == -1 && turnCW) || (previousAngleSign == -1 && angleSign == 1 && !turnCW)) { break; } // if we're pointed already, break the loop
        // if(abs(angleDistance) < angleMargin) { break; } // if we're pointed already, break the loop       
        previousAngleSign = angleSign;

        posSim[2] = wrapAngle(posSim[2] + velSim[2]*timeIncrement); // increment position
        velSim[2] += accSim[2]; // increment velocity if we're past the lag time

        int signCW = 1; if(turnCW) { signCW = -1; } // invert accel direction based on desired turn direction

        accSim[2] = accFiltered[2] + signCW*0.3f*maxTurnAccel; // be default, assume they're acceling slowly 
        if(timeSim > lagTime) { accSim[2] = maxTurnAccel*signCW; } // if we're past the lag time, the human inputs correctly

        velSim[2] = std::clamp(velSim[2], -maxTurnSpeed, maxTurnSpeed);
        
        timeSim += timeIncrement;
    }

    return timeSim; // return the final time
}


// returns the sign
int FilteredRobot::sign(float num) {
    int returnSign = 1;
    if(num < 0) { returnSign = -1; }
    return returnSign;
}


// how long to turn to an angle assuming you're gonna go in CW or CCW as inputted
// assumes you're gonna accelerate up to the known turn speed, including if you have to reverse
float FilteredRobot::timeToTurnToAngle(std::vector<float> pos, std::vector<float> vel, float angle, bool turnCW) {
    float TO_RAD = 3.14159f / 180.0f;

    float angleDistance = wrapAngle(angle - pos[2]); // how far we have to turn
    if(!turnCW) { angleDistance *= -1.0f; }
    if(angleDistance < 0) { angleDistance = 360.0f*TO_RAD + angleDistance; }

    // std::cout << " angle distance = " << angleDistance << "    ";

    float currentTurnVel = vel[2]; if(!turnCW) { currentTurnVel *= -1.0f; } // reverse for positive direction problem

    float constAccelTurnTime = (-currentTurnVel + sqrt(pow(currentTurnVel, 2) - (2 * maxTurnAccel * -angleDistance))) / maxTurnAccel;
    float constAccelTurnTime2 = (-currentTurnVel - sqrt(pow(currentTurnVel, 2) - (2 * maxTurnAccel * -angleDistance))) / maxTurnAccel;

    // use the lowest positive time
    if(constAccelTurnTime > 0 && constAccelTurnTime2 > 0 && constAccelTurnTime2 < constAccelTurnTime) { constAccelTurnTime = constAccelTurnTime2; }
    else if(constAccelTurnTime < 0) { constAccelTurnTime = constAccelTurnTime2; }

    // std::cout << "      constAccelTurnTime = " << constAccelTurnTime << "    ";

    float timeToFullSpeed = std::max((maxTurnSpeed - currentTurnVel) / maxTurnAccel, 0.0f);
    if(timeToFullSpeed < constAccelTurnTime) { constAccelTurnTime = timeToFullSpeed; }

    float distanceCoveredByAccel = (0.5f * maxTurnAccel * pow(constAccelTurnTime, 2)) + (currentTurnVel * constAccelTurnTime);

    // std::cout << "      time to full speed = " << timeToFullSpeed << "    ";
    // std::cout << "      distancecoveredbyaccel = " << distanceCoveredByAccel << "    ";

    // any remaining distance is assumed at top speed
    angleDistance -= distanceCoveredByAccel;
    float timeAtTopSpeed = abs(angleDistance) / maxTurnSpeed;

    // std::cout << "   time at top speed = " << timeAtTopSpeed << ", constAccelTurnTime = " << constAccelTurnTime;

    return constAccelTurnTime + timeAtTopSpeed;
}


// angle to the inputted point from the front of the robot
float FilteredRobot::angleTo(cv::Point2f point) {
    float rawAngle = atan2(point.y - posFiltered[1], point.x - posFiltered[0]);
    return wrapAngle(rawAngle - posFiltered[2]);
}



// updates the path tracking vector
void FilteredRobot::updatePath() {

    int pathSize = round(pathLength / pathSpacing);

    cv::Point2f position(posFiltered[0], posFiltered[1]);
    if(path.size() == 0) { path.emplace_back(position); }

    if(cv::norm(position - path[path.size() - 1]) > pathSpacing) {
        if(path.size() < pathSize) { 
            path.emplace_back(position);
        }

        else {
            for(int i = 0; i < path.size() - 1; i++) {
                path[i] = path[i + 1];
            }
            path[path.size() - 1] = position;
        }
    }
}


// runs additional path tangency stuff for opp
void FilteredRobot::updateFiltersOpp(float deltaTime, cv::Point2f visionPos, float visionTheta) {

    // add our increment
    pathTangency(deltaTime, visionTheta);
    float actualThetaMeasured = wrapAngle(visionTheta + visionAngleOffset);

    updateFilters(deltaTime, visionPos, actualThetaMeasured);
}


// averages in path tangency
void FilteredRobot::pathTangency(float deltaTime, float visionTheta) {

    // what direction are we driving
    float velAngle = atan2(velFilteredSlow[1], velFilteredSlow[0]);
    // float rawOffset = 0.5f * wrapAngle(2 * (velAngle - visionTheta));
    float rawOffset = wrapAngle(velAngle - visionTheta);


    // how much we factor in the path tangency
    float velMag = cv::norm(cv::Point2f(velFilteredSlow[0], velFilteredSlow[1]));
    // float factor = std::clamp((velMag - 110.0f) * 0.002f, 0.0f, 1.0f);
    float factor = std::clamp((std::max(velMag - 50.0f, 0.0f) / std::max(abs(velFilteredSlow[2]), 0.01f)) * deltaTime * 0.03f, 0.0f, 0.1f);

    // std::cout << "factor = " << factor << "        ";


    // weighted average in the vision angle
    visionAngleOffset = wrapAngle(visionAngleOffset + factor * 0.5f * wrapAngle(2.0f * (rawOffset - visionAngleOffset)));
    // posFiltered[2] = wrapAngle(visionTheta + visionAngleOffset);
}



// get robot data
cv::Point2f FilteredRobot::position() { return cv::Point2f(posFiltered[0], posFiltered[1]); }
cv::Point2f FilteredRobot::moveVel() { return cv::Point2f(velFiltered[0], velFiltered[1]); }
float FilteredRobot::angle() { return posFiltered[2]; }
float FilteredRobot::turnVel() { return velFiltered[2]; }
float FilteredRobot::turnAccel() { return accFiltered[2]; }
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


std::vector<float> FilteredRobot::getPosFiltered() {
    return posFiltered;
}
std::vector<float> FilteredRobot::getVelFiltered() {
    return velFiltered;
}
std::vector<float> FilteredRobot::getAccFiltered() {
    return accFiltered;
}