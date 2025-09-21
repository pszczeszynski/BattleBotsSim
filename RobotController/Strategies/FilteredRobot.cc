#include "FilteredRobot.h"
#include "../MathUtils.h"
#include <iostream>
#include <cstdlib>


#ifndef angleWrapRad
#define angleWrapRad(rad) ((rad < -M_PI) ? fmod(rad + M_PI, 2*M_PI) + M_PI : fmod(rad + M_PI, 2*M_PI) - M_PI)
#endif




// CONSTRUCTOR
FilteredRobot::FilteredRobot() { }

FilteredRobot::FilteredRobot(float pathSpacing, float pathLength, float moveSpeed,
                float moveAccel, float turnSpeed, float turnAccel, float weaponAngleReach, 
                float weaponDriftScaleReach, float sizeRadius) {

    this->pathSpacing = pathSpacing;
    this->pathLength = pathLength;
    this->maxMoveSpeed = moveSpeed;
    this->maxMoveAccel = moveAccel;
    this->maxTurnSpeed = turnSpeed;
    this->maxTurnAccel = turnAccel;
    this->weaponAngleReach = weaponAngleReach;
    this->weaponDriftScaleReach = weaponDriftScaleReach;
    this->sizeRadius = sizeRadius;


    // initialize
    posFiltered = {0, 0, 0};
    velFiltered = {0, 0, 0};
    accFiltered = {0, 0, 0};

    velFilteredSlow = {0, 0, 0};
}

FilteredRobot::FilteredRobot(cv::Point2f position, float sizeRadius) { // make a robot that just has a position and a size

    pathSpacing = 0.0f;
    pathLength = 0.0f;
    maxMoveSpeed = 0.0f;
    maxMoveAccel = 0.0f;
    maxTurnSpeed = 0.0f;
    maxTurnAccel = 0.0f;
    weaponAngleReach = 0.0f;
    weaponDriftScaleReach = 0.0f;

    this->sizeRadius = sizeRadius;

    posFiltered = {position.x, position.y, 0.0f};
    velFiltered = {0, 0, 0};
    accFiltered = {0, 0, 0};

    velFilteredSlow = {0, 0, 0};
}


// hard sets the pos
void FilteredRobot::setPos(std::vector<float> pos) { posFiltered = pos; }
void FilteredRobot::setVel(std::vector<float> vel) { velFiltered = vel; velFilteredSlow = vel; }
void FilteredRobot::setAccel(std::vector<float> acc) { accFiltered = acc; }



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

    float deltaXRobot = 0.0f;
    float deltaYRobot = 0.0f;
    float deltaT = velFilteredSlow[2] * time; // Δθ

    float velDirection = atan2(velFiltered[1], velFiltered[0]); // direction of the current vel
    float currSpeed = moveSpeedSlow();

    if (abs(velFilteredSlow[2]) < 1e-6f) { // straight-line motion
        deltaXRobot = currSpeed * time;
        deltaYRobot = 0.0f;

    } else { // circular arc motion
        float r = currSpeed / velFilteredSlow[2]; // path radius
        deltaXRobot = r * std::sin(deltaT);
        deltaYRobot = r * (1.0f - std::cos(deltaT));
    }

    // rotate from robot frame to field frame
    float deltaXField = deltaXRobot * cos(velDirection) - deltaYRobot * sin(velDirection);
    float deltaYField = deltaXRobot * sin(velDirection) + deltaYRobot * cos(velDirection);

    // new pos
    std::vector<float> extrapolatedPos = {
        posFiltered[0] + deltaXField,
        posFiltered[1] + deltaYField,
        (float) angleWrapRad(posFiltered[2] + deltaT)
    }; 

    // rotate vel by delta angle
    float newVelX = velFilteredSlow[0] * cos(deltaT) - velFilteredSlow[1] * sin(deltaT);
    float newVelY = velFilteredSlow[0] * sin(deltaT) + velFilteredSlow[1] * cos(deltaT);

    // new vel
    std::vector<float> extrapolatedVel = {
        newVelX,
        newVelY,
        velFilteredSlow[2]
    };

    // return final values
    return std::vector<std::vector<float>> {extrapolatedPos, extrapolatedVel};
}


// extraps with constant vel and writes those positions and vels
void FilteredRobot::constVelExtrapWrite(float time) {

    std::vector<std::vector<float>> posVel = constVelExtrap(time);
    setPos(posVel[0]);
    setVel(posVel[1]);
    setAccel({0.0f, 0.0f, 0.0f});
}


// extrapolates the opp's pos to see what point we'll line up at
FilteredRobot FilteredRobot::createVirtualOpp(FilteredRobot opp, bool forward, float maxExtrapTime, float timeOffsetFactor, std::vector<cv::Point2f> &path) {

    float timeIncrement = 0.01f; // time increment of the simulation
    float simTime = 0.0f; // time in the sim
    float distanceToCollision = distanceTo(opp.position()) - sizeRadius - opp.getSizeRadius();
    float timeOffset = timeOffsetFactor * distanceToCollision; // how much we want to lag the opponent to make sure our side doesn't get hit, more lag (higher value) = less likely to lead them but more likely to get side hit

    FilteredRobot virtualOpp = opp; // will return this virtual opp
    cv::Point2f oppPositionExtrap = opp.position(); // extrapolated opp pos, starts on the opp

    // run until the sim time is above the max time
    while(simTime < maxExtrapTime) {

        std::vector<cv::Point2f> bruh = {};
        float orbETA = ETASim(virtualOpp, bruh, false, false, forward);
        if(orbETA < simTime + timeOffset) { break; } // break when we arrive at the same time
        if(virtualOpp.moveSpeedSlow() < 1.0f && abs(virtualOpp.turnVel()) < 0.01f) { break; } // break if opp's vel has decayed


        simTime += timeIncrement; // increment sim time
        std::vector<std::vector<float>> oppExtrap = virtualOpp.constVelExtrap(timeIncrement); // extrapolate opp another time step

        float velLeft1Sec = 0.1f; // what percent of velocity is left after each second
        float velPercent = pow(velLeft1Sec, timeIncrement); // what percent of velocity is left after this timestep

        float turnLeft1Sec = 0.1f; 
        float turnPercent = pow(turnLeft1Sec, timeIncrement);

        std::vector<float> newPos = oppExtrap[0]; // pull out new pos
        std::vector<float> newVel = oppExtrap[1]; // pull out new vel

        newVel[0] *= velPercent; 
        newVel[1] *= velPercent;
        newVel[2] *= turnPercent;

        virtualOpp.setPos(newPos);
        virtualOpp.setVel(newVel);
        virtualOpp.setAccel({0, 0, 0});

        path.emplace_back(cv::Point2f(newPos[0], newPos[1])); // save the point to be returned
    }

    return virtualOpp; // return the extrapped opp
}


// simulates us and opp into the future to calculate ETA times and extrapolate appropriately
void dynamicSim(FilteredRobot& opp, std::vector<cv::Point2f> &path, std::vector<cv::Point2f> oppPath, bool forward) {

    float timeIncrement = 0.01f;
    float simTime = 0.0f;

    

    FilteredRobot virtualOpp = opp; // will return this virtual opp
    cv::Point2f oppPositionExtrap = opp.position(); // extrapolated opp pos, starts on the opp

}


// calculates move and turn speeds to follow the followPoint
std::vector<float> FilteredRobot::curvatureController(cv::Point2f followPoint, float kP, float kD, float moveInput, float deltaTime, int turnDirection, bool forward, cv::Point2f oppVel) {

    float oppSpeed = cv::norm(oppVel); // how fast the opp is moving in absolute

    float pointAngle = atan2(followPoint.y - posFiltered[1], followPoint.x - posFiltered[0]); // absolute angle to the point

    float p = oppVel.x*cos(pointAngle) + oppVel.y*sin(pointAngle);
    float s = sqrt(pow(maxMoveSpeed, 2) - pow(oppVel.x*sin(pointAngle) - oppVel.y*cos(pointAngle), 2));

    float pointSpeed = 0.0f;
    if(s > p) { pointSpeed = -p + s; } // thanks chat, ensures vector sum is always to maxMoveSpeed

    cv::Point2f pointVel = cv::Point2f(pointSpeed*cos(pointAngle), pointSpeed*sin(pointAngle)); // vector in the direction of the followPoint
    cv::Point2f totalVel = oppVel + pointVel; // sum the opp's vel with our desired vel in their frame
    float targetAngle = atan2(totalVel.y, totalVel.x); // target angle is in the direction of the total vector

    float angleError = angle_wrap(targetAngle - posFiltered[2]); // how far off we are from target
    if(turnDirection == 1) { angleError = angleWrapRad(angleError - M_PI) + M_PI; }
    if(turnDirection == -1) { angleError = angleWrapRad(angleError + M_PI) - M_PI; }

    float angleErrorChange = angleWrapRad(angleError - prevAngleError) / deltaTime; // how the error is changing per time
    prevAngleError = angleError; // save for next time

    // determine desired path curvature/drive radius using pd controller and magic limits
    float currSpeed = moveSpeedSlow();

    float maxCurveGrip = pow(300.0f, 2.0f) / std::max(pow(currSpeed, 2), 0.1); // max curvature to avoid slipping, faster you go the less curvature you're allowed
    float maxCurveScrub = currSpeed * 0.01f; // max curvature to avoid turning in place when moving slow, faster you go the more curvature you're allowed cuz you're already moving
    float maxCurve = std::min(maxCurveGrip, maxCurveScrub); // use the lower value as the (upper) bound

    // pd controller that increases path curvature with angle error
    float curvature = std::clamp(kP*angleError + kD*angleErrorChange, -maxCurve, maxCurve);

    moveInput = abs(moveInput); // just take magnitude of input
    if(!forward) { moveInput *= -1.0f; } // reverese the input if we're going backwards
    float turnInput = abs(moveInput) * curvature; // curvature = 1/radius so this is the same as w = v/r formula

    // don't demand more than 1.0 speed from any one motor side, scale everything down since that will maintain the same path curvature
    float maxInput = abs(moveInput) + abs(turnInput);
    if(maxInput > 1.0f) {
        moveInput /= maxInput;
        turnInput /= maxInput;
    }

    return std::vector<float> {moveInput, turnInput};
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
    float positionKalmanGain = std::clamp(deltaTime * 30.0f, 0.05f, 1.0f); // 15.0f
    float turnKalmanGain = std::clamp(deltaTime * 50.0f, 0.05f, 1.0f); // 40.0f
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
    for(int i = 0; i < 3; i++) { velFilteredSlow[i] += std::clamp(deltaTime * 10.0f, 0.0f, 0.2f) * (velFiltered[i] - velFilteredSlow[i]); }
}


// velocity thats directed away from a given point, positive is away
float FilteredRobot::velAwayFromPoint(cv::Point2f point) {

    float speed = cv::norm(moveVelSlow());
    float speedAngle = atan2(velFilteredSlow[1], velFilteredSlow[0]);
    float pointToUsAngle = atan2(position().y - point.y, position().x - point.x);
    return speed * cos(speedAngle - pointToUsAngle);
}


// if we're colliding with another robot within a tolerance
bool FilteredRobot::colliding(FilteredRobot opp, float tolerance) {
    return distanceTo(opp.position()) < sizeRadius + opp.getSizeRadius() + tolerance;
}


// if we're facing another robot
bool FilteredRobot::facing(FilteredRobot opp, bool forward) {
    return abs(angleTo(opp.position(), forward)) < weaponAngleReach;
}



// how long to collide with a robot using current velocity and assumed turn speed
float FilteredRobot::collideETA(FilteredRobot& opp, bool forward) {

    // how much orb velocity towards the opp do we actually count
    float angleThresh1 = 50.0f*TO_RAD; // 60.0f
    float angleThresh2 = 140.0f*TO_RAD; // 130.0f

    float velAngle = atan2(velFilteredSlow[1], velFilteredSlow[0]); // angle we're actually moving in
    float angleToOppAbs = atan2(opp.position().y - posFiltered[1], opp.position().x - posFiltered[0]);
    float angleToOppVel = angle_wrap(angleToOppAbs - velAngle);
    if(cv::norm(moveVel()) == 0.0f) { angleToOppVel = 0.0f; }
    float angleToOpp = angleToOppVel;

    // how good can our velocity be used to drive directly to the opponent if we turn directly towards them
    float velFactor = 1.0f - std::clamp((abs(angleToOpp) - angleThresh1) / (angleThresh2 - angleThresh1), 0.0f, 1.0f);    
    int velSign = sign(tangentVel(forward)); // measure direction of main drive velocity
    float orbUsableSpeed = cv::norm(moveVelSlow()) * velSign * velFactor;

    // subtract out the opponents velocity away from us so we don't get overconfident
    float oppVelAway = opp.velAwayFromPoint(position());
    orbUsableSpeed -= oppVelAway;

    // clip just in case
    orbUsableSpeed = std::clamp(orbUsableSpeed, 1.0f, maxMoveSpeed);

    // estimate time to turn to opp
    // float turnTime = turnTimeMin(opp.position(), 0.0f, 30.0f*TO_RAD, forward, false);
    float turnTime = turnTimeSimple(opp.position(), weaponAngleReach, forward, false);

    // estimate time to drive to opp
    float collisionDistance = sizeRadius + opp.getSizeRadius();
    float distanceToCollision = std::max(distanceTo(opp.position()) - collisionDistance, 0.0f);
    float moveTime = moveETASim(distanceToCollision, orbUsableSpeed, false);

    // total time is sum of turning and driving
    return moveTime + turnTime;
}


// simulates orb's path to the opp to calculate the ETA
float FilteredRobot::ETASim(FilteredRobot target, std::vector<cv::Point2f> &path, bool stopIfHit, bool orbNeedsToFace, bool forward) {

    int direction = 1; if(!forward) { direction = -1; }

    std::vector<float> simPos = posFiltered;
    float simSpeed = moveSpeedSlow() * direction;

    float timeStep = 0.005f;
    float maxTime = 2.0f;

    float simTime = 0.0f;

    int steps = (int) (maxTime / timeStep);

    for(int i = 0; i < steps; i++) { // simulate orb's actions through each time step

        path.emplace_back(cv::Point2f(simPos[0], simPos[1])); // add the point to the path

        // pasted BS plz fix
        FilteredRobot dummyOrb = *this;
        dummyOrb.setPos(simPos);
        float angleError = dummyOrb.angleTo(target.position(), forward); // how far off we are from target



        // if we're in the opp's weapon range then return a high number so fraction goes high (if we want to stop if hit)
        if((abs(target.angleTo(dummyOrb.position(), true)) < target.getWeaponAngleReach()) && stopIfHit) {
            simTime = 9999999999.0f;
            break;
        }

        // if we're close enough to collide and facing opp, then return
        bool colliding = dummyOrb.distanceTo(target.position()) < dummyOrb.getSizeRadius() + target.getSizeRadius();
        bool facing = abs(dummyOrb.angleTo(target.position(), forward)) < 80.0f*TO_RAD;
        if(colliding && (facing || !orbNeedsToFace)) { break; }


        // 290
        float maxCurveGrip = pow(300.0f, 2.0f) / std::max(pow(abs(simSpeed), 2), 0.1); // max curvature to avoid slipping, faster you go the less curvature you're allowed
        float maxCurveScrub = abs(simSpeed) * 0.01f; // max curvature to avoid turning in place when moving slow, faster you go the more curvature you're allowed cuz you're already moving
        float maxCurve = std::min(maxCurveGrip, maxCurveScrub); // use the lower value as the (upper) bound

        // pd controller that increases path curvature with angle error
        float curvature = std::clamp(1.0f*angleError, -maxCurve, maxCurve); // 0.8f, 0.06f

        float moveSpeed = 1.0f * direction; // assume full gas
        float turnSpeed = abs(moveSpeed) * curvature; // curvature = 1/radius so this is the same as w = v/r formula

        // don't demand more than 1.0 speed from any one motor side, scale everything down since that will maintain the same path curvature
        float totalSpeed = abs(moveSpeed) + abs(turnSpeed);
        if(totalSpeed > 1.0f) {
            moveSpeed /= totalSpeed;
            turnSpeed /= totalSpeed;
        }

        std::vector<float> controlInput {moveSpeed, turnSpeed};
        // end of pasted bs plz fix above



        simSpeed = controlInput[0] * getMaxMoveSpeed(); // calculate new speed, also saves for next time
        float simTurnSpeed = controlInput[1] * maxTurnSpeed;

        float deltaXRobot = 0.0f;
        float deltaYRobot = 0.0f;
        float turnDistance = simTurnSpeed * timeStep; // Δθ

        if (abs(simTurnSpeed) < 1e-6f) {
            // straight-line motion
            deltaXRobot = simSpeed * timeStep;
            deltaYRobot = 0.0f;
        } else {
            // circular arc motion
            float r = simSpeed / simTurnSpeed; // path radius
            deltaXRobot = r * std::sin(turnDistance);
            deltaYRobot = r * (1.0f - std::cos(turnDistance));
        }

        // rotate from robot frame to field frame
        float orientation = simPos[2];
        float deltaXField = deltaXRobot * std::cos(orientation) - deltaYRobot * std::sin(orientation);
        float deltaYField = deltaXRobot * std::sin(orientation) + deltaYRobot * std::cos(orientation);


        // new pos in sim
        simPos = {
            simPos[0] + deltaXField,
            simPos[1] + deltaYField,
            (float) angleWrapRad(orientation + turnDistance)
        };


        simTime += timeStep;
    }

    return simTime;
}


// signed velocity that's actually in the driving direction, positive means it's in the direction of orientation
float FilteredRobot::tangentVel(bool forward) {
    float velAngle = atan2(velFilteredSlow[1], velFilteredSlow[0]);
    float velAngleOffset = angle_wrap(velAngle - posFiltered[2]);
    int sign = 1; if(!forward) { sign = -1; }
    return moveSpeedSlow() * cos(velAngleOffset) * sign;
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
    // if (i == kMaxIterations) {
    //     std::cout << "Warning: moveETASim reached max iterations" << std::endl;
    // }

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
        velSim[2] = std::clamp(velSim[2], -maxTurnSpeed, maxTurnSpeed);

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

// calculates time to turn assuming constant vel
float FilteredRobot::turnTimeSimple(cv::Point2f point, float angleMargin, bool forward, bool print) {

    float angleError = angleTo(point, forward);
    if(abs(angleError) < angleMargin) { return 0.0f; } // if we're already facing then return 0 seconds

    float angleToTurn = abs(angleError) - angleMargin;
    return angleToTurn / maxTurnSpeed;
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
    float offset = forward ? 0.0f : M_PI; // turn by 180 if its backwards
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




// get robot data
cv::Point2f FilteredRobot::position() { return cv::Point2f(posFiltered[0], posFiltered[1]); }
cv::Point2f FilteredRobot::moveVel() { return cv::Point2f(velFiltered[0], velFiltered[1]); }
cv::Point2f FilteredRobot::moveVelSlow() { return cv::Point2f(velFilteredSlow[0], velFilteredSlow[1]); }
float FilteredRobot::moveSpeedSlow() { return cv::norm(moveVelSlow()); }

float FilteredRobot::angle(bool forward) { 
    int offset = 0.0f; if(!forward) { offset = M_PI; } // offset by 180 if not forwards
    return angle_wrap(posFiltered[2] + offset); 
}

float FilteredRobot::turnVel() { return velFiltered[2]; }
float FilteredRobot::getMaxTurnSpeed() { return maxTurnSpeed; }
float FilteredRobot::getMaxMoveSpeed() { return maxMoveSpeed; }
float FilteredRobot::moveAccel() { return cv::norm(cv::Point2f(accFiltered[0], accFiltered[1])); }
float FilteredRobot::turnAccel() { return accFiltered[2]; }
std::vector<cv::Point2f> FilteredRobot::getPath() { return path; }
float FilteredRobot::getWeaponAngleReach() { return weaponAngleReach; }
float FilteredRobot::getWeaponDriftScaleReach() { return weaponDriftScaleReach; }
float FilteredRobot::getSizeRadius() { return sizeRadius; }




// clips the inputted point to the field bounds
cv::Point2f FilteredRobot::clipInBounds(cv::Point2f point) {
    float resultX = std::clamp(point.x, fieldMin, fieldMax);
    float resultY = std::clamp(point.y, fieldMin, fieldMax);
    
    return cv::Point2f(resultX, resultY);
}


std::vector<float> FilteredRobot::getPosFiltered() { return posFiltered; }
std::vector<float> FilteredRobot::getVelFiltered() { return velFiltered; }
std::vector<float> FilteredRobot::getAccFiltered() { return accFiltered; }
std::vector<float> FilteredRobot::getVelFilteredSlow() { return velFilteredSlow; }