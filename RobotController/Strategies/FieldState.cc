#include "../RobotOdometry.h"
#include "FieldState.h"
#include <iostream>



FieldState::FieldState() { }

FieldState::FieldState(FilteredRobot orb, FilteredRobot opp, float simTime, float timeStep, float maxSimTime, 
        int numEvo) {

    orbFiltered = orb;
    oppFiltered = opp;

    this->simTime = simTime;
    this->timeStep = timeStep;
    this->maxSimTime = maxSimTime;
    this->numEvo = numEvo;
}



// loss function used to compare field states
float FieldState::metric() {

    // float orbETA = orbFiltered.collideETA(oppFiltered, true);
    // float oppETA = oppFiltered.turnTimeSimple(orbFiltered.position(), oppFiltered.getWeaponAngleReach(), true, false);

    // float fraction = 999999999.0f;
    // if (oppETA != 0.0f) { fraction = max((orbETA - oppETA) / oppETA, 0.0f); }





    float distance = oppFiltered.distanceTo(orbFiltered.position());

    float orbETA = (distance - orbFiltered.getSizeRadius() - oppFiltered.getSizeRadius()) / orbFiltered.getMaxMoveSpeed();
    float oppETA = abs(oppFiltered.angleTo(orbFiltered.position(), true)) / oppFiltered.getMaxTurnSpeed();

    float fraction = 999999999.0f;
    if (oppETA != 0.0f) { fraction = max((orbETA - oppETA) / oppETA, -999999999.0f); }


    // return fraction;
    return distance;
}



// starts the calculation chain
float FieldState::chooseInput() {

    // lowest metric we've seen so far and which child it was
    float lowestMetric = 999999999999.0f;
    int lowestChild = -1;

    // make each child with different control inputs
    for(int i = 0; i < numEvo; i++) {
        float turn = calculateChildTurn(i); // get the turn value from -1.0 to 1.0
        std::vector<FilteredRobot> newRobots = extrapRobots(turn); // extrapolate robots with the turn value

        FieldState child = FieldState(newRobots[0], newRobots[1], simTime + timeStep, timeStep, maxSimTime, numEvo); // make child

        // save the lowest metric in this chain
        float childLowest = child.evolve();
        if(childLowest < lowestMetric || i == 0) { 
            lowestMetric = childLowest; 
            lowestChild = i;
        }
    }

    std::cout << "lowest metric = " << lowestMetric << std::endl;
    return calculateChildTurn(lowestChild);
}



// generates child states and returns the lowest metric of thiers
float FieldState::evolve() {

    float lowestMetric = metric(); // choose lowest metric out of this state and all it's children

    if(simTime > maxSimTime) { return lowestMetric; } // if we're done simulating then return
    if(oppFiltered.inWeaponRegion(orbFiltered.position())) { return 99999999999.0f; } // if we got hit then return
    // if(lowestMetric < 0.0f) { return lowestMetric; } // if we can guarenteed hit the op then return
    // IF WE BREAK A CONSTRAINT THEN RETURN VERY HIGH VALUE


    // make each child with different control inputs
    for(int i = 0; i < numEvo; i++) {
        float turn = calculateChildTurn(i); // get the turn value from -1.0 to 1.0
        std::vector<FilteredRobot> newRobots = extrapRobots(turn); // extrapolate robots with the turn value

        FieldState child = FieldState(newRobots[0], newRobots[1], simTime + timeStep, timeStep, maxSimTime, numEvo); // make child

        // save the lowest metric in this chain
        float childLowest = child.evolve();
        if(childLowest < lowestMetric) { lowestMetric = childLowest; }
    }

    return lowestMetric;
}


// returns the most curve orb can currently do
float FieldState::calculateMaxCurve() {

    float currSpeed = orbFiltered.moveSpeedSlow();
    float maxCurveGrip = pow(290.0f, 2.0f) / max(pow(currSpeed, 2), 0.1); // max curvature to avoid slipping, faster you go the less curvature you're allowed
    float maxCurveScrub = currSpeed * 0.01f; // max curvature to avoid turning in place when moving slow, faster you go the more curvature you're allowed cuz you're already moving
    return min(min(maxCurveGrip, maxCurveScrub), 0.7f); // use the lower value as the upper bound
}


// returns the turn input for this child
float FieldState::calculateChildTurn(int i) {
    float maxCurve = calculateMaxCurve();
    float maxTurn = maxCurve / (1.0f + maxCurve);
    return (((2.0f * i) / (numEvo - 1.0f)) - 1.0f) * calculateMaxCurve();
}


// returns sign of input
int FieldState::sign(float value) {
    int sign = 1;
    if(value < 0.0f) { sign = -1; }
    return sign;
}


// extrapolates the robots with a given turn input for orb
std::vector<FilteredRobot> FieldState::extrapRobots(float orbTurn) {
    
    float orbMove = 1.0f - std::abs(orbTurn); // scale linear vs turning effort (1.0 when straight, drops as you steer harder)
    float moveSpeed = orbFiltered.getMaxMoveSpeed() * orbMove;   // linear speed v
    float turnSpeed = orbFiltered.getMaxTurnSpeed() * orbTurn;   // angular speed ω

    float deltaXRobot = 0.0f;
    float deltaYRobot = 0.0f;
    float turnDistance = turnSpeed * timeStep; // Δθ

    if (abs(turnSpeed) < 1e-6f) {
        // straight-line motion
        deltaXRobot = moveSpeed * timeStep;
        deltaYRobot = 0.0f;
    } else {
        // circular arc motion
        float r = moveSpeed / turnSpeed; // path radius
        deltaXRobot = r * std::sin(turnDistance);
        deltaYRobot = r * (1.0f - std::cos(turnDistance));
    }

    // rotate from robot frame to field frame
    float orientation = orbFiltered.angle(true);
    float deltaXField = deltaXRobot * std::cos(orientation) - deltaYRobot * std::sin(orientation);
    float deltaYField = deltaXRobot * std::sin(orientation) + deltaYRobot * std::cos(orientation);

    // new orb orientation and pose
    float newOrbAngle = angleWrapRad(orientation + turnDistance);
    std::vector<float> newOrbPos = {
        orbFiltered.position().x + deltaXField,
        orbFiltered.position().y + deltaYField,
        newOrbAngle
    };

    // new orb velocity
    std::vector<float> newOrbVel = {
        moveSpeed * cos(newOrbAngle),
        moveSpeed * sin(newOrbAngle),
        turnSpeed
    };

    // --- Opponent update ---
    float angleToOrb      = oppFiltered.angleTo(orbFiltered.position(), true);
    float oppTurnDistance = min(oppFiltered.getMaxTurnSpeed() * 0.5f * timeStep, abs(angleToOrb)); 
    oppTurnDistance *= sign(angleToOrb);
    float newOppAngle     = angleWrapRad(oppFiltered.angle(true) + oppTurnDistance);
    std::vector<float> newOppPos = {
        oppFiltered.position().x,
        oppFiltered.position().y,
        newOppAngle
    };

    // Construct updated robots
    FilteredRobot newOrb = orbFiltered;
    newOrb.setPos(newOrbPos);
    newOrb.setVel1(newOrbVel);

    FilteredRobot newOpp = oppFiltered;
    newOpp.setPos(newOppPos);

    // return both
    return { newOrb, newOpp };







    // float orbMove = 1.0f - abs(orbTurn); // assume we are always full gassing it

    // // how fast orb will move as a result of the inputs
    // float moveSpeed = orbFiltered.getMaxMoveSpeed() * orbMove;
    // float turnSpeed = orbFiltered.getMaxTurnSpeed() * orbTurn;

    // float moveDistance = moveSpeed * timeStep;
    // float turnDistance = turnSpeed * timeStep; // how far we move over the time step


    // float orientation = orbFiltered.angle(true);
    // float orbNewX = orbFiltered.position().x + moveDistance*cos(orientation);
    // float orbNewY = orbFiltered.position().y + moveDistance*sin(orientation);
    // float newOrbAngle = angleWrapRad(orientation + turnDistance);

    // std::vector<float> newOrbPos = {orbNewX, orbNewY, newOrbAngle};


    // // make new robots with new poses
    // FilteredRobot newOrb = orbFiltered; newOrb.setPos(newOrbPos);
    // FilteredRobot newOpp = oppFiltered;


    // // return robots
    // return std::vector<FilteredRobot> {newOrb, newOpp};
}
