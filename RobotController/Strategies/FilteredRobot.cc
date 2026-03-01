#include "FilteredRobot.h"
#include "../MathUtils.h"
#include "../RobotController.h"
#include <iostream>
#include <cstdlib>
#include <algorithm>
using std::max;
using std::min;


// CONSTRUCTOR
FilteredRobot::FilteredRobot() { }

FilteredRobot::FilteredRobot(float pathSpacing, float pathLength, float moveSpeed, float moveAccel, float turnSpeed, 
    float turnAccel, float weaponAngleReach, float sizeRadius) {

    this->pathSpacing = pathSpacing;
    this->pathLength = pathLength;
    this->maxMoveSpeed = moveSpeed;
    this->maxMoveAccel = moveAccel;
    this->maxTurnSpeed = turnSpeed;
    this->maxTurnAccel = turnAccel;
    this->weaponAngleReach = weaponAngleReach;
    this->sizeRadius = sizeRadius;


    // initialize
    posFiltered = {0, 0, 0};
    velFiltered = {0, 0, 0};
    accFiltered = {0, 0, 0};

    velFilteredSlow = {0, 0, 0};
    velFilteredUltraSlow = {0, 0, 0};

    modelParams = {};
    modelParamScales = {};
}

FilteredRobot::FilteredRobot(cv::Point2f position, float sizeRadius) { // make a robot that just has a position and a size

    pathSpacing = 0.0f;
    pathLength = 0.0f;
    maxMoveSpeed = 0.0f;
    maxMoveAccel = 0.0f;
    maxTurnSpeed = 0.0f;
    maxTurnAccel = 0.0f;
    weaponAngleReach = 0.0f;

    this->sizeRadius = sizeRadius;

    posFiltered = {position.x, position.y, 0.0f};
    velFiltered = {0, 0, 0};
    accFiltered = {0, 0, 0};

    velFilteredSlow = {0, 0, 0};
    velFilteredUltraSlow = {0, 0, 0};
}


// hard sets the pos
void FilteredRobot::setPos(std::vector<float> pos) { posFiltered = pos; }
void FilteredRobot::setVel(std::vector<float> vel) { velFiltered = vel; velFilteredSlow = vel; }
void FilteredRobot::setAccel(std::vector<float> acc) { accFiltered = acc; }
void FilteredRobot::setSizeRadius(float sizeRadius) { this->sizeRadius = sizeRadius; }




// incrementally extrapolates with constant accel
std::vector<std::vector<float>> FilteredRobot::kalmanExtrapAccel(float time) {

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



// extrapolates with constant vel
std::vector<std::vector<float>> FilteredRobot::kalmanExtrapVel(float time) {

    float deltaXRobot = 0.0f;
    float deltaYRobot = 0.0f;
    float deltaT = velFiltered[2] * time; // Δθ

    float velDirection = atan2(velFiltered[1], velFiltered[0]); // direction of the current vel



    if (abs(velFiltered[2]) < 1e-6f) { // straight-line motion
        deltaXRobot = moveSpeed() * time;
        deltaYRobot = 0.0f;

    } else { // circular arc motion
        float r = moveSpeed() / turnVel(); // path radius
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
        (float) angle_wrap(posFiltered[2] + deltaT)
    }; 

    // rotate vel by delta angle
    float newVelX = velFiltered[0] * cos(deltaT) - velFiltered[1] * sin(deltaT);
    float newVelY = velFiltered[0] * sin(deltaT) + velFiltered[1] * cos(deltaT);

    // new vel
    std::vector<float> extrapolatedVel = {
        newVelX,
        newVelY,
        velFiltered[2]
    };

    // return final values
    return std::vector<std::vector<float>> {extrapolatedPos, extrapolatedVel};
}



// extrapolates with constant vel (circular arc motion)
std::vector<std::vector<float>> FilteredRobot::constVelExtrap(float time) {

    float deltaXRobot = 0.0f;
    float deltaYRobot = 0.0f;
    float deltaT = velFilteredSlow[2] * time;

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
        (float) angle_wrap(posFiltered[2] + deltaT)
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



// sets velFiltered to velFilteredSlow
void FilteredRobot::setToSlowVel() {
    velFiltered = velFilteredSlow;
}




// calculates move and turn speeds to follow the followPoint WEIRD PD BS
std::vector<float> FilteredRobot::curvatureController(float targetAngle, float moveInput, float deltaTime, bool forward, int enforceTurnDirection) {

    float ogMoveInput = moveInput;

    float reverseOffset = forward ? 0 : M_PI; // add a 180deg offset to target angle if going backward
    float angleError = angle_wrap(targetAngle - posFiltered[2] + reverseOffset); // how far off we are from target

    // rewrap based on specified turn direction
    if(enforceTurnDirection == 1) { angleError = angle_wrap(angleError - M_PI) + M_PI; }
    if(enforceTurnDirection == -1) { angleError = angle_wrap(angleError + M_PI) - M_PI; }

    // determine desired path curvature/drive radius using pd controller and magic limits
    float currSpeed = moveSpeedSlow();



    float slowGain = 1.0f; // 0.88
    float fastGain = 0.40f; // 0.32
    float fastSpeed = 400.0f;
    float gain = (std::max)(slowGain - (slowGain - fastGain)*pow(currSpeed / fastSpeed, 0.5f), 0.0f);
    float curvature = gain * angleError;


    // by default, apply input velocity and turn based on that
    moveInput = abs(moveInput); // just take magnitude of input
    if(!forward) { moveInput *= -1.0f; } // reverese the input if we're going backwards
    float turnInput = abs(moveInput) * curvature; // curvature = 1/radius so this is the same as w = v/r formula

    // don't demand more than 1.0 speed from any one motor side, scale everything down since that will maintain the same path curvature
    float maxInput = abs(moveInput) + abs(turnInput);
    if(maxInput > 1.0f) {
        moveInput /= maxInput;
        turnInput /= maxInput;
    }


    float turnSpeedChange = (turnInput - prevAngleError) / deltaTime; // angular acceleration needed for curvature change
    prevAngleError = turnInput; // save the turn speed wanted for the normal amount of curvature


    // float turnSpeedOffset = pow(abs(turnSpeedChange), 1.3f) * sign(turnSpeedChange) * 0.14f;
    float turnSpeedOffset = 0.11f * turnSpeedChange;
    turnInput = std::clamp(turnInput + turnSpeedOffset, -1.0f, 1.0f);


    // use up extra juice or clip
    moveInput = (1.0f - abs(turnInput)) * sign(moveInput);
    moveInput = std::clamp(moveInput, -abs(ogMoveInput), abs(ogMoveInput));
    


    return std::vector<float> {moveInput, turnInput};
}




// displays circle of size radius and weapon region lines
void FilteredRobot::displayRobot(int thick, cv::Scalar sizeColor, cv::Scalar weaponColor, bool forward) {

    cv::Point2f weapon1 = position() + sizeRadius*cv::Point2f(cos(angle(forward) + weaponAngleReach), sin(angle(forward) + weaponAngleReach));
    cv::Point2f weapon2 = position() + sizeRadius*cv::Point2f(cos(angle(forward) - weaponAngleReach), sin(angle(forward) - weaponAngleReach));
    cv::Point2f weaponCenter = position() + sizeRadius*cv::Point2f(cos(angle(forward)), sin(angle(forward)));

    // draw weapon region and center line
    cv::line(RobotController::GetInstance().GetDrawingImage(), position(), weapon1, weaponColor, thick);
    cv::line(RobotController::GetInstance().GetDrawingImage(), position(), weapon2, weaponColor, thick);
    cv::line(RobotController::GetInstance().GetDrawingImage(), position(), weaponCenter, weaponColor, thick);

    safe_circle(RobotController::GetInstance().GetDrawingImage(), position(), sizeRadius, sizeColor, thick); // draw size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), position(), 3, sizeColor, 5); // draw dot
}





// updates the position filters
void FilteredRobot::updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta) {

    if(visionPos.x < fieldMin || visionPos.x > fieldMax || std::isnan(visionPos.x)) { return; }
    if(visionPos.y < fieldMin || visionPos.y > fieldMax || std::isnan(visionPos.y)) { return; }


    // extrapolate each state for new beliefs
    std::vector<std::vector<float>> outputs = kalmanExtrapAccel(deltaTime);
    // std::vector<std::vector<float>> outputs = kalmanExtrapVel(deltaTime);
    std::vector<float> posBelief = outputs[0];
    std::vector<float> velBelief = outputs[1];
    std::vector<float> accBelief = outputs[2];


 
    // from measurement, find deltas to pos
    std::vector<float> posDelta = {visionPos.x - posBelief[0], visionPos.y - posBelief[1], angle_wrap(visionTheta - posBelief[2])};



    // what even are covariances, determines percentage of delta to use
    float positionKalmanGain = 1 - exp(-deltaTime / 0.05f); // std::clamp(deltaTime * 30.0f, 0.05f, 1.0f);
    float turnKalmanGain = 1 - exp(-deltaTime / 0.03f); // std::clamp(deltaTime * 50.0f, 0.05f, 1.0f);
    std::vector<float> kalmanGain = {positionKalmanGain, positionKalmanGain, turnKalmanGain};


    std::vector<float> posFilteredNew = {0, 0, 0};
    std::vector<float> velFilteredNew = {0, 0, 0};
    std::vector<float> accFilteredNew = {0, 0, 0};

    float maxVel = 1000.0f;
    float maxAccel = 100000.0f;


    // update each axis
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
    for(int i = 0; i < 3; i++) { velFilteredSlow[i] += (1 - exp(-deltaTime / 0.05f)) * (velFiltered[i] - velFilteredSlow[i]); }
    for(int i = 0; i < 3; i++) { velFilteredUltraSlow[i] += (1 - exp(-deltaTime / 0.10f)) * (velFiltered[i] - velFilteredUltraSlow[i]); }
}



// prints model parameters
void FilteredRobot::printModel() {
    std::cout << "model = ";

    for(int i = 0; i < modelParams.size(); i++) {
        std::cout << " " << modelParams[i] << ",";
    }

    std::cout << std::endl;
}




// runs the model and tunes it if wanted
void FilteredRobot::tuneModel(bool autoTune, std::vector<float> inputs, std::vector<float> trueOutputs) {

    // if we're not turning then just run it and don't adjust model
    std::vector<float> virtualInputs = {inputs[0], inputs[1], tangentVelFast(true), turnVel(), inputs[4] };
    if(!autoTune) {
        updateModel(virtualInputs, modelParams, true);
        return;
    }



    // try adjusting every parameter and adjust it in the good direction
    std::vector<float> deltas = {-0.1f, 0.1f};
    std::vector<float> updatedModel = modelParams; // will become the new model

    // for each parameter, calculate partial derivative
    for(int param = 0; param < modelParams.size(); param++) {

        // will record the error at each delta
        std::vector<float> errorMetrics = {0, 0};

        for(int delta = 0; delta < deltas.size(); delta++) {

            // create a model with an altered parameter
            std::vector<float> testModel = modelParams;
            testModel[param] += deltas[delta];

            // calculate the outputs with those parameters
            std::vector<float> testOutputs = updateModel(inputs, testModel, false);


            // error metric is sum of error squares
            std::vector<float> error = {0, 0};
            std::vector<float> errorScales = {400.0f, 20.0f};

            for(int out = 0; out < error.size(); out++) {
                error[out] = (testOutputs[out] - trueOutputs[out]) / errorScales[out]; // normalize so all params are equally weighted;
                errorMetrics[delta] += pow(error[out], 2);
            }
        }

        // partial derivative of error function
        float slope = (errorMetrics[1] - errorMetrics[0]) / (deltas[1] - deltas[0]);

        // update the parameter in the direction of down slope
        float maxUpdate = modelParamScales[param] * 0.003f;
        updatedModel[param] -= std::clamp(300.0f * modelParamScales[param] * slope, -maxUpdate, maxUpdate);


        // std::cout << "best gain = " << changeGains[lowestErrorIndex];
    }

    // run the new model to write new positions
    modelParams = updatedModel;
    updateModel(virtualInputs, modelParams, true);



    std::vector<float> newModelOutputs = updateModel(inputs, modelParams, false);

    // std::cout << "true = " << trueOutputs[0] << ", " << trueOutputs[1] << ", model = " << newModelOutputs[0] << ", " << newModelOutputs[1] << std::endl;

}



// predicts how this robot moves based on inputs and current state
// give it an empty model vector if you want to use the default model
std::vector<float> FilteredRobot::updateModel(std::vector<float> inputs, std::vector<float>& model, bool writePos) {

    // default values
    if(model.size() == 0) { 
        model = {430, 26.6, 14.0, 10.0, 1.30, 1.71, 0.73, 0.91, 0, 0};
        modelParamScales = model; // default values used as scales
    }


    // assign parameters
    float pMaxMoveSpeed = model[0];
    float pMaxTurnSpeed = model[1];
    float pMoveAccel = model[2];
    float pTurnAccel = model[3];
    float pMoveInputPower = model[4];
    float pTurnInputPower = model[5];
    float pMoveAccelPower = model[6];
    float pTurnAccelPower = model[7];
    float pMoveDrag = model[8];
    float pTurnDrag = model[9];

    // input mapping
    float moveInput = inputs[0];
    float turnInput = inputs[1];
    float initialMoveSpeed = inputs[2];
    float initialTurnSpeed = inputs[3];
    float deltaTime = inputs[4];


    // adjust move and turn inputs with a curve to shape it to the actual output
    moveInput = pow(abs(moveInput), pMoveInputPower) * sign(moveInput);
    turnInput = pow(abs(turnInput), pTurnInputPower) * sign(turnInput);


    // don't demand more than 1.0 speed from any one motor side
    float totalSpeed = abs(moveInput) + abs(turnInput);
    if(totalSpeed > 1.0f) {
        moveInput /= totalSpeed;
        turnInput /= totalSpeed;
    }

    // free speeds as defined by simulated curvature controller
    float desiredLinearSpeed = moveInput * pMaxMoveSpeed;
    float desiredTurnSpeed = turnInput * pMaxTurnSpeed;

    // how far off from free speeds we are
    float moveSpeedError = desiredLinearSpeed - initialMoveSpeed;
    float turnSpeedError = desiredTurnSpeed - initialTurnSpeed;

    // how much we accelerate linearly and rotationally, basically the F = ma of this model
    float linearAccel = pMoveAccel * pow(abs(moveSpeedError), pMoveAccelPower) * sign(moveSpeedError) - sign(initialMoveSpeed) * pMoveDrag;
    float turnAccel = pTurnAccel * pow(abs(turnSpeedError), pTurnAccelPower) * sign(turnSpeedError) - sign(initialTurnSpeed) * pTurnDrag;

    // increment speeds by accels
    float newMoveVel = initialMoveSpeed + std::clamp(linearAccel * deltaTime, -abs(moveSpeedError), abs(moveSpeedError));
    float newTurnVel = initialTurnSpeed + std::clamp(turnAccel * deltaTime, -abs(turnSpeedError), abs(turnSpeedError));
    // float newMoveVel = initialMoveSpeed + linearAccel * deltaTime;
    // float newTurnVel = initialTurnSpeed + turnAccel * deltaTime;


    
    float deltaXRobot = 0.0f;
    float deltaYRobot = 0.0f;
    float turnDistance = newTurnVel * deltaTime;

    if (abs(newTurnVel) < 1e-6f) {
        // straight-line motion
        deltaXRobot = newMoveVel * deltaTime;
        deltaYRobot = 0.0f;
    } else {
        // circular arc motion
        float r = newMoveVel / newTurnVel; // path radius
        deltaXRobot = r * std::sin(turnDistance);
        deltaYRobot = r * (1.0f - std::cos(turnDistance));
    }

    // rotate from robot frame to field frame
    float orientation = posFiltered[2];
    float deltaXField = deltaXRobot * std::cos(orientation) - deltaYRobot * std::sin(orientation);
    float deltaYField = deltaXRobot * std::sin(orientation) + deltaYRobot * std::cos(orientation);


    // only write new positions if inputted
    if(writePos) {
        posFiltered = {
            posFiltered[0] + deltaXField,
            posFiltered[1] + deltaYField,
            (float) angle_wrap(orientation + turnDistance),
        };
    
        velFiltered = {
            newMoveVel*cos(posFiltered[2]),
            newMoveVel*sin(posFiltered[2]),
            newTurnVel
        };
    
        for(int i = 0; i < 3; i++) { velFilteredSlow[i] += (1 - exp(-deltaTime / 0.05f)) * (velFiltered[i] - velFilteredSlow[i]); }
    
        accFiltered = {
            0,
            0,
            0
        };
    }

    return std::vector<float> {newMoveVel, newTurnVel};
}



// if we're colliding with another robot within a tolerance
bool FilteredRobot::colliding(FilteredRobot opp, float tolerance) {
    return distanceToCollide(opp) <= tolerance;
}


// if we're facing another robot
bool FilteredRobot::facing(FilteredRobot opp, bool forward) {
    return abs(angleTo(opp.position(), forward)) < weaponAngleReach;
}

bool FilteredRobot::facingPoint(cv::Point2f point, bool forward) {
    return abs(angleTo(point, forward)) < weaponAngleReach;
}



// simple calculation for how long it'll take to get to a point
float FilteredRobot::collideETASimple(cv::Point2f point, float pointSizeRadius, bool forward) {

    float driveDistance = (std::max)(distanceTo(point) - pointSizeRadius - sizeRadius, 0.0f);
    float initialVel = tangentVel(forward) * cos(angleTo(point, forward) * 0.5f);
    float moveTime = moveETAAccel(driveDistance, initialVel);

    float turnTime = pointETAAccel(point, forward, weaponAngleReach);

    return turnTime + moveTime;
}



// how long to drive striaght this distance given initial velocity
float FilteredRobot::moveETAAccel(float distance, float startingVel) {

    if (distance == 0.0f) return 0.0f;

    float t = 0.0f;

    // If moving opposite direction, brake to zero first
    if ((distance > 0.0f && startingVel < 0.0f) || (distance < 0.0f && startingVel > 0.0f)) {

        float t_brake = std::abs(startingVel) / maxMoveAccel;
        float d_brake = (startingVel * startingVel) / (2.0f * maxMoveAccel);

        t += t_brake;
        distance -= (distance > 0.0f ? d_brake : -d_brake);
        startingVel = 0.0f;

        // If braking alone reaches/passes the target
        if ((distance > 0.0f && distance <= 0.0f) || (distance < 0.0f && distance >= 0.0f)) { return t; }
    }

    // Distance needed to reach max velocity
    float d_to_vmax =
        (maxMoveSpeed*maxMoveSpeed - startingVel*startingVel) /
        (2.0f * maxMoveAccel);

    // Case: never reaches max velocity (only accel phase)
    if ((distance > 0.0f && d_to_vmax >= distance) || (distance < 0.0f && d_to_vmax <= distance)) {

        // Solve: distance = startingVel*t + 0.5*a*t^2
        float A = 0.5f * maxMoveAccel * (distance >= 0.0f ? 1.0f : -1.0f);
        float B = startingVel;
        float C = -distance;

        float disc = B*B - 4*A*C;
        disc = (std::max)(0.0f, disc);

        float t_accel = (-B + std::sqrt(disc)) / (2*A);
        return t + t_accel;
    }

    // Accel phase to max velocity
    float t_accel = ((distance >= 0.0f ? maxMoveSpeed : -maxMoveSpeed) - startingVel) / (maxMoveAccel * (distance >= 0.0f ? 1.0f : -1.0f));

    t += t_accel;
    distance -= d_to_vmax;

    // Cruise phase
    float t_cruise = distance / (distance >= 0.0f ? maxMoveSpeed : -maxMoveSpeed);
    t += t_cruise;

    return t;
}




// if the direction we'll turn towards a point matches CW or CCW
bool FilteredRobot::pointCorrectSide(cv::Point2f point, bool CW, bool forward, float tolerance) {
    return (angleTo(point, forward) > -tolerance && CW) || (angleTo(point, forward) < tolerance && !CW);
}


// how much distance we'd have to get closer to touch another robot
float FilteredRobot::distanceToCollide(FilteredRobot opp) {
    float collisionRad = opp.getSizeRadius() + sizeRadius;
    return (std::max)(distanceTo(opp.position()) - collisionRad, 0.0f);
}




// signed velocity that's actually in the driving direction, positive means it's in the direction of orientation
float FilteredRobot::tangentVel(bool forward) {
    float velAngle = atan2(velFilteredSlow[1], velFilteredSlow[0]);
    float velAngleOffset = angle_wrap(velAngle - posFiltered[2]);
    return moveSpeedSlow() * cos(velAngleOffset) * (forward? 1 : -1);
}


float FilteredRobot::tangentVelFast(bool forward) {
    float velAngle = atan2(velFiltered[1], velFiltered[0]);
    float velAngleOffset = angle_wrap(velAngle - posFiltered[2]);
    return moveSpeed() * cos(velAngleOffset) * (forward? 1 : -1);
}




// calculates time to turn towards point based on known acceleration
float FilteredRobot::pointETAAccel(cv::Point2f target, bool forward, float margin) {

    // how far we need to rotate
    float angleError = angleTo(target, forward);
    if(abs(angleError) < margin) { angleError = 0.0f; }
    else { angleError -= margin * sign(angleError); } // margin always subtracts from error magnitude
    

    float accel = maxTurnAccel * sign(angleError); // assume we accel in the direction we're going to turn
    float dirac = pow(velFilteredSlow[2], 2.0f) + (2.0f * accel * angleError);
    float t1 = (-velFilteredSlow[2] + sqrt(dirac)) / accel;
    float t2 = (-velFilteredSlow[2] - sqrt(dirac)) / accel;

    float time = (std::min)(t1, t2);
    if(t1 < 0.0f) { time = t2; }
    if(t2 < 0.0f) { time = t1; }

    return time;
}



// returns the sign
int FilteredRobot::sign(float num) { return (num < 0) ? -1 : 1; }


// angle to the inputted point from the front of the robot
float FilteredRobot::angleTo(cv::Point2f point, bool forward) {
    float rawAngle = atan2(point.y - posFiltered[1], point.x - posFiltered[0]);
    float offset = forward ? 0.0f : M_PI; // turn by 180 if its backwards
    return angle_wrap(rawAngle - posFiltered[2] + offset);
}


// distance to the inputted point
float FilteredRobot::distanceTo(cv::Point2f point) { return cv::norm(position() - point); }



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

float FilteredRobot::angle(bool forward) { 
    float offset = forward? 0 : M_PI;
    return angle_wrap(posFiltered[2] + offset); 
}

float FilteredRobot::turnVel() { return velFiltered[2]; }
float FilteredRobot::turnVelSlow() { return velFilteredSlow[2]; }
float FilteredRobot::getMaxTurnSpeed() { return maxTurnSpeed; }
float FilteredRobot::getMaxMoveSpeed() { return maxMoveSpeed; }
float FilteredRobot::getMaxMoveAccel() { return maxMoveAccel; }
float FilteredRobot::getMaxTurnAccel() { return maxTurnAccel; }
float FilteredRobot::moveAccel() { return cv::norm(cv::Point2f(accFiltered[0], accFiltered[1])); }
float FilteredRobot::turnAccel() { return accFiltered[2]; }
std::vector<cv::Point2f> FilteredRobot::getPath() { return path; }
float FilteredRobot::getWeaponAngleReach() { return weaponAngleReach; }
float FilteredRobot::getSizeRadius() { return sizeRadius; }
float FilteredRobot::moveSpeed() { return cv::norm(moveVel()); }
float FilteredRobot::moveSpeedSlow() { return cv::norm(moveVelSlow()); }


std::vector<float> FilteredRobot::getPosFiltered() { return posFiltered; }
std::vector<float> FilteredRobot::getVelFiltered() { return velFiltered; }
std::vector<float> FilteredRobot::getAccFiltered() { return accFiltered; }
std::vector<float> FilteredRobot::getVelFilteredSlow() { return velFilteredSlow; }
std::vector<float> FilteredRobot::getVelFilteredUltraSlow() { return velFilteredUltraSlow; }