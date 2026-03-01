
#include "AStarAttack.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../UIWidgets/GraphWidget.h"
#include "../UIWidgets/ClockWidget.h"
#include "../MathUtils.h"
#include <cstdlib>
#include "DisplayUtils.h"
#include "FollowPoint.h"
#include "opencv2/core/check.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/opencv.hpp"
#include <limits>
#include <vector>
#include <wingdi.h>

AStarAttack* AStarAttack::_instance = nullptr;

AStarAttack::AStarAttack()
{
    _instance = this;
    
    // init filters
    orbFiltered = FilteredRobot(1.0f, 40.0f, 430.0f, 800.0f, 
        22.0f, 50.0f, 70.0f*TO_RAD, 20.0f);
    orbVirtual = orbFiltered;

    oppFiltered = FilteredRobot(1.0f, 50.0f, 430.0f, 1200.0f, 
        25.0f, 60.0f, 60.0f*TO_RAD, 28.0f); // weapon = 60
    
    field = Field();

    previousGamepad = {0, 0};
}





DriverStationMessage AStarAttack::Execute(Gamepad &gamepad, double rightStickY)
{
    // track loop time
    static Clock updateClock;
    static ClockWidget processingTimeVisualizer("Orbit");
    processingTimeVisualizer.markStart();

    double deltaTime = updateClock.getElapsedTime(); // reset every loop so elapsed time is loop time
    if(deltaTime == 0) { deltaTime = 0.001; } // broski what
    updateClock.markStart(); // reset for next loop



    // save what this updates inputs were
    inputs = {previousGamepad[0], previousGamepad[1], orbFiltered.tangentVelFast(true), orbFiltered.turnVel(), (float) deltaTime };


    // get odometry data for orb and opp
    OdometryData orbData = RobotController::GetInstance().odometry.Robot();
    OdometryData oppData = RobotController::GetInstance().odometry.Opponent();

    // update filtered positions/velocities and paths
    orbFiltered.updateFilters(deltaTime, orbData.robotPosition, orbData.GetAngle()); orbFiltered.updatePath();
    oppFiltered.updateFilters(deltaTime, oppData.robotPosition, oppData.GetAngle()); 
    

    // what did orb actually end up doing
    trueOutputs = {orbFiltered.tangentVelFast(true), orbFiltered.turnVel()};


    // control orbVirtual and model tuning
    bool autoTune = gamepad.GetLeftBumper() && !gamepad.GetRightBumper();
    bool resetState = gamepad.GetRightBumper();
    bool resetModel = gamepad.GetRightBumper() && gamepad.GetLeftBumper();
    bool displayOrbVirtual = gamepad.GetLeftTrigger() > 0.3f;

    if(displayOrbVirtual || autoTune || resetState || resetModel) {
        controlOrbVirtual(autoTune, resetState, resetModel);
    }





    bool forwardInput = (rightStickY >= 0.0f); // if driver is pushing forward or backward

    std::vector<FollowPoint> follows = {}; // all follow points generated
    std::vector<FollowPoint> followsFocussed = {}; // doesn't show every radius gain

    bool enableForward = false;
    bool enableCW = !gamepad.GetRightBumper();
    bool enableTurnAway = !gamepad.GetLeftBumper();
    std::vector<bool> enable = {enableForward, enableCW, enableTurnAway};


    FollowPoint follow = createFollowPoint(deltaTime, forwardInput, enable, follows, followsFocussed); // generate follow point


    // store the followPoints for debugging/display
    _lastFollowPoints = followsFocussed; 
    // _lastFollowPoints = follows;

    
    


    display(follow, follows, followsFocussed); // display follow point data
    DisplayUtils::displayLines(field.getBoundLines(), cv::Scalar(0, 0, 255)); // display field bound lines




    // follow.approachCurve.followCurve(orbFiltered.position(), 40, true);

    // std::cout << ", orb eta = " << follow.orbETA << ", opp eta = " << follow.oppETA;
    // std::cout << ", turn enforce = " << follow.enforceTurnDirection;
    // std::cout << std::endl;

    

    

    std::vector<float> driveInputs = orbFiltered.curvatureController(follow.driveAngle, 
        rightStickY, deltaTime, follow.forward, follow.enforceTurnDirection);


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];

        
    processingTimeVisualizer.markEnd();


    previousGamepad = {(float) rightStickY, gamepad.GetLeftStickX() }; // save what the gamepad was for next update

    return ret;
}























// displays all the data
void AStarAttack::display(FollowPoint follow, std::vector<FollowPoint> follows, std::vector<FollowPoint> followsFocussed) {

    bool colliding = orbFiltered.colliding(oppFiltered, 10.0f);
    bool facing = orbFiltered.facing(oppFiltered, follow.forward);
    bool oppFacing = oppFiltered.facing(orbFiltered, true);

    bool hitOpp = colliding && facing && !oppFacing;
    bool hitOrb = colliding && oppFacing;



    // colors
    cv::Scalar colorOrb = follow.forward ? cv::Scalar(255, 200, 0) : cv::Scalar(0, 230, 255);
    cv::Scalar colorOrbLight = follow.forward ? cv::Scalar(255, 230, 200) : cv::Scalar(200, 230, 255);

    cv::Scalar colorOpp = cv::Scalar(0, 0, 255);
    cv::Scalar colorOppLight = cv::Scalar(200, 200, 255);



    // turn opp green if we hit them
    if(hitOpp) { 
        colorOpp = cv::Scalar(200, 255, 200); 
        colorOppLight = cv::Scalar(200, 255, 200); 
        DisplayUtils::emote(); // DO NOT DELETE
    }

    // turn orb red if we get hit
    if(hitOrb) {
        colorOrb = cv::Scalar(0, 50, 255);
        colorOrbLight = cv::Scalar(0, 50, 255);
    }


    cv::Scalar wallColor = follow.CW? cv::Scalar(0, 215, 255) : cv::Scalar(0, 165, 255);


    
    // display every orb sim path
    for(int testFollow = 0; testFollow < follows.size(); testFollow++) {
        cv::Scalar color = cv::Scalar{128, 128, 128};
        DisplayUtils::displayPath(follows[testFollow].orbSimPath, color, color, 1);
    }

    


    // display circles
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.point, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRad(orbFiltered.moveSpeedSlow()), colorOrbLight, 2); // draw pp radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRadWall(), colorOrbLight, 1); // draw pp wall radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.orbSimPath.back(), 8, cv::Scalar(255, 255, 255), 5); // highlight the last point of our path to opp


    // display paths
    DisplayUtils::displayPath(follow.orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(follow.oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path
    DisplayUtils::displayPath(orbFiltered.getPath(), cv::Scalar(100, 100, 100), colorOrbLight, 3); // display orb path
    DisplayUtils::displayPath(follow.opp.getPath(), cv::Scalar(200, 200, 200), colorOppLight, 3); // display opp path
    DisplayUtils::displayPath(follow.approachCurve.getCurvePoints(), cv::Scalar(200, 255, 200), cv::Scalar(100, 255, 100), 2); // display approach curve
    DisplayUtils::displayPoints(follow.wallScanPoints, wallColor, wallColor, 2);



    // display text
    std::string forwardStatus = follow.forward ? "Forward" : "Backward"; 
    std::string CWStatus = follow.CW ? "CW" : "CCW";

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Orbiting", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), CWStatus, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);


    // display robot sizes and weapon regions
    orbFiltered.displayRobot(2, colorOrb, colorOrbLight, follow.forward);
    oppFiltered.displayRobot(2, colorOpp, colorOppLight, true);

}




// generates the greatest follow point you've ever seen
// enable order is forward, CW, turnAway
FollowPoint AStarAttack::createFollowPoint(float deltaTime, bool forwardInput, std::vector<bool> enable, std::vector<FollowPoint>& follows, std::vector<FollowPoint>& followsFocussed) {

    follows = {}; // list of every follow point generated
    followsFocussed = {}; // only lists the best radius gain for each type

    // tracks which point is the best
    float lowestScore = 0.0f;
    int lowestIndex = 0;

    // default values for parameters that don't vary
    bool defaultForward = forwardInput;
    bool defaultCW = true;
    bool defaultTurnAway = true;

    // generate every combination of follow point
    for(int forward = 0; forward < 2; forward++) {

        if(!enable[0]) {
            if(forward != 0) { break; }
            forward = defaultForward;
        }

        for(int CW = 0; CW < 2; CW++) {

            if(!enable[1]) {
                if(CW != 0) { break; }
                CW = defaultCW;
            }

            for(int turnAway = 0; turnAway < 2; turnAway++) {

                if(!enable[2]) {
                    if(turnAway != 0) { break; }
                    turnAway = defaultTurnAway;
                }

                // tracks the best point of this set so we don't list every radius combination
                float lowestScoreFocussed = 0.0f;
                int lowestIndexFocussed = 0;

                std::vector<float> endAngles = generateEndAngles(CW); // generate list of collision angles

                // create a point for every end angle
                for(int end = 0; end < endAngles.size(); end++) {

                    // create a point with this config
                    FollowPoint testFollow = FollowPoint(forward, CW, turnAway, endAngles[end], orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius(), oppFiltered);

                    orbToOppPath(testFollow); // generate our path to opp
                    oppToOrbETA(testFollow); // calculate opp's time to turn to us
                    driveAngle(testFollow); // generate drive angle for this point
                    avoidBoundsVector(testFollow); // adjust turn direction to avoid wall if needed
                    followScore(testFollow, forwardInput); // score this point


                    follows.emplace_back(testFollow); // add to the list

                    // total score of this point
                    float score = testFollow.directionScores.back();

                    // save to focussed list if it's the best of it's type
                    if(score < lowestScoreFocussed || end == 0) { 
                        lowestScoreFocussed = score;
                        lowestIndexFocussed = follows.size() - 1;
                    }

                    // save as global best
                    if(score < lowestScore || follows.size() == 1) { 
                        lowestScore = score;
                        lowestIndex = follows.size() - 1;
                    }
                }

                followsFocussed.emplace_back(follows[lowestIndexFocussed]); // add the best point to the focussed list
            }
        }
    }

    if(follows.empty()) { return FollowPoint(); } // no crashy
    return follows[lowestIndex]; // return the global best point
}




// generates the list of absolute angles to try colliding with opp at
std::vector<float> AStarAttack::generateEndAngles(bool CW) {

    int pathsAtCollide = 9;
    int pathsFar = 4;
    float farAway = 400.0f; 
    int totalPathEnds = (int) (pathsAtCollide + (pathsFar - pathsAtCollide)*std::clamp(orbFiltered.distanceToCollide(oppFiltered) / farAway, 0.0f, 1.0f));
    // totalPathEnds = 1;

    float backRegion = 2*(M_PI - oppFiltered.getWeaponAngleReach()); // angular span of opp's back region
    float endSpacing = backRegion / (totalPathEnds + 1); // angle between adjacent path ends


    // build the array of end angles to target
    std::vector<float> endAngles = {};

    for(int end = 0; end < totalPathEnds; end++) {
        float endAngle = angle_wrap(((end + 1) * endSpacing) + oppFiltered.angle(true) + oppFiltered.getWeaponAngleReach()); // end angle for this point

        float upperScanBound = 190*TO_RAD; // 190
        float lowerScanBound = -(endSpacing + 0.01f); // make sure to generate 1 path that goes right to opp

        // only create this point if the end angle is within some range of us
        float angleToOrb = angle(oppFiltered.position(), orbFiltered.position());
        float wrapOffset = M_PI - upperScanBound; if(CW) { wrapOffset *= -1; }
        float radialTravel = angle_wrap(endAngle - angleToOrb - wrapOffset) + wrapOffset;

        if(!CW) { radialTravel *= -1; }

        // add this end to the list if it's in range
        if(radialTravel > lowerScanBound && radialTravel < upperScanBound) {
            endAngles.emplace_back(endAngle);
        }
    }

    // if nothing is in range, add the opp's back
    if(endAngles.empty()) { 
        endAngles.emplace_back(angle_wrap(oppFiltered.angle(false))); 
    }

    return endAngles;
}



// simulates opp assuming he floors it towards the collision point of orb's path
void AStarAttack::oppToOrbETA(FollowPoint &follow) {

    float timeStep = 0.005f; // 0.01 
    float maxTime = 1.0f;
    float simTime = 0.0f; // amount of time elapsed since start of sim
    int steps = (int) (maxTime / timeStep);


    FilteredRobot virtualOpp = oppFiltered; // start simulated opp at opp's state

    std::vector<float> slowVel = virtualOpp.getVelFilteredSlow();
    virtualOpp.setVel({slowVel[0], slowVel[1], slowVel[2]});
    follow.oppSimPath = {}; // reset



    // opp turns in the direction orb starts in unless we pass by opp's front
    float startingAngleToOrb = follow.opp.angleTo(orbFiltered.position(), true); // assume opp rotates towards where we are at the beginning of the sim

    int turnDirection = sign(startingAngleToOrb);
    if(follow.crossesOppFront) { turnDirection = follow.CW? 1 : -1; }



    // simulate actions through each time step
    for(int i = 0; i < steps; i++) { 

        // add current position to the path
        follow.oppSimPath.emplace_back(virtualOpp.position());


        // opp's eta is when orb is first in the weapon region while he's turning the right way
        if(virtualOpp.facingPoint(follow.orbSimPath.back(), true)) { 
            break;
        }


        // opp's behavior is flooring turn towards wherever orb currently is
        std::vector<float> oppDriveInputs = {0.0f, 1.0f * turnDirection};
        // oppDriveInputs = {0.0f, -1.0f};


        std::vector<float> oppModelInputs = {oppDriveInputs[0], oppDriveInputs[1], 0.0f, 0.0f, timeStep};
        virtualOpp.tuneModel(false, oppModelInputs, {});

        // show opp orientations with lines
        // cv::Point2f secondPoint = virtualOpp.position() + 40.0f*cv::Point2f(cos(virtualOpp.angle(true)), sin(virtualOpp.angle(true)));
        // cv::line(RobotController::GetInstance().GetDrawingImage(), virtualOpp.position(), secondPoint, cv::Scalar(0, 120, 255), 1);


        simTime += timeStep;
    }

    follow.oppETA = simTime; 
}





// generates orb's path to opp using orb's model
void AStarAttack::orbToOppPath(FollowPoint &follow) {

    float maxTime = 2.8f; // 1.5
    float simTime = 0.0f; // amount of time elapsed since start of sim
    float timeStep = 0.02f; // 0.02 gets set in loop

    follow.orbSimPath = {};
    follow.orbSimPathTimes = {};

    FilteredRobot virtualOrb = orbFiltered; // start simulated orb at our current state
    virtualOrb.setToSlowVel(); // use slow vel as starting reference

    bool reachedFollowPoint = false; // if we've been aligned with the follow point yet
    bool reachedSpeed = false; // if we've reached a threshold speed yet
    float startingAngle = follow.opp.angleTo(virtualOrb.position(), true); // what side of opp we start on
    follow.worstTimeMargin = 99999.0f; // default

 

    // simulate orb's actions through each time step
    while(simTime < maxTime) {

        // add current position to the path
        follow.orbSimPath.emplace_back(virtualOrb.position()); 
        follow.orbSimPathTimes.emplace_back(simTime);


        // check if we crossed opp's front and flag it
        float currAngle = follow.opp.angleTo(virtualOrb.position(), true);
        if((sign(startingAngle) != sign(currAngle)) && (abs(currAngle - startingAngle) < M_PI)) {
            follow.crossesOppFront = true;
            // std::cout << "crosses = " << follow.crossesOppFront;
        }


        // create the follow point using approach curve
        cv::Point2f virtualFollow = follow.approachCurve.followCurve(virtualOrb.position(), ppRad(ppRad(virtualOrb.moveSpeed())), false);

        // how far off we are from the simulated follow point
        float angleError = virtualOrb.angleTo(virtualFollow, follow.forward); 
    
        // how far off we are form opp
        float angleToOpp = virtualOrb.angleTo(follow.opp.position(), follow.forward); 




        // check what sides of us the follow point is on
        float tolerance = 0.0f*TO_RAD; // 10
        bool oppCorrectSide = virtualOrb.pointCorrectSide(follow.opp.position(), follow.CW, follow.forward, tolerance);
        bool pointCorrectSide = virtualOrb.pointCorrectSide(virtualFollow, follow.CW, follow.forward, tolerance);

        
        // force a turn around if necessary
        float wrapOffset2 = 0;
        if(!oppCorrectSide) {
            if(follow.CW && follow.turnAway) { wrapOffset2 = M_PI; }
            if(!follow.CW && follow.turnAway) { wrapOffset2 = -M_PI; }
    
            if(follow.CW && !follow.turnAway) { wrapOffset2 = -M_PI; }
            if(!follow.CW && !follow.turnAway) { wrapOffset2 = M_PI; }
        }
        
        angleError = angle_wrap(angleError - wrapOffset2) + wrapOffset2;


        // save which direction was enforced at first for the actual follow point
        int enforce = sign(angleError);

        // set real follow point and turn directions to first update of simulator
        if(simTime <= 0.0f) { 
            follow.enforceTurnDirection = enforce;
            follow.point = virtualFollow; 
        }



        if(abs(angleError) < 60.0f*TO_RAD) { reachedFollowPoint = true; } // if we've pointed at the point yet
        if(reachedFollowPoint && virtualOrb.moveSpeed() > 50.0f) { reachedSpeed = true; } // 150 if we've been fast since aligning to the follow point



        // exit conditions occur when we hit the opp
        if(virtualOrb.colliding(follow.opp, 0.0f) && abs(angleError) < 80.0f*TO_RAD && abs(angleToOpp) < 80.0f*TO_RAD) {
            break;
        }


        // save the worst time margin for unsafe points
        float oppETA = follow.opp.collideETASimple(virtualOrb.position(), orbFiltered.getSizeRadius(), true);
        float timeMargin = oppETA - simTime; // how much later the opp will get to current position than we do
        if(timeMargin < follow.worstTimeMargin && !reachedFollowPoint) { follow.worstTimeMargin = timeMargin; }

        

        // drive to the follow point
        float driveAngle = angle(virtualOrb.position(), virtualFollow);


        std::vector<float> driveInputs = virtualOrb.curvatureController(driveAngle, 
            1.0f, timeStep, follow.forward, enforce);


        // timestep is larger if you're further away
        float stepAtCollide = 0.01f;
        float stepAtFar = 0.05f;
        float farAway = 400.0f;
        timeStep = stepAtCollide + (stepAtFar - stepAtCollide)*(std::clamp(virtualOrb.distanceToCollide(follow.opp) / farAway, 0.0f, 1.0f));

        std::vector<float> modelInputs = {driveInputs[0], driveInputs[1], 0.0f, 0.0f, timeStep};
        virtualOrb.tuneModel(false, modelInputs, {});


        simTime += timeStep;
    }

    follow.orbETA = simTime; 
}







// returns angle to drive along by canceling opponents velocity
void AStarAttack::driveAngle(FollowPoint &follow) {

    cv::Point2f oppVel = oppFiltered.moveVelSlow(); // opp's move velocity vector
    float oppSpeed = oppFiltered.moveSpeedSlow(); // how fast the opp is moving in absolute
    float maxCancel = orbFiltered.getMaxMoveSpeed() * 0.9f; // reserve some speed margin
    if(oppSpeed > maxCancel) { oppVel *= abs(maxCancel / oppSpeed); }

    float pointAngle = angle(cv::Point2f(orbFiltered.getPosFiltered()[0], orbFiltered.getPosFiltered()[1]), follow.point); // absolute angle to the point

    float p = oppVel.x*cos(pointAngle) + oppVel.y*sin(pointAngle);
    float s = sqrt(pow(orbFiltered.getMaxMoveSpeed(), 2) - pow(oppVel.x*sin(pointAngle) - oppVel.y*cos(pointAngle), 2));

    float pointSpeed = 0.0f;
    if(s > p) { pointSpeed = -p + s; } // thanks chat, ensures vector sum is always to maxMoveSpeed

    cv::Point2f pointVel = cv::Point2f(pointSpeed*cos(pointAngle), pointSpeed*sin(pointAngle)); // vector in the direction of the followPoint
    cv::Point2f totalVel = oppVel + pointVel; // sum the opp's vel with our desired vel in their frame

    follow.driveAngle = atan2(totalVel.y, totalVel.x); // target angle is in the direction of the total vector
}





void AdjustRadiusWithBumpers() {
    static bool prevRightBumper = false;
    static bool prevLeftBumper = false;
    bool currRightBumper = RobotController::GetInstance().gamepad.GetRightBumper();
    bool currLeftBumper = RobotController::GetInstance().gamepad.GetLeftBumper();
    if (currRightBumper && !prevRightBumper) {
        RADIUS_CURVE_Y1 += 5;
    }
    if (currLeftBumper && !prevLeftBumper) {
        RADIUS_CURVE_Y1 -= 5;
    }

    RADIUS_CURVE_Y1 = fmin(RADIUS_CURVE_Y1, RADIUS_CURVE_Y2 - 10);
    RADIUS_CURVE_Y1 = fmax(RADIUS_CURVE_Y1, 0);

    prevRightBumper = currRightBumper;
    prevLeftBumper = currLeftBumper;
}




// absolute angle made by 2 points
float AStarAttack::angle(cv::Point2f point1, cv::Point2f point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}




// spirals out from our current radius from opp to see how walls will affect us
float AStarAttack::wallScore(FollowPoint &follow) {

    const float maxPathLength = 500.0f; // length of predicted path we'll sweep

    follow.wallScanPoints = {}; // reset just in case
    const float sweepIncrement = follow.CW ? 5.0f*TO_RAD : -5.0f*TO_RAD; // how much to increment sweep angle for each point

    float startAngle = angle(oppFiltered.position(), orbFiltered.position());
    float score = 0.0f; // integrated score of all the points
    float pathLength = 0.0f; // accumlated predicted path length


    cv::Point2f center = field.clipPointInBounds(oppFiltered.position()); // center the sweep around opp's position

    // scan up to a full circle
    for (float sweptAngle = 0; abs(sweptAngle) < 2*M_PI; sweptAngle += sweepIncrement) {

        float currAngle = angle_wrap(startAngle + sweptAngle); // current angle we're scanning at
        float spiralRadius = abs(sweptAngle)*110.0f + oppFiltered.distanceTo(orbFiltered.position()); // radius of predicted spiral here

        // make a line from the center at the current angle at the sprial radius
        cv::Point2f spiralPoint = center + spiralRadius*cv::Point2f(cos(currAngle), sin(currAngle));
        Line scanLine = Line(center, spiralPoint);

        // find if this line intersects a bound
        cv::Point2f intersect = field.boundIntersection(scanLine);

        // if it does intersect
        if(intersect != cv::Point2f(-1.0f, -1.0f)) {

            spiralPoint = intersect; // set to intersected point

            float gapSize = std::max(oppFiltered.distanceTo(spiralPoint) - oppFiltered.getSizeRadius(), 0.0f); // how big is the gap to drive in
            float margin = std::max(gapSize - 2*orbFiltered.getSizeRadius(), 0.01f); // how much margin would we have if we had to drive past this point
            float sweptPercent = pathLength / maxPathLength; // roughly what percentage has been swept so far

            float rangeWeight = cos(-sweptPercent * 0.5f * M_PI);

            score += pow(margin, -0.6f) + rangeWeight; // score function is integrated based on how much gap we have
        }


        // total up the path length
        if(!follow.wallScanPoints.empty()) {
            pathLength += cv::norm(follow.wallScanPoints.back() - spiralPoint);
        }
        
        // add new point to the list
        follow.wallScanPoints.emplace_back(spiralPoint);

        
        // if the points are so far away now then stop counting
        if(pathLength > maxPathLength) { break; }
    }

    score /= pathLength; // normalize to sweep range so we can change that without losing tune
    return score;
}


// returns sign of the input
int AStarAttack::sign(float num) { return (num < 0)? -1 : 1; }





// calculates pure pursuit radius based on speeds
float AStarAttack::ppRad(float speed) {
    float radSlow = ASTAR_PP_RAD_SLOW;
    float radFast = ASTAR_PP_RAD_FAST;
    float speedFast = ASTAR_PP_SPEED_FAST;
    return radSlow + ((radFast - radSlow) / speedFast) * speed;
}


// pp radius used for walls
float AStarAttack::ppRadWall() {
    float radSlow = 40.0f;
    float radFast = 110.0f;
    float speedFast = 400.0f;
    return radSlow + ((radFast - radSlow) / speedFast) * orbFiltered.moveSpeedSlow();
}



// adjusts drive angle if needed to obey pure pursuit radius on walls
void AStarAttack::avoidBoundsVector(FollowPoint &follow) {

    std::vector<cv::Point2f> ppWallPoints = PurePursuit::followPath(orbFiltered.position(), field.getBoundPoints(), ppRadWall());

    // if we're not hitting a wall and in the field don't change the drive angle
    bool insideField = field.insideFieldBounds(orbFiltered.position()); // if orb is inside the field
    if(ppWallPoints.size() == 0 && insideField) { return; }

    cv::Point2f boundPoint = field.closestBoundPoint(orbFiltered.position()); // get the closest point to us thats on a field bound

    // if we're not hitting a wall and fully out of the field then drive back in to closest bound point
    if(ppWallPoints.size() == 0 && !field.insideFieldBounds(orbFiltered.position())) { 
        safe_circle(RobotController::GetInstance().GetDrawingImage(), boundPoint, 8, cv::Scalar(255, 255, 255), 3);
        follow.turnAway = 0; // don't worry about turning away from opp, need to return optimally
        follow.enforceTurnDirection = 0; 
        follow.driveAngle = angle(orbFiltered.position(), boundPoint);
        return;
    }

    // from here we're guarenteed to have pure pursuit intersections with the wall







    // now let's find which pp points actually define the region

    float angleToBound = angle(orbFiltered.position(), boundPoint); // angle to the closest bound point

    // min and max offsets from the line to the bound point
    float mostPosOffset = 0;
    float mostNegOffset = 0;

    // find the most extreme offsets, those will be the outer ones used for pure pursuit
    for(int i = 0; i < ppWallPoints.size(); i++) {
        float pointAngle = angle(orbFiltered.position(), ppWallPoints[i]);
        float offset = angle_wrap(pointAngle - angleToBound); // offset from the line to the center of the field

        if(offset > mostPosOffset) { mostPosOffset = offset; }
        if(offset < mostNegOffset) { mostNegOffset = offset; }
    }

    // angle region in which drive angle must lie
    float minAngle = angle_wrap(angleToBound + mostNegOffset);
    float maxAngle = angle_wrap(angleToBound + mostPosOffset);


    // points at the min and max angles
    cv::Point2f minPoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(minAngle), sin(minAngle));
    cv::Point2f maxPoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(maxAngle), sin(maxAngle));

    safe_circle(RobotController::GetInstance().GetDrawingImage(), minPoint, 8, cv::Scalar(255, 255, 255), 3);
    safe_circle(RobotController::GetInstance().GetDrawingImage(), maxPoint, 8, cv::Scalar(255, 255, 255), 3);









    // now change the drive angle and point if needed
    // don't override follow.point for visualization, doesn't matter atp anyway


    // intersection of pp radius and current orientation, determines if we need to override turn direction enforce
    cv::Point2f drivePoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(follow.driveAngle), sin(follow.driveAngle));


    // if drive point isn't in the field we need to round it to nearest wall point
    if(!field.insideFieldBounds(drivePoint)) {

        int bound = follow.CW? 1 : -1; // 1 = max bound, -1 = min bound
        if(!insideField) { bound *= -1; } // invert if outside the field
        follow.driveAngle = (bound == 1)? maxAngle : minAngle;
    }


    drivePoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(follow.driveAngle), sin(follow.driveAngle));
    // safe_circle(RobotController::GetInstance().GetDrawingImage(), drivePoint, 8, cv::Scalar(255, 0, 255), 3);








    // make sure we turn the correct direction to the drive angle

    float orbOffset = angle_wrap(angleToBound - orbFiltered.angle(follow.forward));
    float driveOffset = angle_wrap(angleToBound - follow.driveAngle);

    // only change the direction enforcement if turning the wrong way would make us go way out of the field
    if(abs(orbOffset) > 5.0f*TO_RAD) { 

        // by default, turn towards the follow point as quick as possible
        float error = orbFiltered.angleTo(drivePoint, follow.forward);
        follow.enforceTurnDirection = sign(error);

        if(orbOffset > 0 && driveOffset < 0) { 
            follow.enforceTurnDirection = insideField? -1 : 1; 
        }

        if(orbOffset < 0 && driveOffset > 0) { 
            follow.enforceTurnDirection = insideField? 1 : -1;
        }
    }
}




// score how good a follow point is
void AStarAttack::followScore(FollowPoint &follow, bool forwardInput) {


    // main score is fraction based on ETAs
    float fraction2 = (follow.orbETA - follow.oppETA) / (follow.orbETA + 0.00001f);
    follow.directionScores.emplace_back(fraction2*100.0f);

    // std::cout << "orb eta = " << follow.orbETA;


    // also penalize very low opp times
    float extra = 9999999.0f;
    if(follow.oppETA > 0.001f) { extra = 0.3f / pow(follow.oppETA, 1.0f); } // 0.25, 1
    follow.directionScores.emplace_back(extra);




    // add score for unsafe points
    float safeScore = 999999.0f;
    if(follow.worstTimeMargin > 0.001f) { safeScore = 0.8f / follow.worstTimeMargin; } // 1.2
    follow.directionScores.emplace_back(safeScore);



    // add penalty for points that might take us near a wall
    float wallGain = 50.0f; // 70
    follow.directionScores.emplace_back(wallScore(follow)*wallGain);
       

    // // how much velocity we already have built up in a given direction, ensures we don't switch to other direction randomly
    // float raw = orbFiltered.tangentVel(follow.forward);
    // float tanVel = pow(abs(raw), 0.5f) * sign(raw);
    // float momentumWeight = -0.08f; // 1.5
    // follow.directionScores.emplace_back(tanVel*momentumWeight);



    // add a penalty for disagreeing with the input direction
    bool directionDisagree = (follow.forward != forwardInput);
    follow.directionScores.emplace_back(1.0f*directionDisagree);


    // sum up the scores and add it as the last entry
    float totalScore = 0.0f;
    for(int i = 0; i < follow.directionScores.size(); i++) { totalScore += follow.directionScores[i]; }
    follow.directionScores.emplace_back(totalScore);
}




// Field boundary editing interface implementation
AStarAttack* AStarAttack::GetInstance() { return _instance; }

const std::vector<FollowPoint>& AStarAttack::GetFollowPoints() const {
    return _lastFollowPoints;
}

// sets field lines to what the UI changed them to
void AStarAttack::SetFieldBoundaryPoints(const std::vector<cv::Point2f>& points) { field.setBoundPoints(points); }

// gets field points for UI
std::vector<cv::Point2f>& AStarAttack::GetFieldBoundaryPoints() { return field.getBoundPoints(); }



// Radius curve parameter interface implementation
void AStarAttack::GetRadiusCurvePoints(float radiusCurveX[4], float radiusCurveY[4]) {
    radiusCurveX[0] = RADIUS_CURVE_X0;
    radiusCurveX[1] = RADIUS_CURVE_X1;
    radiusCurveX[2] = RADIUS_CURVE_X2;
    radiusCurveX[3] = RADIUS_CURVE_X3;
    radiusCurveY[0] = RADIUS_CURVE_Y0;
    radiusCurveY[1] = RADIUS_CURVE_Y1;
    radiusCurveY[2] = RADIUS_CURVE_Y2;
    radiusCurveY[3] = RADIUS_CURVE_Y3;
}



void AStarAttack::SetRadiusCurvePoints(const float radiusCurveX[4], const float radiusCurveY[4]) {
    RADIUS_CURVE_X0 = radiusCurveX[0];
    RADIUS_CURVE_X1 = radiusCurveX[1];
    RADIUS_CURVE_X2 = radiusCurveX[2];
    RADIUS_CURVE_X3 = radiusCurveX[3];
    RADIUS_CURVE_Y0 = radiusCurveY[0];
    RADIUS_CURVE_Y1 = radiusCurveY[1];
    RADIUS_CURVE_Y2 = radiusCurveY[2];
    RADIUS_CURVE_Y3 = radiusCurveY[3];
}



void AStarAttack::ResetRadiusCurveToDefault() {
    RADIUS_CURVE_X0 = 0.0f;   RADIUS_CURVE_Y0 = 0.0f;
    RADIUS_CURVE_X1 = 15.0f;  RADIUS_CURVE_Y1 = 85.0f;
    RADIUS_CURVE_X2 = 100.0f; RADIUS_CURVE_Y2 = 150.0f;
    RADIUS_CURVE_X3 = 200.0f; RADIUS_CURVE_Y3 = 150.0f;
}



void AStarAttack::ResetFieldBoundariesToDefault() {
    field.resetBoundsToDefault();
}



// forces field to regenerate bound lines
void AStarAttack::RegenerateFieldBoundaryLines() { field.generateBoundLines(); }



// controls everything with orbVirtual
void AStarAttack::controlOrbVirtual(bool autoTune, bool resetState, bool resetModel) {

    // run the virtual orb model, tune if wanted
    orbVirtual.tuneModel(autoTune, inputs, trueOutputs);

    if(resetState) {
        orbVirtual.setPos(orbFiltered.getPosFiltered());
        orbVirtual.setVel(orbFiltered.getVelFiltered());
        // std::cout << "reset state";
    }

    if(resetModel) {
        orbVirtual = orbFiltered;
        // std::cout << "reset model";
    }


    // // make virtual orb go towards opp constantly
    // std::vector<float> orbVirtualInputs = orbVirtual.curvatureController(angle(orbVirtual.position(), oppFiltered.position()), 1.0f, deltaTime, true, 0);
    // std::vector<float> modelInputs = {orbVirtualInputs[0], orbVirtualInputs[1], 0.0f, 0.0f, (float) deltaTime};

    // // run orbvirtual to the opponent
    // orbVirtual.tuneModel(false, modelInputs, {});



    cv::Point2f secondPoint = orbVirtual.position() + 40.0f*cv::Point2f(cos(orbVirtual.angle(true)), sin(orbVirtual.angle(true)));
    cv::line(RobotController::GetInstance().GetDrawingImage(), orbVirtual.position(), secondPoint, cv::Scalar(0, 200, 255), 3);
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbVirtual.position(), 5, cv::Scalar(0, 127, 255), 3);
    
    if(autoTune) { std::cout << "auto tuning, "; }
    if(resetModel) { std::cout << "reset model, "; }
    if(resetState) { std::cout << "reset state, "; }
    orbVirtual.printModel();
}