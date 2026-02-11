
#include "AStarAttack.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../UIWidgets/GraphWidget.h"
#include "../UIWidgets/ClockWidget.h"
#include "../MathUtils.h"
#include <cstdlib>
#include "DisplayUtils.h"
#include "FollowPoint.h"
#include "opencv2/core/types.hpp"
#include <limits>
#include <wingdi.h>

AStarAttack* AStarAttack::_instance = nullptr;

AStarAttack::AStarAttack()
{
    _instance = this;
    
    // init filters
    // orbFiltered = FilteredRobot(1.0f, 50.0f, 400.0f, 500.0f, 
    //     22.0f, 50.0f, 50.0f*TO_RAD, 20.0f);
    orbFiltered = FilteredRobot(1.0f, 40.0f, 430.0f, 800.0f, 
        22.0f, 50.0f, 70.0f*TO_RAD, 20.0f);
    oppFiltered = FilteredRobot(1.0f, 50.0f, 430.0f, 1200.0f, 
        25.0f, 60.0f, 60.0f*TO_RAD, 28.0f); // weapon = 60
    orbVirtual = orbFiltered;

    // init field
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
    std::vector<float> trueOutputs = {orbFiltered.tangentVelFast(true), orbFiltered.turnVel() };



    // run the virtual orb model, tune if wanted
    // orbVirtual.tuneModel(gamepad.GetLeftBumper(), inputs, trueOutputs);


    if(gamepad.GetRightBumper()) {
        orbVirtual.setPos(orbFiltered.getPosFiltered());
        orbVirtual.setVel(orbFiltered.getVelFiltered());
    }

    if(gamepad.GetRightBumper() && gamepad.GetLeftBumper()) {
        orbVirtual = orbFiltered;
    }


    std::vector<float> driveInputs2 = orbVirtual.curvatureController(angle(orbVirtual.position(), oppFiltered.position()), 1.0f, deltaTime, true, 0);
    std::vector<float> modelInputs = {driveInputs2[0], driveInputs2[1], 0.0f, 0.0f, (float) deltaTime};

    // run orbvirtual to the opponent
    orbVirtual.tuneModel(false, modelInputs, {});


    // raw follow points in every direction
    // std::vector<bool> pointsCW = {true, true, false, false};
    // std::vector<bool> pointsForward = {true, false, true, false};
    // std::vector<bool> pointsCW = {true, false};
    // std::vector<bool> pointsForward = {false, false};



    std::vector<bool> pointsCW = {true, true, false, false};
    std::vector<bool> pointsTurnAway = {true, false, true, false};
    std::vector<bool> pointsForward = {true, true, true, true};

    // pointsCW = {true};
    // pointsTurnAway = {false};
    // pointsForward = {true};

    // pointsCW = {true, false};
    // pointsTurnAway = {false, false};
    // pointsForward = {true, true};

    // pointsCW = {true, true};
    // pointsTurnAway = {true, false};
    // pointsForward = {true, true};
    





    
    // bumpers can enforce a single follow point
    if(gamepad.GetRightBumper()) {
        pointsCW = {true};
        pointsTurnAway = {false};
        pointsForward = {true};
    }
    if(gamepad.GetLeftBumper()) {
        pointsCW = {true, false};
        pointsTurnAway = {false, false};
        pointsForward = {true, true};

        pointsCW = {true};
        pointsTurnAway = {true};
        pointsForward = {true};
    }








    // generate every possible follow point
    std::vector<FollowPoint> follows = {};
    
    for(int i = 0; i < pointsCW.size(); i++) {

        FollowPoint follow = createFollowPoint(pointsCW[i], pointsForward[i], pointsTurnAway[i], deltaTime);
        driveAngle(follow);

        avoidBoundsVector(follow);
        follows.emplace_back(follow);    
    }


    bool forwardInput = (rightStickY >= 0.0f); // if driver is pushing forward or backward
    FollowPoint follow = chooseBestPoint(follows, forwardInput); // pick which point to use
    
    // store the followPoints for debugging/display (after scores have been computed)
    _lastFollowPoints = follows;


    
    
    display(follow); // display data

    


    // display virtual orb position
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbVirtual.position(), 5, cv::Scalar(0, 127, 255), 3);
    cv::Point2f secondPoint = orbVirtual.position() + 50.0f*cv::Point2f(cos(orbVirtual.angle(true)), sin(orbVirtual.angle(true)));
    cv::line(RobotController::GetInstance().GetDrawingImage(), orbVirtual.position(), secondPoint, cv::Scalar(0, 127, 255), 3);
    // orbVirtual.printModel();



    std::vector<float> driveInputs = orbFiltered.curvatureController(follow.driveAngle, 
        rightStickY, deltaTime, follow.forward, follow.enforceTurnDirection);



    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];


    // save what the gamepad was
    previousGamepad = {(float) rightStickY, gamepad.GetLeftStickX() };


    std::cout << "orb eta = " << follow.orbETA << ", opp eta = " << follow.oppETA;
    std::cout << ", opp to orb accel = " << oppFiltered.collideETASimple(orbFiltered.position(), orbFiltered.getSizeRadius(), true);

    std::cout << std::endl;
    
    
    processingTimeVisualizer.markEnd();
    return ret;
}























// displays all the data
void AStarAttack::display(FollowPoint follow) {

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


    


    // display circles
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.point, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    // safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), follow.radius, cv::Scalar(200, 200, 255), 2); // draw radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRad(orbFiltered.moveSpeedSlow()), colorOrbLight, 2); // draw pp radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRadWall(), colorOrbLight, 1); // draw pp wall radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.opp.position(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppFiltered.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.orbSimPath.back(), 10, cv::Scalar(255, 255, 255), 5);
    // safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.orbSimPath[follow.inflectIndex], 6, cv::Scalar(0, 255, 0), 5); // bold the inflect point of the path we chose


    // display paths
    DisplayUtils::displayPath(follow.orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(follow.oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path
    DisplayUtils::displayLines(field.getBoundLines(), cv::Scalar(0, 0, 255)); // draw field bound lines
    DisplayUtils::displayPath(orbFiltered.getPath(), cv::Scalar(100, 100, 100), colorOrbLight, 3); // display orb path
    DisplayUtils::displayPath(follow.opp.getPath(), cv::Scalar(200, 200, 200), colorOppLight, 3); // display opp path
    DisplayUtils::displayPath(follow.escapePath, cv::Scalar(0, 255, 0), cv::Scalar(255, 255,255), 2); // show the escape path


    // display lines
    cv::Point2f oppOrientationEnd = cv::Point2f(oppFiltered.position().x + oppFiltered.getSizeRadius()*cos(oppFiltered.angle(true)), oppFiltered.position().y + oppFiltered.getSizeRadius()*sin(oppFiltered.angle(true)));
    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppOrientationEnd, colorOpp, 2);


    // display text
    std::string forwardStatus = follow.forward ? "Forward" : "Backward"; 
    std::string CWStatus = follow.CW ? "CW" : "CCW";

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Orbiting", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), CWStatus, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);




    // add lines to show weapon regions
    cv::Point2f oppWeapon1 = oppFiltered.position() + oppFiltered.getSizeRadius()*cv::Point2f(cos(oppFiltered.angle(true) + oppFiltered.getWeaponAngleReach()), sin(oppFiltered.angle(true) + oppFiltered.getWeaponAngleReach()));
    cv::Point2f oppWeapon2 = oppFiltered.position() + oppFiltered.getSizeRadius()*cv::Point2f(cos(oppFiltered.angle(true) - oppFiltered.getWeaponAngleReach()), sin(oppFiltered.angle(true) - oppFiltered.getWeaponAngleReach()));
    
    cv::Point2f orbWeapon1 = orbFiltered.position() + orbFiltered.getSizeRadius()*cv::Point2f(cos(orbFiltered.angle(follow.forward) + orbFiltered.getWeaponAngleReach()), sin(orbFiltered.angle(follow.forward) + orbFiltered.getWeaponAngleReach()));
    cv::Point2f orbWeapon2 = orbFiltered.position() + orbFiltered.getSizeRadius()*cv::Point2f(cos(orbFiltered.angle(follow.forward) - orbFiltered.getWeaponAngleReach()), sin(orbFiltered.angle(follow.forward) - orbFiltered.getWeaponAngleReach()));

    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppWeapon1, colorOppLight, 2);
    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppWeapon2, colorOppLight, 2);

    cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbWeapon1, colorOrbLight, 2);
    cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbWeapon2, colorOrbLight, 2);

}




// makes a valid point
FollowPoint AStarAttack::createFollowPoint(bool CW, bool forward, bool turnAway, float deltaTime) {

    // extrapolate the opp based on this point's parameters
    std::vector<cv::Point2f> oppSimPath = {};
    FilteredRobot targetOpp = orbFiltered.createVirtualOpp(oppFiltered, forward, 
        CW, turnAway, 0.0f, oppSimPath); // 0.25


    // create follow point based on the extrapolated opp's and the given parameters
    FollowPoint follow = FollowPoint(forward, CW, turnAway, targetOpp, oppSimPath);
    

    // set the radius
    radiusEquation(follow);


    // float startingAngle = angle(follow.opp.position(), orbFiltered.position());
    // float endingAngle = oppFiltered.angle(false);
    // float remainingAngle = (endingAngle - startingAngle)*(follow.CW? 1 : -1);
    // float wrapOffset = 90.0f*TO_RAD; wrapOffset *= follow.CW? 1 : -1;
    // remainingAngle = angle_wrap(remainingAngle - wrapOffset) + wrapOffset;

    // // std::vector<cv::Point2f> approach = approachCurve(follow.opp.position(), follow.CW, follow.opp.angle(false));
    DisplayUtils::displayPoints(follow.approach, cv::Scalar{255, 255, 0}, cv::Scalar{255, 255, 255}, 2);


    // follow.point = followApproachCurve(orbFiltered.position(), follow, 60.0f);


    follow.point = cv::Point2f(0, 0); //follow.opp.position();
    int indexToUse = 0;
    if(follow.orbSimFollowPoints.size() > indexToUse) {
        follow.point = follow.orbSimFollowPoints[indexToUse];
    }


    return follow;
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



    // simulate actions through each time step
    for(int i = 0; i < steps; i++) { 

        // add current position to the path
        follow.oppSimPath.emplace_back(virtualOpp.position());



        float angleToOrb = virtualOpp.angleTo(follow.orbSimPath.back(), true);
        bool turningRightWay = sign(angleToOrb) == sign(virtualOpp.turnVel());


        // opp's eta is when orb is first in the weapon region while he's turning the right way
        if(virtualOpp.facingPoint(follow.orbSimPath.back(), true) && turningRightWay) { 
            break;
        }


        // opp's behavior is flooring turn towards wherever orb currently is
        std::vector<float> oppDriveInputs = {0.0f, 1.0f * sign(angleToOrb)};

        std::vector<float> oppModelInputs = {oppDriveInputs[0], oppDriveInputs[1], 0.0f, 0.0f, timeStep};
        virtualOpp.tuneModel(false, oppModelInputs, {});


        simTime += timeStep;
    }

    follow.oppETA = simTime; 
}



// generates a pure pursuit point to follow the approach curve
cv::Point2f AStarAttack::followApproachCurve(cv::Point2f currPosition, FollowPoint follow, float ppRad) {

    if(follow.approach.size() == 0) { return oppFiltered.position(); } // no crashy

    cv::Point2f point = follow.opp.position(); // default, will get set to the right point


    float angleHere = angle(follow.opp.position(), currPosition);

    float wrapOffset = 90.0f*TO_RAD; wrapOffset *= follow.CW? 1 : -1;
    float offsetAngle = angle_wrap(follow.endingAngle - angleHere - wrapOffset) + wrapOffset;


    // target the opp directly if you're wrapping around
    if(offsetAngle*(follow.CW? 1 : -1) < 0) { return point; }


    float radiusHere = approachRadiusEquation(offsetAngle);
    float distanceFromCenter = cv::norm(currPosition - follow.opp.position());
    bool outsideApproach = distanceFromCenter > radiusHere;

    // std::cout << "outside = " << outsideApproach;



    // if we're outside the curve, go to the tangent point
    if(outsideApproach) {
        float mostTangentAngle = 0;

        for(int i = 0; i < follow.approach.size(); i++) {
            float angleToPoint = angle(currPosition, follow.approach[i]);
            float offset = angle_wrap(angleToPoint - mostTangentAngle);
            offset *= follow.CW? 1 : -1;

            // set the point to the most tangent point
            if(offset < 0 || i == 0) { 
                mostTangentAngle = angleToPoint;
                point = follow.approach[i];
            }
        }

        // tangent point is good to use as long as it's further away than the pp rad
        if(cv::norm(point - currPosition) > ppRad) { return point; }
    }


    // set the point to the pp point, will be compared against if inside the curve
    point = approachPP(currPosition, follow, ppRad);


    // if we're inside the curve, so drive out at specified angle
    if(!outsideApproach) {

        // angles are offset relative to line that starts at opp and connects to orb
        float angleAtRadius = 90.0f*TO_RAD; // 90
        float angleAtCollision = 30.0f*TO_RAD; // 40


        // drive angle is linearly interpolated from collision angle to at the radius
        float collisionRadius = follow.opp.getSizeRadius() + orbFiltered.getSizeRadius();
        float distanceAway = std::max(follow.opp.distanceTo(currPosition) - collisionRadius, 0.0f);
        float radiusPercent = std::clamp(distanceAway / (radiusHere - collisionRadius), 0.0f, 1.0f);
        float driveAngle = angleAtCollision + radiusPercent*(angleAtRadius - angleAtCollision);




        // clip the drive angle if the pp angle is more extreme
        if(cv::norm(currPosition - follow.opp.position()) > radiusHere - ppRad) {

            float angleToPPGlobal = angle(currPosition, point);
            float angleToPP = angle_wrap(angleToPPGlobal - angleHere); // angle to pp point in opp's coordinates

            driveAngle = std::max(driveAngle, abs(angleToPP));
        }

        
        driveAngle = angle_wrap(angleHere + driveAngle*(follow.CW? 1 : -1));
        

        // safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 2, cv::Scalar(255, 0, 255), 5);

        // generate a point at the pp rad at the correct angle
        point = currPosition + ppRad*cv::Point2f(cos(driveAngle), sin(driveAngle));

    }

    return point;
}



// finds pure pursuit solution to follow the approach curve
cv::Point2f AStarAttack::approachPP(cv::Point2f currPosition, FollowPoint follow, float ppRad) {

    float closestDistance = 99999.0f;
    int closestIndex = 0;

    // list is ordered, so first point found in the pp rad is the one to follow
    for(int i = 0; i < follow.approach.size(); i++) {

        float distance = cv::norm(follow.approach[i] - currPosition);

        // first point to be in the pp rad is used
        if(distance < ppRad) { 
            if(i == 0) { return follow.opp.position(); }
            return follow.approach[i]; 
        }

        if(distance < closestDistance) {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    return follow.approach[closestIndex];
}



// defines the radius of the approach curve as a function of offset angle
float AStarAttack::approachRadiusEquation(float offsetAngle) {

    float collisionRad = oppFiltered.getSizeRadius() + orbFiltered.getSizeRadius();
    float angleSpan = 120.0f*TO_RAD; // 130
    float maxRad = 120.0f;
    float radius = collisionRad + pow(abs(offsetAngle) / angleSpan, 0.4f) * (maxRad - collisionRad);
    radius = std::clamp(radius, 0.0f, maxRad);

    return radius;
}


// function that defines how we approach opp
void AStarAttack::approachCurve(FollowPoint &follow) {

    float angleIncrement = 5.0f*TO_RAD;
    float offsetAngle = 0.0f;

    follow.approach = {};

    // sweep up to the the specified range for this follow point
    while(abs(offsetAngle) < follow.simRadGain) {

        float radius = approachRadiusEquation(offsetAngle);

        float currAngle = angle_wrap(follow.endingAngle + offsetAngle);
        cv::Point2f newPoint = follow.opp.position() + radius*cv::Point2f(cos(currAngle), sin(currAngle));
        follow.approach.emplace_back(newPoint);

        offsetAngle += angleIncrement * (follow.CW? -1 : 1); 
    }
}



// generates orb's path to opp using orb's model
void AStarAttack::orbToOppPath(FollowPoint &follow) {

    float timeStep = 0.022f; // 0.01 
    float maxTime = 2.0f; // 1.5
    float simTime = 0.0f; // amount of time elapsed since start of sim
    int steps = (int) (maxTime / timeStep);

    follow.inflectIndex = 0;
    follow.orbSimPath = {};
    follow.orbSimPathTimes = {};
    follow.orbSimFollowPoints = {};

    FilteredRobot virtualOrb = orbFiltered; // start simulated orb at our current state
    std::vector<float> slowVel = virtualOrb.getVelFilteredSlow();
    virtualOrb.setVel({slowVel[0], slowVel[1], slowVel[2]});


    float startingAngle = angle(follow.opp.position(), virtualOrb.position());
    follow.endingAngle = angle_wrap(startingAngle + follow.simRadGain*(follow.CW? 1 : -1));
    approachCurve(follow);
    
    // DisplayUtils::displayPoints(approach, cv::Scalar{255, 255, 0}, cv::Scalar{255, 255, 255}, 2);

    bool reachedFollowPoint = false; // if we've been aligned with the follow point yet
    bool reachedSpeed = false; // if we've reached a threshold speed yet

 

    // simulate orb's actions through each time step
    for(int i = 0; i < steps; i++) { 

        // add current position to the path
        follow.orbSimPath.emplace_back(virtualOrb.position()); 
        follow.orbSimPathTimes.emplace_back(simTime);

        




        // create the follow point using approach curve
        cv::Point2f virtualFollow = followApproachCurve(virtualOrb.position(), follow, ppRad(virtualOrb.moveSpeed())); // ppRad(virtualOrb.moveSpeed())
        follow.orbSimFollowPoints.emplace_back(virtualFollow);


        // how far off we are from the simulated follow point
        float angleError = virtualOrb.angleTo(virtualFollow, follow.forward); 
        float angleErrorRaw = angleError;




        // check what sides of us the follow point is on
        float tolerance = 0.0f*TO_RAD; // 10
        bool oppCorrectSide = virtualOrb.pointCorrectSide(follow.opp.position(), follow.CW, follow.forward, tolerance);
        bool pointCorrectSide = virtualOrb.pointCorrectSide(virtualFollow, follow.CW, follow.forward, tolerance);

        
        // force a turn around if necessary
        float wrapOffset2 = 0;
        if(follow.CW && !oppCorrectSide && follow.turnAway) { wrapOffset2 = M_PI; }
        if(!follow.CW && !oppCorrectSide && follow.turnAway) { wrapOffset2 = -M_PI; }

        if(follow.CW && !oppCorrectSide && !follow.turnAway) { wrapOffset2 = -M_PI; }
        if(!follow.CW && !oppCorrectSide && !follow.turnAway) { wrapOffset2 = M_PI; }

        angleError = angle_wrap(angleError - wrapOffset2) + wrapOffset2;


        //oppCorrectSide || follow.turnAway
        if(abs(angleErrorRaw) < 40.0f*TO_RAD) { reachedFollowPoint = true; } // if we've pointed at the point yet
        if(reachedFollowPoint && virtualOrb.moveSpeed() > 50.0f) { reachedSpeed = true; } // 150 if we've been fast since aligning to the follow point

        bool safe = reachedFollowPoint; // && reachedSpeed;
              

        float oppETA = follow.opp.collideETASimple(virtualOrb.position(), orbFiltered.getSizeRadius(), true);
        float timeMargin = oppETA - simTime; // how much later the opp will get to current position than we do
        float timeFraction = (oppETA - simTime) / oppETA;

        if(timeMargin < 0.0f && !safe) { // 0.35
        // if(timeFraction < 0.5f && !safe) { // 0.35
            std::cout << "hitting";
            simTime = maxTime;
            break;
        }


        // exit conditions occur when we hit the opp
        if(virtualOrb.colliding(follow.opp, 0.0f)) {

            // if we're hitting them in a valid way, return the actual sim time
            // if(virtualOrb.facing(follow.opp, follow.forward) && turningCorrect) { break; }
            // if(anglePercent > 1.0f && virtualOrb.facing(follow.opp, follow.forward)) { break; }
            if(virtualOrb.facing(follow.opp, follow.forward) && virtualOrb.moveSpeed() > 100.0f) { break; }

            // otherwise we're hitting them in an invalid way, so return max time
            simTime = maxTime*99;
            break;
        }

        

        
        // drive to the follow point
        float driveAngle = angle(virtualOrb.position(), virtualFollow);

        // save which direction was enforced at first for the actual follow point
        int enforce = sign(angleError);
        if(i == 0) { follow.enforceTurnDirection = enforce; }

        std::vector<float> driveInputs = virtualOrb.curvatureController(driveAngle, 
            1.0f, timeStep, follow.forward, enforce);

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



// if we ever directly target the opp, enforce that we keep targetting them for a minimum time
void AStarAttack::commitToTarget(FollowPoint &follow, double deltaTime, float targetTime) {

    static bool enforceTarget = false; // if we're currently running the clock to force target the opp
    static float timeTargetting = 0.0f; // how long we've been force targetting the opp

    // if the follow point is basically right on the opp, turn on force targeting and reset the clock
    if(oppFiltered.distanceTo(follow.point) < oppFiltered.getSizeRadius() * 0.2f) { 
        enforceTarget = true;
        timeTargetting = 0.0f;
    }

    // if we've been targetting for long enough, don't enforce the target anymore and reset clock
    if(timeTargetting > targetTime) { 
        enforceTarget = false;
        timeTargetting = 0.0f;
    }

    // if we're currently enforcing target, then set followPoint and increment the clock
    if(enforceTarget) { 
        follow.point = oppFiltered.position(); 
        timeTargetting += deltaTime;
    }

    // std::cout << "timeTargetting = " << timeTargetting << "     " << std::endl;
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



// equation for determining size of tangent circle
void AStarAttack::radiusEquation(FollowPoint &follow) {

    // calculate times for orb and opp
    float bestFraction = -1;

    std::vector<float> radGains = { 100.0f, 230.0f, 1000.0f};
    radGains = {20.0f*TO_RAD, 70.0f*TO_RAD,100.0f*TO_RAD, 130.0f*TO_RAD};
    radGains = {0, 10.0f*TO_RAD, 30.0f*TO_RAD, 50.0f*TO_RAD,70.0f*TO_RAD, 90.0f*TO_RAD,130.0f*TO_RAD};
    // radGains = {180.0f*TO_RAD};


    // what in the world
    for(int i = 0; i < radGains.size(); i++) {

        // generate a path for the given sim settings
        FollowPoint testFollow = follow;
        testFollow.simRadGain = radGains[i];

        orbToOppPath(testFollow);
        oppToOrbETA(testFollow);

        


        float fraction2 = 0.0f;
        if(testFollow.orbETA > 0.01f) { fraction2 = std::max((testFollow.orbETA - testFollow.oppETA) / (testFollow.orbETA), 0.0f); }
        // if(testFollow.orbETA + testFollow.oppETA > 0.01f) { fraction2 = std::max((testFollow.orbETA - testFollow.oppETA) / (testFollow.orbETA + testFollow.oppETA), 0.0f); }


        // AdjustRadiusWithBumpers(); // let driver adjust aggressiveness with bumpers

        // input fraction to output radius using RobotConfig variables
        std::vector<cv::Point2f> radiusCurve = {
            cv::Point2f(RADIUS_CURVE_X0, RADIUS_CURVE_Y0),
            cv::Point2f(RADIUS_CURVE_X1, RADIUS_CURVE_Y1),
            cv::Point2f(RADIUS_CURVE_X2, RADIUS_CURVE_Y2),
            cv::Point2f(RADIUS_CURVE_X3, RADIUS_CURVE_Y3)
        };


        testFollow.radius = piecewise(radiusCurve, fraction2);


        if(fraction2 < bestFraction || i == 0) { 
            follow = testFollow;
            bestFraction = fraction2;
        }

        // set the actual follow point to the best result we get
        // solutions need to be below threshold to be valid
        // if((fraction2 < bestFraction || bestFraction == -1) && fraction2 < 1.0f) { 
        //     follow = testFollow;
        //     bestFraction = fraction2;
        // }

        // if we're on the last point and we haven't found a good point yet, use it bc it's the most conservative
        // if(bestFraction == -1 && i == radGains.size() - 1) {
        //     std::cout << "last one";
        //     follow = testFollow;
        // }

        // display all paths with thin lines
        DisplayUtils::displayPath(testFollow.orbSimPath, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), 1);
    }
}


// evalutates a piecewise function defined by the point array
float AStarAttack::piecewise(std::vector<cv::Point2f> points, float x) {

    // Clamp below
    if (x <= points.front().x) { return points.front().y; }

    // Clamp above
    if (x >= points.back().x) { return points.back().y; }

    // Interpolate between points
    for (int i = 0; i < points.size() - 1; i++) {
        cv::Point2f p0 = points[i];
        cv::Point2f p1 = points[i + 1];

        if (x >= p0.x && x < p1.x) {
            float percent = (x - p0.x) / (p1.x - p0.x);
            return p0.y + percent * (p1.y - p0.y);
        }
    }

    return 0.0f; // fallback
}


// returns the tangent point of a circle with set radius around the opponent
cv::Point2f AStarAttack::tangentPoint(float radius, cv::Point2f center, cv::Point2f point, bool CW) {

    float d = cv::norm(center - point);

    // tangent doesn't exist if point is inside the circle
    if(d < radius) { return center; }
    
    // epic math
    float theta = acos(radius / d);
    float alpha = atan2(point.y - center.y, point.x - center.x);
    float theta2 = angle_wrap(alpha + (CW? 1 : -1)*theta);

    return center + radius*(cv::Point2f(cos(theta2), sin(theta2)));
}


// returns intersection of pp radius with circle of set radius around opponent in the correction direction
cv::Point2f AStarAttack::ppPoint(FollowPoint follow) {

    std::vector<cv::Point2f> circle = arcPointsFromCenter(follow.radius, 2*M_PI, 5.0f);
    transformList(circle, oppFiltered.position(), 0.0f);

    // find the greatest or least index point in the pp radius, thats the one to follow
    int defaultIndex = -1;
    int followIndex = defaultIndex;

    for(int i = 0; i < circle.size(); i++) {
        if(cv::norm(circle[i] - orbFiltered.position()) < ppRad(orbFiltered.moveSpeedSlow())) {

            int indexDistance = i - followIndex;
            if(indexDistance > circle.size()/2.0f) { indexDistance -= circle.size(); }

            if((indexDistance > 0 && follow.CW) || (indexDistance < 0 && !follow.CW) || followIndex == defaultIndex) {
                followIndex = i;
            }
        }
    }

    if(followIndex == defaultIndex) { followIndex = 0; } // no crashy
    return circle[followIndex];
}



// generates a list of arc points from the center
std::vector<cv::Point2f> AStarAttack::arcPointsFromCenter(float radius, float angle, float pointSpacing) {

    float arcLength = radius * angle;
    float thetaIncrement = pointSpacing / radius;
    int increments = (int) (angle / thetaIncrement);

    std::vector<cv::Point2f> points;

    for (int i = 0; i < increments; i++) {
        float currentAngle = i * thetaIncrement;
        float currentX = radius*cos(currentAngle);
        float currentY = radius*sin(currentAngle);
        points.emplace_back(cv::Point2f(currentX, currentY));
    }

    // make sure the exact final point is added
    points.emplace_back(cv::Point2f(radius*cos(angle), radius*sin(angle)));
    return points;
}


// moves a list of points to the input point and angle, rewrites to the existing list
void AStarAttack::transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle) {

    for(int i = 0; i < list.size(); i++) {

        cv::Point2f currentPoint = list[i];
        float newX = startPoint.x + currentPoint.x*cos(angle) - currentPoint.y*sin(angle);
        float newY = startPoint.y + currentPoint.x*sin(angle) + currentPoint.y*cos(angle);

        list[i] = cv::Point2f(newX, newY);
    }
}



// absolute angle made by 2 points
float AStarAttack::angle(cv::Point2f point1, cv::Point2f point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}



// distance to the closest line from the list
std::pair<float, int> AStarAttack::closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point) {

    float closest = 999999999.9f; // will always be further away than anything else
    int index = -1;

    // check the whole list and record closest distance
    for (int i = 0; i < lineList.size(); i++) {
        float distance = lineList[i].howClosePoint(point);
        if (distance < closest) { 
            closest = distance; 
            index = i;
        }
    }
    return std::pair<float, int>(closest, index);
}



// does a point list include this point, returns -1 if not found
int AStarAttack::vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint) {

    for(int i = 0; i < pointList.size(); i++) {
        if(pointList[i] == testPoint) { return i; }
    }
    return -1;
}



// spirals out from our current radius from opp to see how walls will affect us
float AStarAttack::wallScore(FollowPoint follow) {

    const float maxPathLength = 500.0f; // length of predicted path we'll sweep

    std::vector<cv::Point2f> scanPoints;
    std::vector<Line> scanLines = {};
    const float sweepIncrement = follow.CW ? 1.0f*TO_RAD : -1.0f*TO_RAD; // how much to increment sweep angle for each point

    float startAngle = angle(oppFiltered.position(), orbFiltered.position());


    float score = 0.0f; // integrated score of all the points
    float pathLength = 0.0f; // accumlated predicted path length

    // scan up to a full circle
    for (float sweptAngle = 0; abs(sweptAngle) < 2*M_PI; sweptAngle += sweepIncrement) {

        float currAngle = angle_wrap(startAngle + sweptAngle); // current angle we're scanning at

        cv::Point2f testPoint = field.clipPointInBounds(oppFiltered.position()); // point to ray cast
        constexpr int kMaxRaycastSteps = 400;


        // increment the point outwards until it's out of bounds or further than our spiral
        for(int i = 0; i < kMaxRaycastSteps; i++) {

            testPoint += 5.0f * cv::Point2f(cos(currAngle), sin(currAngle));

            // if the ray cast has exited the field
            if(!field.insideFieldBounds(testPoint)) {
                testPoint = field.closestBoundPoint(testPoint); // raycasted point might lie slightly outside, so make sure to re-clip after

                float gapSize = std::max(oppFiltered.distanceTo(testPoint) - oppFiltered.getSizeRadius(), 0.0f); // how big is the gap to drive in
                float margin = std::max(gapSize - 2*orbFiltered.getSizeRadius(), 0.01f); // how much margin would we have if we had to drive past this point
                float sweptPercent = pathLength / maxPathLength; // roughly what percentage has been swept so far

                float rangeWeight = cos(-sweptPercent * 0.5f * M_PI);

                score += pow(margin, -0.6f) + rangeWeight; // score function is integrated based on how much gap we have

                break;
            }

            float spiralRadius = abs(sweptAngle)*110.0f + oppFiltered.distanceTo(orbFiltered.position()); // slope of predicted spiral
            
            // if the point has gotten to the spiral, don't add to score
            if(oppFiltered.distanceTo(testPoint) > spiralRadius) { break; }
        }

        // total up the path length
        if(!scanPoints.empty()) {
            pathLength += cv::norm(scanPoints.back() - testPoint);
        }
        
        
        scanPoints.emplace_back(testPoint); // add to the list to display later
        scanLines.emplace_back(Line(oppFiltered.position(), testPoint));

        

        // display scanned points
        cv::Scalar color = cv::Scalar(150, 255, 150);
        if(!follow.CW) { color = cv::Scalar(150, 150, 255); }

        safe_circle(RobotController::GetInstance().GetDrawingImage(), testPoint, 2, color, 2);


        // // if we have at least 2 points start drawing lines
        // if(scanPoints.size() > 1) {
        //     cv::line(RobotController::GetInstance().GetDrawingImage(), scanPoints[scanPoints.size() - 2], testPoint, color, 2);
        // }

        

        // if the points are so far away now then stop counting
        if(pathLength > maxPathLength) { break; }
    }

    score /= pathLength; // normalize to sweep range so we can change that without losing tune
    return score;
}


// returns sign of the input
int AStarAttack::sign(float num) { return (num < 0)? -1 : 1; }


// // true if turning towards this follow point will require it to turn past the opp
// bool AStarAttack::willTurnPastOpp(FollowPoint follow) {

//     float angleToOpp = orbFiltered.angleTo(oppFiltered.position(), follow.forward);
//     float targetAngle = orbFiltered.angleTo(follow.point, follow.forward);

//     // turn direction is either what's enforced or whatever default direction
//     float turnDirection = follow.turnRight ? 1 : -1;

//     if(turnDirection == 1) { targetAngle = angleWrapRad(targetAngle - M_PI) + M_PI; }
//     if(turnDirection == -1) { targetAngle = angleWrapRad(targetAngle + M_PI) - M_PI; }

//     // std::cout << "turn direction = " << turnDirection;

//     float startMargin = orbFiltered.getTurnPastStartMargin() * turnDirection; // if we turn by the opp in the first few degrees anyway then ignore
//     float endMargin = orbFiltered.getTurnPastEndMargin() * turnDirection; // if we don't have to turn past the opp anyway (or just barely turn past) then ignore


//     // std::cout << "target = " << targetAngle << ", start = " << startMargin << ", end = " << endMargin;


//     // if the sweep range is a full circle we don't turn past opp
//     if(turnDirection * (targetAngle - startMargin - endMargin) < 0) { // don't touch
//         // std::cout << "full circle";
//         return false; } 

//     float endAngle = angleWrapRad(targetAngle - endMargin); // end of the range we'll turn through


//     // std::cout << "turn direction = " << turnDirection;



//     // float globalStart = angleWrapRad(startMargin + orbFiltered.angle(follow.forward));
//     // float globalEnd = angleWrapRad(endAngle + orbFiltered.angle(follow.forward));
//     // cv::Point2f startPoint = ppRad() * cv::Point2f(cos(globalStart), sin(globalStart)) + orbFiltered.position();
//     // cv::Point2f endPoint = ppRad() * cv::Point2f(cos(globalEnd), sin(globalEnd)) + orbFiltered.position();

//     // cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), startPoint, cv::Scalar(255, 255, 255), 2);
//     // cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), endPoint, cv::Scalar(0, 255, 255), 2);


//     // rotate everything such the start of the range is on the wrap around line
//     float rotatedEnd = angleWrapRad(endAngle - startMargin + M_PI);
//     float rotatedOpp = angleWrapRad(angleToOpp - startMargin + M_PI);

//     // we turn past the opp if it's on the turned-through side of the range
//     if(rotatedOpp*turnDirection < rotatedEnd*turnDirection) { 
//         // std::cout << " turning past opp";
//         return true; }

//     // std::cout << "not turning past opp";
//     return false;
// }



// scores how bad a point based on the curve we need to execute to switch to it
float AStarAttack::switchPointScore(FollowPoint follow) {

    // create a fake opponent at the follow point so we can use ETASim
    FilteredRobot ghostOpp = FilteredRobot();
    ghostOpp.setPos({follow.point.x, follow.point.y, 0.0f});
    ghostOpp.setSizeRadius(0.1f);

    // generate a predicted path to the follow point
    std::vector<cv::Point2f> path = {};
    float garbage = 0.0f;
    orbFiltered.ETASim(ghostOpp, path, false, false, follow.forward, follow.CW, follow.turnAway, garbage, 0.6f);


    // see how long it'll take opp to get to each point
    float worstTimeMargin = -99999999999999.0f;
    int worstIndex = 0;
    for(int i = 0; i < path.size(); i++) {

        float oppTime = (abs(oppFiltered.angleTo(path[i], true)) - oppFiltered.getWeaponAngleReach())/oppFiltered.getMaxTurnSpeed()
        + std::max(oppFiltered.distanceTo(path[i]) - oppFiltered.getSizeRadius() - orbFiltered.getSizeRadius(), 0.0f)/oppFiltered.getMaxMoveSpeed();

        // increases as opp gets there faster, positive if opp will get there first
        // float timeMargin = pathTimes[i] - oppTime; 
        float timeMargin = 0.0f;
        
        // save the point that's closest to us getting hit
        if(timeMargin > worstTimeMargin || i == 0) { 
            worstTimeMargin = timeMargin; 
            worstIndex = i;
        }
    }


    // // check to see how much this path cuts into the radius
    // float maxSqueeze = 0.0f;
    // int maxIndex = 0;
    // for(int i = 0; i < path.size(); i++) {
    //     float squeeze = std::max(follow.radius - oppFiltered.distanceTo(path[i]), 0.0f);
    //     if(squeeze > maxSqueeze) { 
    //         maxSqueeze = squeeze; 
    //         maxIndex = i;
    //     }
    // }


    // DisplayUtils::displayPoints(path, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), 1);
    // safe_circle(RobotController::GetInstance().GetDrawingImage(), path[worstIndex], 3, cv::Scalar(255, 255, 255), 2);

    // std::cout << "  worst time = " << worstTimeMargin << "    ";

    return worstTimeMargin;






    // float distanceToPoint = orbFiltered.distanceTo(follow.point);
    // float testCurveRad = std::min(25.0f, 0.5f*distanceToPoint);

    // float orbSideAngle = 90.0f*TO_RAD; if(!follow.turnRight) { orbSideAngle *= -1; }
    // float centerOffsetAngle = angleWrapRad(orbFiltered.angle(follow.forward) + orbSideAngle);

    // cv::Point2f arcCenter = orbFiltered.position() + testCurveRad*cv::Point2f(cos(centerOffsetAngle), sin(centerOffsetAngle));

    // float distance = cv::norm(arcCenter - follow.point);
    // float theta = acos(testCurveRad / distance);
    // float alpha = atan2(follow.point.y - arcCenter.y, follow.point.x - arcCenter.x);

    // int invert = follow.turnRight? -1 : 1;
    // cv::Point2f arcEnd = arcCenter + testCurveRad*cv::Point2f(cos(theta*invert + alpha), sin(theta*invert + alpha));


    // float squeezeDistance = std::max(testCurveRad + follow.radius - cv::norm(oppFiltered.position() - arcCenter), 0.0);

    // float startToOpp = oppFiltered.distanceTo(orbFiltered.position());
    // float endToOpp = oppFiltered.distanceTo(arcEnd);

    // safe_circle(RobotController::GetInstance().GetDrawingImage(), arcCenter, testCurveRad, cv::Scalar(255, 255, 255), 1);
    // cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), arcCenter, cv::Scalar(255, 255, 255), 2);
    // cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), arcCenter, cv::Scalar(255, 255, 255), 2);
    // cv::line(RobotController::GetInstance().GetDrawingImage(), arcEnd, arcCenter, cv::Scalar(255, 255, 255), 2);


    // // std::cout << "squeeze = " << squeezeDistance << "   ";

    // return squeezeDistance;
}





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
    if(ppWallPoints.size() == 0 && field.insideFieldBounds(orbFiltered.position())) { return; }

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

    float angleToBound = angle(orbFiltered.position(), field.closestBoundPoint(orbFiltered.position())); // angle to the closest bound point

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









    // now change the drive angle if needed

    // intersection of pp radius and drive angle
    cv::Point2f drivePoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(follow.driveAngle), sin(follow.driveAngle));

    // if drive point isn't in the field we need to change it
    if(!field.insideFieldBounds(drivePoint)) {

        // set it to the closest bound of the region
        float minAngleOffset = angle_wrap(follow.driveAngle - minAngle);
        float maxAngleOffset = angle_wrap(follow.driveAngle - maxAngle);

        // set drive angle to the correct bound
        follow.driveAngle = maxAngle;
        if(abs(minAngleOffset) < abs(maxAngleOffset)) { follow.driveAngle = minAngle; }
    }









    // make sure we turn the right direction to the drive angle

    float orbOffset = angle_wrap(angleToBound - orbFiltered.angle(follow.forward));
    float driveOffset = angle_wrap(angleToBound - follow.driveAngle);

    // only change the direction enforcement if turning the wrong way would make us go way out of the field
    if(abs(orbOffset) > 5.0f*TO_RAD) { 

        // stop enforcing direction bc that might make it go out
        follow.turnAway = false;

        if(orbOffset > 0 && driveOffset < 0) { follow.enforceTurnDirection = -1; }
        if(orbOffset < 0 && driveOffset > 0) { follow.enforceTurnDirection = 1; }

        if(!field.insideFieldBounds(orbFiltered.position())) { follow.enforceTurnDirection *= -1; } // needs to invert if out of the field
    }

}



// how good a follow point is based on our ability to get away if needed
float AStarAttack::escapeScore(FollowPoint &follow) {

    generateEscapePath(follow); // first generate the path we'll take when getting away


    // check the escape path for the worst score point
    float highestScore = 0.0f;
    int highestIndex = 0;
    for(int i = 0; i < follow.escapePath.size(); i++) {

        float orbETA = follow.escapePathTimes[i];
        float oppETA = oppFiltered.collideETASimple(follow.escapePath[i], orbFiltered.getSizeRadius(), true);
    
        float score = 999999999999.0f;
        if(oppETA > 0.001f) { score = -(oppETA - orbETA) / oppETA; }
    
        if(score > highestScore || i == 0) { 
            highestScore = score; 
            highestIndex = 0;
        }
    }

    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.escapePath[highestIndex], 5, cv::Scalar(255, 255, 255), 5);

    return highestScore; 
}



// generates an escape path
void AStarAttack::generateEscapePath(FollowPoint &follow) {

    std::vector<float> simPos = orbFiltered.getPosFiltered();

    // starting speeds
    float simLinearSpeed = orbFiltered.tangentVel(true);
    float simTurnSpeed = orbFiltered.turnVelSlow();

    float timeStep = 0.01f;
    float maxTime = 1.5f;

    float simTime = 0.0f;
    int steps = (int) (maxTime / timeStep);

    follow.escapePath = {};
    follow.escapePathTimes = {};

    float angleErrorPrev = 0.0f; // oh boy

    FilteredRobot virtualOrb = orbFiltered; // virtual orb that gets simulated

    // simulate orb's actions through each time step
    for(int i = 0; i < steps; i++) { 

        // add the point to the path
        virtualOrb.setPos(simPos);
        follow.escapePath.emplace_back(virtualOrb.position()); 
        follow.escapePathTimes.emplace_back(simTime);



        // set target angle to directly away from opp
        float targetAngle = angle(follow.opp.position(), virtualOrb.position());
        
        // how far off we are from the simulated follow point
        float angleError = angle_wrap(targetAngle - virtualOrb.angle(follow.forward));
        // float angleToOpp = virtualOrb.angleTo(follow.opp.position(), follow.forward);
        // float angleToPoint = orbFiltered.angleTo(follow.point, follow.forward);
        // float virtualAngleToPoint = virtualOrb.angleTo(follow.point, follow.forward);

        // int pointDirection = (angleToPoint > 0)? 1 : -1;


        // // make sure we turn the proper direction to simulate getting away the proper direction
        // float wrapOffset = 0.0f;
        // if(follow.turnAway) {
        //     if(follow.CW) { wrapOffset = M_PI; }
        //     else { wrapOffset = -M_PI; }
        // }
        // else {
        //     if(pointDirection == 1) { wrapOffset = M_PI; }
        //     else { wrapOffset = -M_PI; }
        // }

        // angleToOpp = angle_wrap(angleToOpp - wrapOffset) + wrapOffset; 
        // angleToPoint = angle_wrap(angleToPoint - wrapOffset) + wrapOffset; 
        // virtualAngleToPoint = angle_wrap(virtualAngleToPoint - wrapOffset) + wrapOffset; 


        // // reverse the turn direction if needed
        // if(abs(angleToPoint) < abs(angleToOpp)) { wrapOffset *= -1; }

        // angleError = angle_wrap(angleError - wrapOffset) + wrapOffset;


        float wrapOffset = follow.CW? -M_PI : M_PI; // enforce that we turn the actual way we'd have to escape
        if(follow.turnAway) { wrapOffset *= -1; } // invert if turning away

        angleError = angle_wrap(angleError - wrapOffset) + wrapOffset;
    


        if(abs(angleError) < 45.0f*TO_RAD) { break; }
        // if(abs(angleToPoint) < abs(angleToOpp) && abs(angleToOpp) < abs(angleError)) { break; }
        // if(abs(virtualAngleToPoint) > 350.0f*TO_RAD || abs(angleError) < 30.0f*TO_RAD) { break; }

        

        

        // simulate what curvature controller would do
        float curvature = 0.90f*angleError; // 0.7



        float moveInput = 1.0f * (follow.forward ? 1 : -1); // assume full gas
        float turnInput = abs(moveInput) * curvature; // curvature = 1/radius so this is the same as w = v/r formula

        // don't demand more than 1.0 speed from any one motor side, scale everything down since that will maintain the same path curvature
        float totalSpeed = abs(moveInput) + abs(turnInput);
        if(totalSpeed > 1.0f) {
            moveInput /= totalSpeed;
            turnInput /= totalSpeed;
        }

        // simulate damper
        float damper = (angleError - angleErrorPrev) / timeStep;
        turnInput -= 0.0007f * damper;
        turnInput = std::clamp(turnInput, -1.0f, 1.0f);

        // let move go back up or down to whatever it can
        moveInput = (1.0f - abs(turnInput)) * (follow.forward ? 1 : -1);


        // free speeds as defined by simulated curvature controller
        float desiredLinearSpeed = moveInput * virtualOrb.getMaxMoveSpeed();
        float desiredTurnSpeed = turnInput * virtualOrb.getMaxTurnSpeed();

        // how far off from free speeds we are
        float linearError = desiredLinearSpeed - simLinearSpeed;
        float turnError = desiredTurnSpeed - simTurnSpeed;

        // how much we accelerate linearly and rotationally 10, 6
        float linearAccel = std::clamp(13.0f * virtualOrb.getMaxMoveAccel() * (linearError / virtualOrb.getMaxMoveSpeed()), -virtualOrb.getMaxMoveAccel(), virtualOrb.getMaxMoveAccel());
        float turnAccel = std::clamp(7.0f * virtualOrb.getMaxTurnAccel() * (turnError / virtualOrb.getMaxTurnSpeed()), -virtualOrb.getMaxTurnAccel(), virtualOrb.getMaxTurnAccel());

        // increment speeds by accels
        simLinearSpeed += std::clamp(linearAccel * timeStep, -abs(linearError), abs(linearError));
        simTurnSpeed += std::clamp(turnAccel * timeStep, -abs(turnError), abs(turnError));


        




        float deltaXRobot = 0.0f;
        float deltaYRobot = 0.0f;
        float turnDistance = simTurnSpeed * timeStep; // Δθ

        if (abs(simTurnSpeed) < 1e-6f) {
            // straight-line motion
            deltaXRobot = simLinearSpeed * timeStep;
            deltaYRobot = 0.0f;
        } else {
            // circular arc motion
            float r = simLinearSpeed / simTurnSpeed; // path radius
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
            (float) angle_wrap(orientation + turnDistance)
        };

        simTime += timeStep;
    }

}



// score function for determining how good a follow point is, used for CW/CCW decision
void AStarAttack::directionScore(FollowPoint &follow, bool forwardInput) {


    follow.directionScores.emplace_back(follow.radius);



    // how far around the circle we have to go for each direction
    float goAroundAngle = M_PI - (follow.CW? 1 : -1)*oppFiltered.angleTo(orbFiltered.position(), true);
    float goAroundGain = 50.0f + 0.06f * oppFiltered.tangentVel(true); // 50    0.06
    follow.directionScores.emplace_back(goAroundAngle*goAroundGain*0.0f);



    // how far the robot has to turn to this point
    float slowness = std::clamp(1 - orbFiltered.moveSpeedSlow()/350, 0.0f, 1.0f);
    float facingness = abs(oppFiltered.angleTo(orbFiltered.position(), false)) / M_PI;

    float slowFacingGain = 10.0f; // 15
    float fastAwayGain = 4.0f;

    float turnGain = fastAwayGain + (slowFacingGain - fastAwayGain)*(slowness*facingness);
    turnGain = 0.0f; // 12
    follow.directionScores.emplace_back(turnGain * turnScore(follow));

    


    // turn past opp score
    // float turnPast = escapeScore(follow);
    follow.directionScores.emplace_back(0.0f); // 20


    




    // how close is the nearest wall in this direction
    float wallGain = 100.0f; // 100
    follow.directionScores.emplace_back(wallScore(follow)*wallGain);
       

    // how much velocity we already have built up in a given direction, ensures we don't switch to other direction randomly
    float raw = orbFiltered.tangentVel(follow.forward);
    float tanVel = pow(abs(raw), 0.5f) * sign(raw);
    float momentumWeight = -0.08f; // 1.5
    follow.directionScores.emplace_back(tanVel*momentumWeight);


    // subtract score from directions that agree with input while adding to ones that don't
    int directionAgreement = (follow.forward == forwardInput)? -1 : 1;
    follow.directionScores.emplace_back(150.0f*directionAgreement); // 170


    // sum up the scores and add it as the last entry
    float totalScore = 0.0f;
    for(int i = 0; i < follow.directionScores.size(); i++) { totalScore += follow.directionScores[i]; }
    follow.directionScores.emplace_back(totalScore);

}



// returns score based on how much we have to turn to face the follow point
float AStarAttack::turnScore(FollowPoint follow) {

    // how far we need to rotate to match the drive angle, includes enforced turn direction
    float reverseOffset = follow.forward ? 0 : M_PI;
    float angleError = angle_wrap(follow.driveAngle - orbFiltered.getPosFiltered()[2] + reverseOffset);

    // bool turningCorrect = (orbFiltered.angleTo(oppFiltered.position(), follow.forward) > 0 && follow.CW) || (orbFiltered.angleTo(oppFiltered.position(), follow.forward) < 0 && !follow.CW);
    // if(follow.turnAway && follow.CW && !turningCorrect) { angleError = angle_wrap(angleError - M_PI) + M_PI; }
    // if(follow.turnAway && !follow.CW && !turningCorrect) { angleError = angle_wrap(angleError + M_PI) - M_PI; }

    float wrapOffset = follow.enforceTurnDirection * M_PI;
    angleError = angle_wrap(angleError - wrapOffset) + wrapOffset;
    

    float accel = orbFiltered.getMaxTurnAccel() * sign(angleError); // assume we accel in the direction we're going to turn
    float dirac = pow(orbFiltered.turnVelSlow(), 2.0f) + (2.0f * accel * angleError);
    float t1 = (-orbFiltered.turnVelSlow() + sqrt(dirac)) / accel;
    float t2 = (-orbFiltered.turnVelSlow() - sqrt(dirac)) / accel;

    float time = std::min(t1, t2);
    if(t1 < 0.0f) { time = t2; }
    if(t2 < 0.0f) { time = t1; }

    // std::cout << "turn time = " << time;

    return time;
}



// follow point when we're in the circle no weird ranges
void AStarAttack::followPointInsideCircleSimple(FollowPoint &follow) {

    // // how far around we are
    // float angleAround = oppFiltered.angleTo(orbFiltered.position(), true);


    // angles are offset relative to line that starts at opp and connects to orb
    float angleAtRadius = 90.0f*TO_RAD;
    float angleAtCollision = 45.0f*TO_RAD; // 60


    float angleOppToOrb = angle(oppFiltered.position(), orbFiltered.position());

    cv::Point2f ppFollow = ppPoint(follow);
    float angleToPPGlobal = angle(orbFiltered.position(), ppFollow);
    float angleToPP = angle_wrap(angleToPPGlobal - angleOppToOrb); // angle to pp point in opp's coordinates


    // drive angle is linearly interpolated from collision angle to at the radius
    float radiusPercent = std::clamp(orbFiltered.distanceToCollide(oppFiltered) / (follow.radius - oppFiltered.getSizeRadius() - orbFiltered.getSizeRadius()), 0.0f, 1.0f);
    float driveAngle = angleAtCollision + radiusPercent*(angleAtRadius - angleAtCollision);



    // clip the drive angle if the pp angle is more extreme
    if(orbFiltered.distanceTo(oppFiltered.position()) > follow.radius - ppRad(orbFiltered.moveSpeedSlow())) {
        driveAngle = std::max(driveAngle, abs(angleToPP));
    }

    driveAngle = angle_wrap(angleOppToOrb + driveAngle*(follow.CW? 1 : -1));
    

    // safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 2, cv::Scalar(255, 0, 255), 5);

    // generate a point at the pp rad at the correct angle
    follow.point = orbFiltered.position() + ppRad(orbFiltered.moveSpeedSlow())*cv::Point2f(cos(driveAngle), sin(driveAngle));
}



// determines follow point when we're inside the main attack radius
void AStarAttack::followPointInsideCircle(FollowPoint &follow) {

    float direction = follow.CW? -1.0 : 1.0f;

    float angleToOrb = oppFiltered.angleTo(orbFiltered.position(), true); 
    float distanceToOpp = orbFiltered.distanceTo(oppFiltered.position());
    float collisionRadius = orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius();
    

    // first define parameters at collision radius
    float minAngleFront = -10.0f*TO_RAD; // -30
    float maxAngleFront = -10.0f*TO_RAD; // 0

    float minAngleSide = -40.0f*TO_RAD; // -90
    float maxAngleSide = -40.0f*TO_RAD; // -0

    float sideAngle = -90.0f*TO_RAD;



    // linearly interpolate min and max angles at collision radius based on the current angle to orb
    float anglePercent = std::clamp((angleToOrb * direction) / sideAngle, 0.0f, 1.0f);
    float minAngleClose = anglePercent * (minAngleSide - minAngleFront) + minAngleFront;
    float maxAngleClose = anglePercent * (maxAngleSide - maxAngleFront) + maxAngleFront;



    
    // both angles are linearly interpolated to -90 degrees at the outer radius
    float angleAtRadius = -90.0f*TO_RAD;
    float radiusPercent = std::max((distanceToOpp - collisionRadius), 0.0f) / std::max((follow.radius - collisionRadius), 0.01f);
    float minAngle = radiusPercent * (angleAtRadius - minAngleClose) + minAngleClose;
    minAngle = minAngleClose;
    float maxAngle = radiusPercent * (angleAtRadius - maxAngleClose) + maxAngleClose;




    // convert to world angle
    float minWorldAngle = angle_wrap(minAngle*direction + angleToOrb + oppFiltered.angle(true));
    float maxWorldAngle = angle_wrap(maxAngle*direction + angleToOrb + oppFiltered.angle(true));



    // if the pure pursuit point might be usable, check
    if(distanceToOpp > follow.radius - ppRad(orbFiltered.moveSpeedSlow())) {
        cv::Point2f ppFollow = ppPoint(follow);
        float angleToPP = angle(orbFiltered.position(), ppFollow);
        float maxAngleDifference = angle_wrap(maxWorldAngle - angleToPP);
        float minAngleDifference = angle_wrap(minWorldAngle - angleToPP);


        // if(CW) { safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 5, cv::Scalar(255, 255, 255), 5); }


        
        if(follow.CW && maxAngleDifference < 0) { maxWorldAngle = angleToPP; }
        if(follow.CW && minAngleDifference < 0) { minWorldAngle = angleToPP; }
        if(!follow.CW && maxAngleDifference > 0) { maxWorldAngle = angleToPP; }
        if(!follow.CW && minAngleDifference > 0) { minWorldAngle = angleToPP; }
    }




    // angle to follow
    float angleRange = angle_wrap(maxWorldAngle - minWorldAngle) * direction;
    float midAngle = minWorldAngle + (angleRange / 2) * direction;
    float angleError = angle_wrap(orbFiltered.angle(follow.forward) - midAngle) * direction;


    // default: assume we're in the angle range so just go forward
    // add an offset to make sure we turn in the right direction (no rounding errors)
    // float offset = follow.turnRight? 1.0f*TO_RAD : -1.0f*TO_RAD;
    float offset = 0.0f;
    float followAngle = angle_wrap(orbFiltered.angle(follow.forward) + offset);
    if(angleError < -angleRange/2) { followAngle = minWorldAngle; }
    if(angleError > angleRange/2) { followAngle = maxWorldAngle; }


    // if(forward && CW) {
    //     cv::Point2f minAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(minWorldAngle), orbFiltered.position().y + ppRadius*sin(minWorldAngle));
    //     cv::Point2f maxAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(maxWorldAngle), orbFiltered.position().y + ppRadius*sin(maxWorldAngle));
    //     cv::Point2f midAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(midAngle), orbFiltered.position().y + ppRadius*sin(midAngle));
    //     cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), minAnglePoint, cv::Scalar(255, 50, 50), 2);
    //     cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), maxAnglePoint, cv::Scalar(255, 50, 50), 2);
    //     cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), midAnglePoint, cv::Scalar(255, 150, 150), 2);

    //     // std::cout << "     minAngle = " << minAngle*TO_DEG << ", maxAngle = " << maxAngle*TO_DEG << "      ";
    // }
    

    // follow point is pp radius away at the calculated angle
    follow.point = cv::Point2f(orbFiltered.position().x + ppRad(orbFiltered.moveSpeedSlow())*cos(followAngle), orbFiltered.position().y + ppRad(orbFiltered.moveSpeedSlow())*sin(followAngle));
}


// returns the best follow point with the lowest score
FollowPoint AStarAttack::chooseBestPoint(std::vector<FollowPoint>& follows, bool forwardInput) {

    if(follows.size() == 0) { return FollowPoint(); } // no crashy

    // find follow point with best score
    int bestIndex = 0;
    directionScore(follows[0], forwardInput);
    float bestScore = follows[0].directionScores.back();

    for(int i = 1; i < follows.size(); i++) {

        directionScore(follows[i], forwardInput);
        float newScore = follows[i].directionScores.back(); // last entry is total score

        if(newScore < bestScore) {
            bestScore = newScore;
            bestIndex = i;
        }
    }

    return follows[bestIndex]; // return best index
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
