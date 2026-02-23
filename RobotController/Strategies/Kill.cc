#include "Kill.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include "DisplayUtils.h"
#include "FilteredRobot.h"

Kill::Kill()
{
    // init filters
    orbFiltered = FilteredRobot(1.0f, 40.0f, 430.0f, 800.0f, 
        22.0f, 50.0f, 70.0f*TO_RAD, 20.0f);

    oppFiltered = FilteredRobot(1.0f, 50.0f, 430.0f, 1200.0f, 
        25.0f, 60.0f, 60.0f*TO_RAD, 28.0f);
    
    field = Field();
}



DriverStationMessage Kill::Execute(Gamepad &gamepad)
{
    // track loop time
    static Clock updateClock;
    static ClockWidget processingTimeVisualizer("Kill");
    processingTimeVisualizer.markStart();

    double deltaTime = updateClock.getElapsedTime(); // reset every loop so elapsed time is loop time
    if(deltaTime == 0) { deltaTime = 0.001; } // broski what
    updateClock.markStart(); // reset for next loop



    // get odometry data for orb and opp
    OdometryData orbData = RobotController::GetInstance().odometry.Robot();
    OdometryData oppData = RobotController::GetInstance().odometry.Opponent();

    // update filtered positions/velocities and paths
    orbFiltered.updateFilters(deltaTime, orbData.robotPosition, orbData.GetAngle()); orbFiltered.updatePath();
    oppFiltered.updateFilters(deltaTime, oppData.robotPosition, oppData.GetAngle()); 


    bool forwardInput = (gamepad.GetRightStickY() >= 0.0f); // if we're inputting forward driving direction


    // will be filled with final paths
    std::vector<cv::Point2f> orbSimPath = {};
    std::vector<cv::Point2f> oppSimPath = {};

    // extrapolate opp
    cv::Point2f target = extrapOpp(oppSimPath, orbSimPath, forwardInput);


    // calculate drive inputs based on curvature controller (it's just atan2)
    float targetAngle = angle(orbFiltered.position(), target);
    std::vector<float> driveInputs = orbFiltered.curvatureController(targetAngle, gamepad.GetRightStickY(), deltaTime, forwardInput, 0);



    
   

    // colors
    cv::Scalar colorOrb = cv::Scalar(255, 200, 0);
    if(!forwardInput) { colorOrb = cv::Scalar(0, 200, 255); }

    cv::Scalar colorOpp = cv::Scalar(0, 50, 255);
    cv::Scalar colorOppLight = cv::Scalar(200, 200, 255);


    bool colliding = orbFiltered.distanceTo(oppFiltered.position()) < orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius() + 10.0f;
    bool facing = abs(orbFiltered.angleTo(oppFiltered.position(), forwardInput)) < orbFiltered.getWeaponAngleReach();
    bool oppFacing = abs(oppFiltered.angleTo(orbFiltered.position(), true)) < oppFiltered.getWeaponAngleReach();

    // turn opp green if we hit them
    if(colliding && facing) { 
        colorOpp = cv::Scalar(200, 255, 200); 
        colorOppLight = cv::Scalar(200, 255, 200); 
        DisplayUtils::emote(); // DO NOT DELETE
    }


    // display things we want
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppSimPath.back(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppSimPath.back(), oppFiltered.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size

    DisplayUtils::displayPath(orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path
    // DisplayUtils::displayLines(field.getBoundLines(), cv::Scalar(0, 0, 255)); // draw field bound lines


    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << 0; //orbTime;
    
    std::string collisionTime = "Time to Collision: " + oss.str() + "s";
    std::string forwardStatus = forwardInput ? "Forward" : "Backward"; 

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Killing", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), collisionTime, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);


    
    


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];

    // processingTimeVisualizer.markEnd();
    return ret;
}




// extraps the opp to a point at which orb will collide at the same time
cv::Point2f Kill::extrapOpp(std::vector<cv::Point2f> &oppSimPath, std::vector<cv::Point2f> &orbSimPath, bool forward) {

    float maxExtrap = 1.0f; // furthest we'll ever extrap the opp
    float increment = 0.02f; // increment of extrap time scanning
    float extrapTime = 0.0f; // how much extrap we're currently checking

    int maxSteps = (int) maxExtrap / increment;

    oppSimPath = {}; // reset just in case

    FilteredRobot virtualOpp = oppFiltered; // start virtual opp at opp's current state
    virtualOpp.setToSlowVel(); // use slow vel to avoid noise


    // extrap up to the max number of steps
    for(int i = 0; i < maxSteps; i++) {

        oppSimPath.emplace_back(virtualOpp.position()); // add position to sim path

        float orbETA = orbTimeToPoint(virtualOpp.position(), orbSimPath, forward); // simulate orb's path to the point
        if(orbETA < extrapTime) { break; } // break when we arrive at the same time




        // now we need to extrapolate opp by another increment for the next update

        std::vector<std::vector<float>> oppExtrap = virtualOpp.constVelExtrap(increment); // extrapolate opp another time step

        float velLeft1Sec = 0.04f; // 0.01 what percent of velocity is left after each second
        float velPercent = pow(velLeft1Sec, increment); // what percent of velocity is left after this timestep

        float turnLeft1Sec = 0.003f; // 0.003
        float turnPercent = pow(turnLeft1Sec, increment);

        std::vector<float> newPos = oppExtrap[0]; // pull out new pos
        std::vector<float> newVel = oppExtrap[1]; // pull out new vel

        newVel[0] *= velPercent; 
        newVel[1] *= velPercent;
        newVel[2] *= turnPercent;

        virtualOpp.setPos(newPos);
        virtualOpp.setVel(newVel);
        virtualOpp.setAccel({0, 0, 0});

        extrapTime += increment;
    }

    return virtualOpp.position(); // return final opp position if max time is reached
}



// simulates orb's path to a point to estimate time
float Kill::orbTimeToPoint(cv::Point2f point, std::vector<cv::Point2f> &orbSimPath, bool forward) {

    float timeStep = 0.02f;
    float maxTime = 3.0f;
    float simTime = 0.0f; // amount of time elapsed since start of sim
    int steps = (int) (maxTime / timeStep);

    orbSimPath = {}; // reset just in case
    FilteredRobot virtualOrb = orbFiltered; // start simulated orb at our current state

    std::vector<float> slowVel = virtualOrb.getVelFilteredSlow();
    virtualOrb.setVel({slowVel[0], slowVel[1], slowVel[2]});
 

    // simulate orb's actions through each time step
    for(int i = 0; i < steps; i++) { 

        // add current position to the path
        orbSimPath.emplace_back(virtualOrb.position()); 


        // how far off we are from the simulated follow point
        float angleError = virtualOrb.angleTo(point, forward); 


        // exit conditions occur when we hit the point and we're facing it
        bool colliding = virtualOrb.distanceTo(point) < orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius();
        if(colliding && virtualOrb.facingPoint(point, forward)) { break; }


        
        // drive to the follow point
        float driveAngle = angle(virtualOrb.position(), point);

        std::vector<float> driveInputs = virtualOrb.curvatureController(driveAngle, 
            1.0f, timeStep, forward, 0);

        std::vector<float> modelInputs = {driveInputs[0], driveInputs[1], 0.0f, 0.0f, timeStep};
        virtualOrb.tuneModel(false, modelInputs, {});


        simTime += timeStep;
    }

    return simTime; 
}



// absolute angle made by 2 points
float Kill::angle(cv::Point2f point1, cv::Point2f point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}
