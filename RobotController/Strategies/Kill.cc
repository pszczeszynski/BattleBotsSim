#include "Kill.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"

Kill::Kill()
{
    // initialize filters
    orbFiltered = FilteredRobot(1.0f, 100.0f, 430.0f, 200.0f, 2.0f*360.0f*TO_RAD, 80.0f*360.0f*TO_RAD, 30.0f*TO_RAD, 10.0f*TO_RAD, 20.0f);
    oppFiltered = FilteredRobot(1.0f, 100.0f, 400.0f, 300.0f, 2.0f*360.0f*TO_RAD, 200.0f*360.0f*TO_RAD, 50.0f*TO_RAD, 40.0f*TO_RAD, 25.0f);

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



    // driving direction
    static bool forward = true;
    forward = (gamepad.GetRightStickY() > 0.0f);


    static cv::Point2f deltaFiltered = cv::Point2f(0, 0);
    cv::Point2f targetPoint = collidePoint(forward); // find the point at which we'll collide at the same time

    cv::Point2f deltaRaw = targetPoint - oppFiltered.position();
    float filterPercent = 0.1f;
    deltaFiltered = (1.0f - filterPercent)*deltaFiltered + (filterPercent)*deltaRaw;


    // it's just atan2
    cv::Point2f followPoint = oppFiltered.position() + deltaFiltered;
    
   

    // display things we want
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, cv::Scalar(0, 0, 255), 2); // draw dot on the opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppFiltered.getSizeRadius(), cv::Scalar(190, 190, 255), 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), cv::Scalar(255, 190, 190), 2); // draw op size



    // calculate drive inputs based on curvature controller
        std::vector<float> driveInputs = curvatureController(followPoint, gamepad.GetRightStickY(), deltaTime, 0, forward);


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];
    
    
    // processingTimeVisualizer.markEnd();
    return ret;
}





// extrapolates the opp's pos to see what point we'll line up at
cv::Point2f Kill::collidePoint(bool forward) {

    float timeIncrement = 0.01f; // time increment of the simulation
    float maxExtrapTime = 0.5f; // max time to project into the future
    float simTime = 0.0f; // time in the sim
    cv::Point2f oppPositionExtrap = oppFiltered.position(); // extrapolated opp pos, starts on the opp
    float timeOffset = 0.0005f * (orbFiltered.distanceTo(oppFiltered.position()) - orbFiltered.getSizeRadius() - oppFiltered.getSizeRadius()); // how much we want to lag the opponent to make sure our side doesn't get hit, more lag (higher value) = less likely to lead them but more likely to get side hit

    // run until the sim time is above the max time
    while(simTime < maxExtrapTime) {

        FilteredRobot virtualOpp = FilteredRobot(oppPositionExtrap, oppFiltered.getSizeRadius()); // create virtual opp at the extrapolated pos with same size
        float orbETA = orbFiltered.collideETA(virtualOpp, forward); // how long will it take orb to get to the virtual opp
        if(orbETA < simTime + timeOffset) { break; } // break the loop at the first point where we'll get there slightly faster

        simTime += timeIncrement; // increment sim time
        std::vector<std::vector<float>> oppExtrap = oppFiltered.constVelExtrap(simTime); // extrapolate opp by the sim time
        oppPositionExtrap = cv::Point2f(oppExtrap[0][0], oppExtrap[0][1]); // grab XY coords
    }

    return oppPositionExtrap; // return the final extrap position
}


// calculates move and turn speeds to follow the followPoint
std::vector<float> Kill::curvatureController(cv::Point2f followPoint, float moveSpeed, float deltaTime, int turnDirection, bool forward) {

    float angleError = orbFiltered.angleTo(followPoint, forward); // how far off we are from target
    if(turnDirection == 1) { angleError = angleWrapRad(angleError - M_PI) + M_PI; }
    if(turnDirection == -1) { angleError = angleWrapRad(angleError + M_PI) - M_PI; }

    static float prevAngleError = 0.0f; // track how error is changing
    float angleErrorChange = (angleError - prevAngleError) / deltaTime; // how the error is changing per time
    prevAngleError = angleError; // save for next time

    // determine desired path curvature/drive radius using pd controller and magic limits
    float currSpeed = cv::norm(cv::Point2f(orbFiltered.getVelFilteredSlow()[0], orbFiltered.getVelFilteredSlow()[1]));

    // 320
    float maxCurveGrip = pow(290.0f, 2.0f) / std::max(pow(currSpeed, 2), 0.1); // max curvature to avoid slipping, faster you go the less curvature you're allowed
    float maxCurveScrub = currSpeed * 0.01f; // max curvature to avoid turning in place when moving slow, faster you go the more curvature you're allowed cuz you're already moving
    float maxCurve = std::min(maxCurveGrip, maxCurveScrub); // use the lower value as the boundary

    // pd controller that increases path curvature with angle error
    float curvature = std::clamp(0.8f*angleError + 0.06f*angleErrorChange, -maxCurve, maxCurve); // 0.8f, 0.07f

    // reverese the input if we're going backwards
    float turnSpeed = abs(moveSpeed) * curvature; // curvature = 1/radius so this is the same as w = v/r formula

    // don't demand more than 1.0 speed from any one motor side, scale everything down since that will maintain the same path curvature
    float totalSpeed = abs(moveSpeed) + abs(turnSpeed);
    if(totalSpeed > 1.0f) {
        moveSpeed /= totalSpeed;
        turnSpeed /= totalSpeed;
    }

    return std::vector<float> {moveSpeed, turnSpeed};
}


// returns sign of the input
int Kill::sign(float num) {
    if(num < 0) { return -1; }
    return 1;
}