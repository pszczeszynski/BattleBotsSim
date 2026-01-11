#include "Kill.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include "DisplayUtils.h"

Kill::Kill()
{
    // init filters
    orbFiltered = FilteredRobot(1.0f, 50.0f, 500.0f, 200.0f, 2.0f*360.0f*TO_RAD, 80.0f*360.0f*TO_RAD, 50.0f*TO_RAD, 40.0f*TO_RAD, 20.0f);
    oppFiltered = FilteredRobot(1.0f, 50.0f, 470.0f, 300.0f, 2.0f*360.0f*TO_RAD, 200.0f*360.0f*TO_RAD, 60.0f*TO_RAD, 40.0f*TO_RAD, 25.0f);

    // init field
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


    // if we're inputting forward driving direction
    bool forwardInput = (gamepad.GetRightStickY() >= 0.0f);


    // generate the extrapolated opp to where we'll collide
    std::vector<cv::Point2f> oppSimPath = {};
    FilteredRobot oppExtrap = orbFiltered.createVirtualOpp(oppFiltered, forwardInput, 1.0f, oppSimPath);

    // // if the opp extrapolated out of the field, clip it in
    // if(!field.insideFieldBounds(oppExtrap.position())) {
    //     cv::Point2f clippedPosition = field.closestBoundPoint(oppExtrap.position());
    //     std::vector<float> clippedPos = {clippedPosition.x, clippedPosition.y, oppFiltered.getPosFiltered()[2]};
    //     oppExtrap.setPos(clippedPos);
    // }


    // recalculate orb sim path for displaying
    std::vector<cv::Point2f> orbSimPath = {};
    float orbTime = orbFiltered.ETASim(oppExtrap, orbSimPath, false, false, forwardInput);
    // std::cout << "orbTime = " << orbTime << std::endl;



    // calculate drive inputs based on curvature controller (it's just atan2)
    float targetAngle = atan2(oppExtrap.position().y - orbFiltered.position().y, oppExtrap.position().x - orbFiltered.position().x);
    std::vector<float> driveInputs = orbFiltered.curvatureController(targetAngle, 0.8f, 0.06f, gamepad.GetRightStickY(), deltaTime, 0, forwardInput);



    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];





    
   

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
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppExtrap.position(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppExtrap.position(), oppExtrap.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size

    DisplayUtils::displayPath(orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path
    // DisplayUtils::displayLines(field.getBoundLines(), cv::Scalar(0, 0, 255)); // draw field bound lines


    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << orbTime;
    
    std::string collisionTime = "Time to Collision: " + oss.str() + "s";
    std::string forwardStatus = forwardInput ? "Forward" : "Backward"; 

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Killing", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), collisionTime, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);


    
    



    // processingTimeVisualizer.markEnd();
    return ret;
}

