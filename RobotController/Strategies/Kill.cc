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



    // driving direction
    static bool forward = true;
    forward = (gamepad.GetRightStickY() > 0.0f);


    // get odometry data for orb and opp
    OdometryData orbData = RobotController::GetInstance().odometry.Robot();
    OdometryData oppData = RobotController::GetInstance().odometry.Opponent();

    // update filtered positions/velocities and paths
    orbFiltered.updateFilters(deltaTime, orbData.robotPosition, orbData.GetAngle()); orbFiltered.updatePath();
    oppFiltered.updateFilters(deltaTime, oppData.robotPosition, oppData.GetAngle()); 

    // create extrapolated opponent
    oppExtrap = orbFiltered.createVirtualOpp(oppFiltered, forward, 0.5f, 0.000f);

    // static cv::Point2f deltaFiltered = cv::Point2f(0, 0);
    // cv::Point2f deltaRaw = oppExtrap.position() - oppFiltered.position();
    // float filterPercent = std::clamp(100.0f * deltaTime, 0.0, 0.2);
    // deltaFiltered = (1.0f - filterPercent)*deltaFiltered + (filterPercent)*deltaRaw;


    std::vector<cv::Point2f> orbSimPath = {};
    float orbTime = orbFiltered.ETASim(oppExtrap, orbSimPath, false, false);
    std::cout << "orbTime = " << orbTime << std::endl;


    // it's just atan2
    cv::Point2f followPoint = oppExtrap.position();
    
   

    // display things we want
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppExtrap.position(), 10, cv::Scalar(0, 0, 255), 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, cv::Scalar(0, 0, 255), 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppExtrap.position(), oppExtrap.getSizeRadius(), cv::Scalar(190, 190, 255), 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), cv::Scalar(255, 190, 190), 2); // draw op size

    displayPathPoints(orbSimPath, cv::Scalar(255, 200, 0)); // display orb's simulated path


    // calculate drive inputs based on curvature controller
    std::vector<float> driveInputs = orbFiltered.curvatureController(followPoint, 0.8f, 0.06f, gamepad.GetRightStickY(), deltaTime, 0, forward);


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];
    
    
    // processingTimeVisualizer.markEnd();
    return ret;
}





// display a path of points as points
void Kill::displayPathPoints(std::vector<cv::Point2f>& path, cv::Scalar color) {
    for (int i = 0; i < path.size(); i++) {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        safe_circle(RobotController::GetInstance().GetDrawingImage(), centerInt, 3, color, 2);
    }
}