#include "Kill.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include "DisplayUtils.h"

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


    bool forward = gamepad.GetRightStickY() >= 0.0f; // if we're driving forward to the opp

    std::vector<cv::Point2f> oppSimPath = {};
    FilteredRobot oppExtrap = orbFiltered.createVirtualOpp(oppFiltered, forward, 1.0f, 0.000f, oppSimPath);


    std::vector<cv::Point2f> orbSimPath = {};
    float orbTime = orbFiltered.ETASim(oppExtrap, orbSimPath, false, false, forward);
    std::cout << "orbTime = " << orbTime << std::endl;


    // it's just atan2
    cv::Point2f followPoint = oppExtrap.position();
    
   

    // colors
    cv::Scalar colorOrb = cv::Scalar(255, 200, 0);
    if(!forward) { colorOrb = cv::Scalar(0, 200, 255); }

    cv::Scalar colorOpp = cv::Scalar(0, 50, 255);
    cv::Scalar colorOppLight = cv::Scalar(200, 200, 255);


    bool colliding = orbFiltered.distanceTo(oppFiltered.position()) < orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius() + 10.0f;
    bool facing = abs(orbFiltered.angleTo(oppFiltered.position(), forward)) < orbFiltered.getWeaponAngleReach();
    bool oppFacing = abs(oppFiltered.angleTo(orbFiltered.position(), true)) < oppFiltered.getWeaponAngleReach();

    // turn opp green if we hit them
    if(colliding && facing) { 
        colorOpp = cv::Scalar(200, 255, 200); 
        colorOppLight = cv::Scalar(200, 255, 200); 
        emote(); // DO NOT DELETE
    }


    // display things we want
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppExtrap.position(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppExtrap.position(), oppExtrap.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size

    DisplayUtils::displayPath(orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path

    std::string forwardStatus = "Forward"; 
    if(!forward) { forwardStatus = "Backward"; }

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Killing", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);


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





// WEWEWEWEWEEE
void Kill::emote() {

    cv::Mat drawing = RobotController::GetInstance().GetDrawingImage();

    cv::Mat logo = cv::imread("SkullEmote.png", cv::IMREAD_UNCHANGED); // keep alpha
    if (!logo.empty()) {
        // Pick top-left corner of where you want it
        cv::Point topLeft(100, 100);

        // Make sure it fits in the drawing surface
        cv::Rect roi(topLeft.x, topLeft.y, logo.cols, logo.rows);

        // If PNG has 4 channels (BGRA), split alpha and blend
        if (logo.channels() == 4) {
            std::vector<cv::Mat> layers;
            cv::split(logo, layers);
            cv::Mat bgr, alpha;
            cv::merge(std::vector<cv::Mat>{layers[0], layers[1], layers[2]}, bgr);
            alpha = layers[3];

            // blend PNG into drawing (simple alpha composite)
            cv::Mat region = drawing(roi);
            for (int y = 0; y < roi.height; ++y) {
                for (int x = 0; x < roi.width; ++x) {
                    float a = alpha.at<uchar>(y, x) / 255.0f;
                    for (int c = 0; c < 3; ++c) {
                        region.at<cv::Vec3b>(y, x)[c] =
                            static_cast<uchar>(a * bgr.at<cv::Vec3b>(y, x)[c] +
                                            (1.0f - a) * region.at<cv::Vec3b>(y, x)[c]);
                    }
                }
            }
        } else {
            // plain BGR copy if no alpha
            logo.copyTo(drawing(roi));
        }
    }

    // std::cout << "Working directory is: " << std::filesystem::current_path() << "\n";
}