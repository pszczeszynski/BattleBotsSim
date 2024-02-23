#include "Input/Gamepad.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "Odometry/BlobDetection/RobotClassifier.h"
#include "RobotLink.h"
// #include "Vision.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include "UIWidgets/IMUWidget.h"
#include "UIWidgets/RobotControllerGUI.h"
#include "imgui.h"
#include "Strategies/Orbit.h"
#include "Strategies/Kill.h"
#include "Strategies/Avoid.h"
#include "Strategies/RobotMovement.h"
#include "UIWidgets/FieldWidget.h"
#include "UIWidgets/KillWidget.h"
#include "UIWidgets/ClockWidget.h"
#include "Input/InputState.h"

int main()
{
    loadGlobalVariablesFromFile("RobotConfig.txt");
    // initialize the robot controller
    RobotController& robotController = RobotController::GetInstance();
    // run the robot controller
    robotController.Run();
    return 0;
}


RobotController::RobotController() : drawingImage(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)),
#ifdef SIMULATION
                                     overheadCamL_sim{"overheadCamL"},
                                     odometry{overheadCamL_sim},
                                     videoSource{overheadCamL_sim}
#elif defined(VIDEO_FILES)
                                     overheadCamL_video{},
                                     odometry{overheadCamL_video},
                                     videoSource{overheadCamL_video}
#else
                                     overheadCamL_real{-1}, //TBD: Need camera index #ll
                                     odometry{overheadCamL_real},
                                     videoSource{overheadCamL_real}
#endif
{
}

RobotController& RobotController::GetInstance()
{
    static RobotController instance;
    return instance;
}

/**
 * Gets the most recent imu data from the robot
*/
IMUData& RobotController::GetIMUData()
{
    return _lastIMUMessage.imuData;
}

/**
 * Gets the most recent can data from the robot
*/
CANData RobotController::GetCANData()
{
    CANData ret;
    // make everything 0
    memset(&ret, 0, sizeof(CANData));
    // lock the mutex
    _lastCanMessageMutex.lock();
    // copy the data
    ret = _lastCANMessage.canData;
    // unlock the mutex
    _lastCanMessageMutex.unlock();

    frontWeaponCurrRPMPercent = ret.motorERPM[2] / 88.0f;
    backWeaponCurrRPMPercent = ret.motorERPM[3] / 88.0f;
    // return the data
    return ret;
}

#define MIN_ROBOT_CONTROLLER_LOOP_TIME_MS 5.0f

void RobotController::Run()
{
    TIMER_INIT
    Clock lastTime;
    lastTime.markStart();

    // For real robot, send a stop drive command to start
#ifndef SIMULATION
    DriveCommand c{0, 0};
    robotLink.Drive(c);
#endif

    // Start the odometry threads
    // Do blob detection
    odometry.Run(OdometryAlg::Blob);

    // Do Heuristic
    odometry.Run(OdometryAlg::Heuristic);

    // run the gui in a separate thread
    std::thread guiThread = std::thread([]() {
        RobotControllerGUI::GetInstance();

        while (true)
        {
            // update the gui
            RobotControllerGUI::GetInstance().Update();
        }
    });

    // // sleep for 0.5 seconds to allow the gui to start
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));


    ClockWidget loopClock("Total loop time");
    cv::Mat zeroArray;

    // receive until the peer closes the connection
    int videoID = -1;
    while (true)
    {
        loopClock.markEnd();

        // If the elapsed time is less then the minimum time for this loop, then sleep for the remaining
        double delta_time = MIN_ROBOT_CONTROLLER_LOOP_TIME_MS -loopClock.getElapsedTime() * 1000.0f;
        if( delta_time > 1.0f )
        {
            Sleep( (DWORD) delta_time);
        }

        loopClock.markStart();

        // Get the latest background image
        if( latestVideoImage.empty() || videoSource.NewFrameReady(videoID) )
        {
            cv::Mat newimage;
            videoID = videoSource.GetFrame(newimage,videoID);

            if( newimage.channels()==1)
            {
                cv::cvtColor(newimage, latestVideoImage, cv::COLOR_GRAY2BGR);
            }
            else
            {
                latestVideoImage = newimage;
            }
        }

        // Initialize our zeroArray
        if(zeroArray.empty()) { zeroArray = cv::Mat::zeros(latestVideoImage.size(), CV_8UC3); }
        
        // Clear the overlay image (RGB)
        zeroArray.copyTo( latestOverlay);

        // update the gamepad
        gamepad.Update();

        // receive the latest message
        RobotMessage msg = robotLink.Receive();

        // save the specific type information in a last struct
        if (msg.type == RobotMessageType::IMU_DATA)
        {
            std::unique_lock<std::mutex> locker(_imudataMutex);
            // Assume each IMU message is unique
            _lastIMUMessage = msg;
            _imuID++; 
            _imuTime = Clock::programClock.getElapsedTime();
        }
        else if (msg.type == RobotMessageType::CAN_DATA)
        {
            _lastCanMessageMutex.lock();
            _lastCANMessage = msg;
            _lastCanMessageMutex.unlock();
        }

        // Update all our odometry data
        odometry.Update();

        // run our robot controller loop
        DriveCommand response = RobotLogic();

        // send the response to the robot
        robotLink.Drive(response);

        // Update overlay images
        // Create a mask of the same size as the overlay image
        cv::Mat mask = cv::Mat::zeros(latestOverlay.size(), CV_8UC1);
        cv::compare(latestOverlay, zeroArray, mask, cv::CMP_NE);

        std::unique_lock<std::mutex> locker(_imageLock);
        latestVideoImage.copyTo(drawingImage);
        latestOverlay.copyTo( drawingImage,mask);
        locker.unlock();

        //_fieldWidget.UpdateMat(drawingImage);
    }

//     RobotControllerGUI::GetInstance().Shutdown();
}


long RobotController::GetIMUFrame(IMUData &output, long old_id, double* frameTime)
{
    // Using scoped mutex locker because conditional variable integrates with it
    // This creates and locks the mutex
    std::unique_lock<std::mutex> locker(_imudataMutex);

    while( (_imuID <= 0) || (_imuID <= old_id))
    {
        // Unlock mutex, waits until conditional varables is notifed, then it locks mutex again
        _imuCV.wait(locker);
    }

    // At this point our mutex is locked and a frame is ready
    output = _lastIMUMessage.imuData;
    old_id = _imuID;

    if( frameTime != NULL)
    {
        *frameTime = _imuTime;
    }

    locker.unlock();

    // return ID of new frame
    return old_id;
}



/**
 * 
*/
void RobotController::UpdateRobotTrackers()
{
    



/*
    static int updatesWithoutOpponent = 0;

    // if vision detected our robot
    if (classification.GetRobotBlob() != nullptr)
    {
        MotionBlob robot = *classification.GetRobotBlob();
        cv::Mat& frame = *(classification.GetRobotBlob()->frame);

        RobotOdometry::Robot().UpdateVisionAndIMU(robot, frame);

        cv::rectangle(drawingImage, robot.rect, cv::Scalar(255, 0, 0, 1));
    }
    else
    {
        // otherwise just update using the imu
        // TODO: DONT JUST USE THE DRAWING IMAGE, that's hacky
        RobotOdometry::Robot().UpdateIMUOnly(drawingImage);
    }

    // if vision detected the opponent
    if (classification.GetOpponentBlob() != nullptr)
    {
        MotionBlob opponent = *classification.GetOpponentBlob();
        cv::Mat& frame = *(classification.GetOpponentBlob()->frame);
        RobotOdometry::Opponent().UpdateVisionOnly(opponent, frame);
        updatesWithoutOpponent = 0;
    }
    else
    {
        updatesWithoutOpponent++;
        if (updatesWithoutOpponent > 10)
        {
            RobotOdometry::Opponent().Invalidate();
        }
    }
*/

}


// Function to draw the rectangle and display the value underneath
void drawAndDisplayValue(cv::Mat &image, double value, double xPosition, cv::Scalar color)
{
    value *= -1;
    // Calculate height and adjust starting y-coordinate based on value's sign
    int rectHeight = std::abs(value);
    int startY = value >= 0 ? HEIGHT / 2 : HEIGHT / 2 - rectHeight;

    // Draw the rectangle
    cv::rectangle(image, cv::Rect(xPosition, startY, 30, rectHeight), color, 10);

    // Display the value under the box
    std::string text = std::to_string(-value);
    int baseline;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    cv::putText(image, text, cv::Point(xPosition, startY + rectHeight + textSize.height + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
}



DriveCommand RobotController::AvoidMode()
{
    DriveCommand ret{0, 0};

    cv::Point2f targetPoint = _movementStrategy.AvoidStrategy();

    // draw the target point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    RobotSimState state;

    OdometryData odoData =  RobotController::GetInstance().odometry.Robot(Clock::programClock.getElapsedTime());

    state.position = odoData.robotPosition;
    state.angle = odoData.robotAngle;
    state.velocity = odoData.robotVelocity;
    state.angularVelocity = odoData.robotAngleVelocity;

    // drive towards it
    ret = RobotMovement::DriveToPosition(state, targetPoint, RobotMovement::DriveDirection::Auto);

    return ret;
}

Gamepad& RobotController::GetGamepad()
{
    return gamepad;
}

/**
 * Allows the spacebar to control switching the robot positions should the trackers swap
*/
void SpaceSwitchesRobots()
{
    static bool spacePressedLast = false;
    bool spacePressed = InputState::GetInstance().IsKeyDown(ImGuiKey_Space);

    // if the space bar was just pressed down
    if (spacePressed && !spacePressedLast)
    {
        RobotController::GetInstance().odometry.SwitchRobots();
    }

    // save the last variable for next time
    spacePressedLast = spacePressed;
}

/**
 * ManualMode
 * Allows the user to drive the robot manually
 */
DriveCommand RobotController::ManualMode()
{
    // drive the robot with the gamepad
    DriveCommand response{0, 0};
    response.movement = gamepad.GetRightStickY();
    response.turn = -gamepad.GetLeftStickX();

    // update the spinner powers
    Weapons& weapons = Weapons::GetInstance();

    weapons.UpdateSpinnerPowers();

    // apply weapon powers
    response.frontWeaponPower = weapons.GetFrontWeaponTargetPower();
    response.backWeaponPower = weapons.GetBackWeaponTargetPower();

    float power = (int) gamepad.GetDpadDown() - (int) gamepad.GetDpadUp();

    power += InputState::GetInstance().IsKeyDown(ImGuiKey_J) - InputState::GetInstance().IsKeyDown(ImGuiKey_U);

    // control the self righter
    _selfRighter.Move(power, response, drawingImage);

    // deadband the movement
    if (abs(response.movement) < 0.05)
    {
        response.movement = 0;
    }

    // deadband the turn
    if (abs(response.turn) < 0.05)
    {
        response.turn = 0;
    }

    SpaceSwitchesRobots();

    return response;
}

/**
 * RobotLogic
 * The main logic for the robot
 */
// sets the resolution of the extrapolation
#define NUM_PREDICTION_ITERS 50
DriveCommand RobotController::RobotLogic()
{
    // draw arrow in the direction of the robot
    OdometryData odoData =  RobotController::GetInstance().odometry.Robot(Clock::programClock.getElapsedTime());


    cv::Point2f robotPos = odoData.robotPosition;
    Angle robotAngle = odoData.robotAngle;
    float robotAnglef = (double) robotAngle;
    cv::Point2f arrowEnd = robotPos + cv::Point2f{cos(robotAnglef), sin(robotAnglef)} * 50;
    cv::arrowedLine(drawingImage, robotPos, arrowEnd, cv::Scalar(0, 0, 255), 2);

    Orbit orbitMode = Orbit{};
    Kill killMode = Kill{};

    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = orbitMode.Execute(gamepad);
    // DriveCommand responseAvoid = AvoidMode();

    // start with just manual control
    DriveCommand ret = responseManual;

    _orbiting = false;
    _killing = false;

    // if the user activates kill mode or is pressing the kill button on the ui
    if (gamepad.GetRightBumper() || KillWidget::GetInstance().IsPressingButton())
    {
        // drive directly to the opponent
        DriveCommand responseGoToPoint = killMode.Execute(gamepad);
        ret.turn = responseGoToPoint.turn;
        ret.movement = responseManual.movement * abs(responseGoToPoint.movement);
        _killing = true;
    }
    // if driver wants to evade (left bumper)
    else if (gamepad.GetLeftBumper())
    {
        // orbit around them
        ret.turn = responseOrbit.turn;
        ret.movement = responseManual.movement * abs(responseOrbit.movement);
        _orbiting = true;
    }

    // return the response
    return ret;
}

IRobotLink& RobotController::GetRobotLink()
{
    return robotLink;
}

// Returns a fresh copy of the internal image
cv::Mat RobotController::GetFinalImageCopy()
{
    std::unique_lock<std::mutex> locker(_imageLock);
    cv::Mat outCopy;
    drawingImage.copyTo(outCopy);
    return outCopy;
}

// Returns the image to do draw overlay info on
cv::Mat& RobotController::GetDrawingImage()
{
    return latestOverlay;
}