#include "Input/Gamepad.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "Odometry/BlobDetection/RobotClassifier.h"
#include "RobotLink.h"
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
                                     overheadCamL_real{},
                                     odometry{overheadCamL_real},
                                     videoSource{overheadCamL_real}
#endif
{
}

RobotController &RobotController::GetInstance()
{
    static RobotController instance;
    return instance;
}

/**
 * Gets the most recent imu data from the robot
 */
IMUData &RobotController::GetIMUData()
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
    static FieldWidget _fieldWidget;
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

    ClockWidget loopClock("Total loop time");
    cv::Mat zeroArray;

    // receive until the peer closes the connection
    int videoID = -1;
    cv::Mat failsafeImage;

    while (true)
    {
        loopClock.markEnd();

        // If the elapsed time is less then the minimum time for this loop, then sleep for the remaining
        double delta_time = MIN_ROBOT_CONTROLLER_LOOP_TIME_MS - loopClock.getElapsedTime() * 1000.0f;
        if (delta_time > 1.0f)
        {
            Sleep((DWORD)delta_time);
        }

        loopClock.markStart();

        // init drawing image to latest frame from camera
        videoSource.GetFrame(drawingImage, 0);

        // Initialize failsafe image
        if( failsafeImage.empty() && !drawingImage.empty())
        { drawingImage.copyTo(failsafeImage);}

        // If the frame is empty then keep old image
        if( drawingImage.empty()) 
        { 
            if( failsafeImage.empty()) {continue;}
            failsafeImage.copyTo(drawingImage);
        }

        // Convert the drawing stop RGB
        if (drawingImage.channels() == 1) {
            cv::cvtColor(drawingImage, drawingImage, cv::COLOR_GRAY2BGR);
        }

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
            locker.unlock();
            _imuCV.notify_all();
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

        ApplyMoveScales(response);
        // send the response to the robot
        robotLink.Drive(response);

        DrawStatusIndicators();

        // update the mat + allow the user to adjust the crop of the field
        _fieldWidget.AdjustFieldCrop();
        _fieldWidget.UpdateMat(drawingImage);
    }
}


long RobotController::GetIMUFrame(IMUData &output, long old_id, double* frameTime)
{
    // Using scoped mutex locker because conditional variable integrates with it
    // This creates and locks the mutex
    std::unique_lock<std::mutex> locker(_imudataMutex);

    while ((_imuID <= 0) || (_imuID <= old_id))
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
    if (abs(response.movement) < 0.07)
    {
        response.movement = 0;
    }

    // deadband the turn
    if (abs(response.turn) < 0.07)
    {
        response.turn = 0;
    }

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

    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = orbitMode.Execute(gamepad);
    // DriveCommand responseAvoid = AvoidMode();

    // start with just manual control
    DriveCommand ret = responseManual;




    // if gamepad pressed dpad up, _orbiting = true
    if (gamepad.GetDpadUp())
    {
        _orbiting = true;
        _killing = false; 
    }

    if (gamepad.GetDpadDown())
    {
        _killing = true;
        _orbiting = false;
    }

    // if there is any turning on the left stick, disable orbiting and killing
    if (abs(gamepad.GetLeftStickX()) > 0.1)
    {
        _killing = false;
        _orbiting = false;
    }


    static bool _orbitingLast = false;
    // start an orbit if we just started orbiting
    if (_orbiting != _orbitingLast)
    {
        if (_orbiting)
        {
            orbitMode.StartOrbit();
        }
        else
        {
            orbitMode.StopOrbit();
        }
    }
    _orbitingLast = _orbiting;

    // draw on drawing image if we are orbiting
    if (_orbiting)
    {
        cv::putText(drawingImage, "Orbiting", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    }
    else if (_killing)
    {
        cv::putText(drawingImage, "Killing", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
    }


    // if we are killing, execute kill mode
    if (_killing)
    {
        DriveCommand responseGoToPoint = killMode.Execute(gamepad);
        ret.turn = responseGoToPoint.turn;
        ret.movement = responseManual.movement * abs(responseGoToPoint.movement);
        _killing = true;
    }
    // if driver wants to evade (left bumper)
    else if (_orbiting)
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

// Returns the image to do draw overlay info on
cv::Mat& RobotController::GetDrawingImage()
{
    return drawingImage;
}

void RobotController::ApplyMoveScales(DriveCommand& command)
{
    // force command to be between -1 and 1
    command.movement = std::max(-1.0f, std::min(1.0f, command.movement));
    command.turn = std::max(-1.0f, std::min(1.0f, command.turn));

    // scale command by the master scales
    command.movement *= MASTER_MOVE_SCALE_PERCENT / 100.0;
    command.turn *= MASTER_TURN_SCALE_PERCENT / 100.0;


    // check if should invert movements
    if (INVERT_MOVEMENT)
    {
        command.movement *= -1;
    }

    if (INVERT_TURN)
    {
        command.turn *= -1;
    }

    // spinner
    command.backWeaponPower *= MAX_BACK_WEAPON_SPEED;
    command.frontWeaponPower *= MAX_FRONT_WEAPON_SPEED;
}

void RobotController::DrawStatusIndicators()
{
    // get the latest can data
    RadioData data = robotLink.GetLastRadioMessage().radioData;

    cv::Scalar color = cv::Scalar(0, 255, 0);
    // draw green circle at top right with text radio if average delay is less than 10
    if (data.averageDelayMS < 20 && data.averageDelayMS >= 0)
    {
        color = cv::Scalar(0, 255, 0);
    }
    else if (data.averageDelayMS < 50 && data.averageDelayMS >= 0)
    {
        color = cv::Scalar(0, 255, 255);
    }
    else
    {
        color = cv::Scalar(0, 0, 255);
    }
    cv::circle(drawingImage, cv::Point(WIDTH - 50, 50), 17, color, -1);
    cv::putText(drawingImage, "Radio", cv::Point(WIDTH - 63, 54), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);


    // check if robotlink transmission is working
    bool transmitterConnected = robotLink.IsTransmitterConnected();

    if (transmitterConnected)
    {
        color = cv::Scalar(0, 255, 0);
    }
    else
    {
        color = cv::Scalar(0, 0, 255);
    }

    cv::circle(drawingImage, cv::Point(WIDTH - 50, 100), 17, color, -1);
    cv::putText(drawingImage, "TX", cv::Point(WIDTH - 57, 104), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);


    // check the gamepad is connected

    bool gamepadConnected = gamepad.IsConnected();

    if (gamepadConnected)
    {
        color = cv::Scalar(0, 255, 0);
    }
    else
    {
        color = cv::Scalar(0, 0, 255);
    }

    cv::circle(drawingImage, cv::Point(WIDTH - 50, 150), 17, color, -1);
    cv::putText(drawingImage, "GP", cv::Point(WIDTH - 57, 154), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);
}