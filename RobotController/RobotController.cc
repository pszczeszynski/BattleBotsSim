#include "Input/Gamepad.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "RobotLink.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include "UIWidgets/IMUWidget.h"
#include "UIWidgets/RobotControllerGUI.h"
#include "imgui.h"
#include "Strategies/Kill.h"
#include "Strategies/RobotMovement.h"
#include "UIWidgets/ClockWidget.h"
#include "Input/InputState.h"
#include "SafeDrawing.h"
#include "DriverStationLog.h"
#define SAVE_VIDEO

int main()
{

    loadGlobalVariablesFromFile(SAVE_FILE_NAME);
    // initialize the robot controller
    RobotController& robotController = RobotController::GetInstance();
    // run the robot controller
    robotController.Run();
    return 0;
}


RobotController::RobotController() : drawingImage(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)),
                                     gamepad2{1},
                                     _logger{},
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
#ifdef SIMULATION
    std::cout << "Running in simulation mode" << std::endl;
#elif defined(VIDEO_FILES)
    std::cout << "Running in video file mode" << std::endl;
#else
    std::cout << "Running in real robot mode" << std::endl;
    robotLink.RegisterLogger(&_logger);
#endif

    // memset last can message
    memset(&_lastCANMessage, 0, sizeof(_lastCANMessage));

#ifdef SAVE_VIDEO
    std::string _logDirectory = _logger.GetLogDirectory();
    _videoWriter = cv::VideoWriter{_logDirectory + "/output0.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(WIDTH, HEIGHT)};
#endif
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
    // return the data
    return ret;
}


long loopCount = 0;

void RobotController::Run()
{
    std::cout << "Starting robot controller..." << std::endl;
    TIMER_INIT
    Clock lastTime;
    lastTime.markStart();

    // For real robot, send a stop drive command to start
#ifndef SIMULATION
    DriverStationMessage stopCommand;
    stopCommand.type = DriverStationMessageType::DRIVE_COMMAND;
    stopCommand.driveCommand = {0, 0, 0, 0, 0};
    stopCommand.resetIMU = false;
    stopCommand.fuseIMU = true;
    robotLink.Drive(stopCommand);
#endif

    std::cout << "Starting odometry threads..." << std::endl;

    // Start the odometry threads
    odometry.Run(OdometryAlg::Blob);
    odometry.Run(OdometryAlg::Heuristic);
    odometry.Run(OdometryAlg::IMU);
    odometry.Run(OdometryAlg::Neural);
    odometry.Run(OdometryAlg::Human);
    odometry.Run(OdometryAlg::NeuralRot);
    odometry.Run(OdometryAlg::OpenCV);


    std::cout << "Starting GUI threads..." << std::endl;

    // Initialize GUI on main thread:
    // run the gui in a separate thread
    std::thread guiThread = std::thread([]()
                                        {
            std::cout << "STARTING GUI" << std::endl;
        
            RobotControllerGUI::GetInstance();

            while (true)
            {
                // update the gui
                RobotControllerGUI::GetInstance().Update();
            } });

    ClockWidget loopClock("Total loop time");
    cv::Mat zeroArray;

    int videoID = -1;


    // receive until the peer closes the connection
    while (true)
    {
        loopCount ++;

        loopClock.markEnd();
        loopClock.markStart();

        // init drawing image to latest frame from camera
        videoID = UpdateDrawingImage();

        // update the gamepad
        gamepad.Update();
        gamepad2.Update();

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
        odometry.Update(videoID);


        // run our robot controller loop
        DriverStationMessage response = RobotLogic();

        // enforce valid ranges and allow scaling down the movement via sliders
        ApplyMoveScales(response);

        // send the response to the robot
        robotLink.Drive(response);

        DrawStatusIndicators();

        // ClickOnHeuristic();

        _trackingWidget.Update();

        // Draw field boundary editor handles before updating the field widget
        _fieldWidget.DrawFieldBoundaryEditor();
        _fieldWidget.UpdateMat(drawingImage);
    }
}




void RobotController::DumpVideo()
{
#ifdef SAVE_VIDEO
    std::unique_lock<std::mutex> locker(_videoWriterMutex);

    _saving = false;

    // if we are saving, stop saving
    _videoWriter.release();

    // increment the video index
    _video_index ++;

    // re-initialize the video writer
    _videoWriter.open(_logger.GetLogDirectory() + "/output" + std::to_string(_video_index) + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(WIDTH, HEIGHT));

    _saving = true;
#endif
}

/**
 * Consumes the next frame from the camera and updates the drawing image
 * Will enforce 3 channels, and if the frame is empty, will return a black image
*/
int RobotController::UpdateDrawingImage()
{
    static Clock videoWriteClock;

    cv::Mat failsafeImage {WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)};

    int retId = -1;
    // check if we have a new frame from the camera
    if (videoSource.NewFrameReady(0))
    {
        // get the LATEST frame from the camera (0 means latest)
        retId = videoSource.GetFrame(drawingImage, 0);
    }


    // If the frame is empty then keep old image
    if (retId == -1 || drawingImage.empty())
    {
        failsafeImage.copyTo(drawingImage);
    }

    // Convert the drawingImage to RGB
    if (drawingImage.channels() == 1)
    {
        cv::cvtColor(drawingImage, drawingImage, cv::COLOR_GRAY2BGR);
    }
    // if save video is enabled, save the frame
#ifdef SAVE_VIDEO
    if (videoWriteClock.getElapsedTime() > 1.0 / 60.0 && _saving)
    {
        std::unique_lock<std::mutex> locker(_videoWriterMutex);

        _videoWriter.write(drawingImage);
        videoWriteClock.markStart();
    }
#endif

    return retId;
}

long RobotController::GetIMUFrame(IMUData &output, long old_id, double* frameTime)
{
    // Using scoped mutex locker because conditional variable integrates with it
    // This creates the locker and locks the mutex
    std::unique_lock<std::mutex> locker(_imudataMutex);

    while ((_imuID <= 0) || (_imuID <= old_id))
    {
        // Unlock mutex, waits until conditional varables is notifed, then it locks mutex again
        if( _imuCV.wait_for(locker, ODO_MUTEX_TIMEOUT) ==  std::cv_status::timeout )
        {
            // Unable to get a new frame, exit
            return -1;
        }
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


Gamepad& RobotController::GetGamepad()
{
    return gamepad;
}

Gamepad& RobotController::GetGamepad2()
{
    return gamepad2;
}

int Sign(double val)
{
    return (0 < val) - (val < 0);
}

/**
 * ManualMode
 * Allows the user to drive the robot manually
 */
DriverStationMessage RobotController::ManualMode()
{
    // drive the robot with the gamepad
    DriveCommand response{0, 0};
    response.movement = gamepad.GetRightStickY() + gamepad2.GetLeftStickY();
    response.turn = gamepad.GetLeftStickX() + gamepad2.GetRightStickX();

    // update the spinner powers
    Weapons& weapons = Weapons::GetInstance();
    weapons.UpdateSpinnerPowers();

    // apply weapon powers
    response.frontWeaponPower = weapons.GetFrontWeaponTargetPower();
    response.backWeaponPower = weapons.GetBackWeaponTargetPower();

    float selfRighterPower = (int) gamepad.GetDpadDown() - (int) gamepad.GetDpadUp();
    selfRighterPower += InputState::GetInstance().IsKeyDown(ImGuiKey_J) - InputState::GetInstance().IsKeyDown(ImGuiKey_U);
    selfRighterPower += (int) gamepad2.GetRightBumper() - (int) gamepad2.GetLeftBumper();
    // control with keyboard if enabled
    if (WASD_ENABLED)
    {
        response.movement += (int) InputState::GetInstance().IsKeyDown(ImGuiKey_W) - (int) InputState::GetInstance().IsKeyDown(ImGuiKey_S);
        response.turn += (int) InputState::GetInstance().IsKeyDown(ImGuiKey_A) - (int) InputState::GetInstance().IsKeyDown(ImGuiKey_D);
    }

    odometry._AdjustAngleWithArrowKeys();

    // control the self righter
    _selfRighter.Move(selfRighterPower, response, drawingImage);

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

    // apply x^2 scaling to the movement
    response.movement = Sign(response.movement) * pow(abs(response.movement), 2);
    response.turn = Sign(response.turn) * pow(abs(response.turn), 2);

    DriverStationMessage ret;
    ret.resetIMU = false;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand = response;






    static bool lastLeftBumper = false;
    static bool lastRightBumper = false;
    // set when you first press left or right bumper
    static float spinTargetAngle = 0;
    static float spinIntermediaryAngle = 0;

    bool leftBumper = gamepad.GetLeftBumper();
    bool rightBumper = gamepad.GetRightBumper();


    // // don't allow auto turning when using orbit or kill modes
    // bool canAutoTurn = true;
    // if ((gamepad.GetLeftStickY() > 0.7 || _guiOrbit) ||
    //     (gamepad.GetLeftStickY() < -0.7 || _guiKill))
    // {
    //     canAutoTurn = false;
    // }
    // //////////////// 180 degrees if left bumper //////////////////
    // if (canAutoTurn && (leftBumper || rightBumper))
    // {
    //     // initiated press
    //     if (!lastLeftBumper && leftBumper)
    //     {
    //         // record the target angle
    //         spinTargetAngle = angle_wrap(odometry.Robot().robotAngle - M_PI);
    //         // record the intermediary angle (90 degrees from target)
    //         spinIntermediaryAngle = angle_wrap(odometry.Robot().robotAngle - M_PI / 2);
    //     }
    //     // initiated press
    //     if (!lastRightBumper && rightBumper)
    //     {
    //         // record the target angle
    //         spinTargetAngle = angle_wrap(odometry.Robot().robotAngle + M_PI);
    //         // record the intermediary angle (90 degrees from target)
    //         spinIntermediaryAngle = angle_wrap(odometry.Robot().robotAngle + M_PI / 2);
    //     }

    //     // default to the final target
    //     float currTargetAngle = spinTargetAngle;

    //     // check delta angle to target
    //     double deltaAngle = angle_wrap(odometry.Robot().robotAngle - spinTargetAngle);

    //     // if we are more than 90 degrees from the target angle
    //     if (abs(deltaAngle) > TO_RAD * 135)
    //     {
    //         // aim at only 90 degrees away
    //         currTargetAngle = spinIntermediaryAngle;
    //     }
    //     // else
    //     // {
    //     //     std::cout << "delta angle: " << deltaAngle << std::endl;
    //     // }

    //     double originalMovement = ret.driveCommand.movement;
    //     cv::Point2f currPoint = odometry.Robot().robotPosition;
    //     cv::Point2f lookAtPoint = currPoint + cv::Point2f{cos(currTargetAngle) * 100, sin(currTargetAngle) * 100};
    //     // draw the points
    //     safe_circle(drawingImage, currPoint, 10, cv::Scalar(0, 255, 0), 2);
    //     safe_circle(drawingImage, lookAtPoint, 10, cv::Scalar(0, 0, 255), 2);

    //     RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    //     ret = RobotMovement::HoldAngle(currPoint,
    //                                    lookAtPoint,
    //                                    KILL_KD_PERCENT,
    //                                    TURN_THRESH_1_DEG_KILL,
    //                                    TURN_THRESH_2_DEG_KILL,
    //                                    MAX_TURN_POWER_PERCENT_KILL,
    //                                    MIN_TURN_POWER_PERCENT_KILL,
    //                                    SCALE_DOWN_MOVEMENT_PERCENT_KILL,
    //                                    direction);
    //     ret.autoDrive.frontWeaponCurrent10 = weapons.GetFrontWeaponTargetPower() * MAX_FRONT_WEAPON_SPEED / 10;
    //     ret.autoDrive.backWeaponCurrent10 = weapons.GetBackWeaponTargetPower() * MAX_BACK_WEAPON_SPEED / 10;
    // }

    lastLeftBumper = leftBumper;
    lastRightBumper = rightBumper;
    return ret;
}

void RobotController::StartForceOrbit()
{
    _guiOrbit = true;
}

void RobotController::StopForceOrbit()
{
    _guiOrbit = false;
}

void RobotController::StartForceKill()
{
    _guiKill = true;
}

void RobotController::StopForceKill()
{
    _guiKill = false;
}

/**
 * RobotLogic
 * The main logic for the robot
 */
DriverStationMessage RobotController::RobotLogic()
{
    // draw arrow in the direction of the robot
    OdometryData odoData =  RobotController::GetInstance().odometry.Robot();


    cv::Point2f robotPos = odoData.robotPosition;
    Angle robotAngle = odoData.GetAngle();
    float robotAnglef = (double) robotAngle;
    cv::Point2f arrowEnd = robotPos + cv::Point2f{cos(robotAnglef), sin(robotAnglef)} * 50;
    safe_arrow(drawingImage, robotPos, arrowEnd, cv::Scalar(0, 0, 255), 2);

    // start with just manual control
    DriverStationMessage ret = ManualMode();
    DriverStationMessage manual = ret;

    // if gamepad pressed left bumper, _orbiting = true
    if (gamepad.GetLeftStickY() > 0.7 || _guiOrbit)
    {
        _orbiting = true;
        _killing = false; 
    }
    else if (gamepad.GetLeftStickY() < -0.7 || _guiKill)
    {
        _killing = true;
        _orbiting = false;
    }
    else
    {
        _killing = false;
        _orbiting = false;
    }

    // draw on drawing image if we are orbiting
    if (_orbiting)
    {
        cv::putText(drawingImage, "A star", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    }
    else if (_killing)
    {
        cv::putText(drawingImage, "Killing", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
    }

    // wonky shit happens if we try to do 180 turns and orbit/kill
    // make sure we can only do one at once
    // if we are killing, execute kill mode
    if (_killing)
    {
        ret = killMode.Execute(gamepad);
        _killing = true;
    }
    else
    {
        DriverStationMessage orbit = aStarMode.Execute(gamepad);
        // if driver wants to evade (left bumper)
        if (_orbiting)
        {
            // orbit around them
            ret = orbit;
            _orbiting = true;
        }
    }

    if (ret.type == AUTO_DRIVE)
    {
        if (manual.type == AUTO_DRIVE)
        {
            ret.autoDrive.frontWeaponCurrent10 = manual.autoDrive.frontWeaponCurrent10;
            ret.autoDrive.backWeaponCurrent10 = manual.autoDrive.backWeaponCurrent10;
        }
        else if (manual.type == DRIVE_COMMAND)
        {
            // convert weapon powers
            ret.autoDrive.frontWeaponCurrent10 = (unsigned char)(manual.driveCommand.frontWeaponPower * MAX_FRONT_WEAPON_SPEED / 10);
            ret.autoDrive.backWeaponCurrent10 = (unsigned char)(manual.driveCommand.backWeaponPower * MAX_BACK_WEAPON_SPEED / 10);
        }
    }
    else if (ret.type == DRIVE_COMMAND)
    {
        ret.driveCommand.frontWeaponPower = weapons.GetFrontWeaponTargetPower();
        ret.driveCommand.backWeaponPower = weapons.GetBackWeaponTargetPower();
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

void RobotController::ApplyMoveScales(DriverStationMessage& msg)
{
    if (msg.type == DriverStationMessageType::DRIVE_COMMAND)
    {
        DriveCommand& command = msg.driveCommand;
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

        if(command.selfRighterPower < 0.1 && command.selfRighterPower > -0.1)
        {
            command.selfRighterDuty = false;
            command.selfRighterPower = SELF_RIGHTER_IDLE_CURRENT/10;
        }
        else
        {
            command.selfRighterDuty = true;
            command.selfRighterPower *= -1;
        }
    }
    else if (msg.type == DriverStationMessageType::AUTO_DRIVE)
    {
        AutoDrive& autoDrive = msg.autoDrive;
        // force command to be between -1 and 1
        autoDrive.movement = std::max(-1.0f, std::min(1.0f, autoDrive.movement));

        // scale command by the master scales
        autoDrive.movement *= MASTER_MOVE_SCALE_PERCENT / 100.0;

        // check if should invert movements
        if (INVERT_MOVEMENT)
        {
            autoDrive.movement *= -1;
        }

        // invert the turn if necessary
        if (INVERT_TURN)
        {
            autoDrive.invertTurn = true;
        }
        
        // weapon speed scaling for auto_drive is applied in robotlogic
        //autoDrive.backWeaponPowerPercent *= MAX_BACK_WEAPON_SPEED;
        //autoDrive.frontWeaponPowerPercent *= MAX_FRONT_WEAPON_SPEED;
    }
}

/**
 * Draws status indicators for each critical part of the robot. This includes:
 * 1. Radio connection (only if transmitter working)
 * 2. Transmitter is connected
 * 3. Gamepad1 is connected
 * 4. Gamepad2 is connected
*/
void RobotController::DrawStatusIndicators()
{
    // get the latest can datax
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
    safe_circle(drawingImage, cv::Point(WIDTH - 50, 50), 17, color, -1);
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

    safe_circle(drawingImage, cv::Point(WIDTH - 50, 100), 17, color, -1);
    cv::putText(drawingImage, "TX", cv::Point(WIDTH - 57, 104), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);

    // check if secondary robotlink transmission is working
    bool secondaryTransmitterConnected = robotLink.IsSecondaryTransmitterConnected();

    if (secondaryTransmitterConnected)
    {
        color = cv::Scalar(0, 255, 0);
    }
    else
    {
        color = cv::Scalar(0, 0, 255);
    }

    safe_circle(drawingImage, cv::Point(WIDTH - 50, 150), 17, color, -1);
    cv::putText(drawingImage, "TX2", cv::Point(WIDTH - 57, 154), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);


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

    safe_circle(drawingImage, cv::Point(WIDTH - 50, 200), 17, color, -1);
    cv::putText(drawingImage, "GP1", cv::Point(WIDTH - 59, 204), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);



    // check if gamepad2 is connected

    bool gamepad2Connected = gamepad2.IsConnected();

    if (gamepad2Connected)
    {
        color = cv::Scalar(0, 255, 0);
    }
    else
    {
        color = cv::Scalar(0, 0, 255);
    }

    safe_circle(drawingImage, cv::Point(WIDTH - 50, 250), 17, color, -1);
    cv::putText(drawingImage, "GP2", cv::Point(WIDTH - 59, 254), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);








    std::string trackText = "";
    // tracking status indicator
    if (odometry.IsTrackingGoodQuality())
    {
        color = cv::Scalar(0, 255, 0);
        trackText = "GOOD TRACK";
    }
    else
    {
        color = cv::Scalar(0, 0, 255);
        trackText = "MID TRACK";
    }

    cv::putText(drawingImage, trackText, cv::Point2f(WIDTH / 2 - 100, HEIGHT * 0.9), 1, 2, color, 3);
}