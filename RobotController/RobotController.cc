#include "Input/Gamepad.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "RobotClassifier.h"
#include "RobotLink.h"
#include "Vision.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include "UIWidgets/IMUWidget.h"
#include "UIWidgets/RobotControllerGUI.h"
#include "imgui.h"
#include "Strategies/Orbit.h"
#include "Strategies/Kill.h"
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
                                     vision{overheadCamL_sim}
#else
                                     overheadCamL_real{},
                                     vision{overheadCamL_real}
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

#define SAVE_VIDEO

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

#ifdef SAVE_VIDEO
    static cv::VideoWriter _videoWriter{"Recordings/output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(WIDTH, HEIGHT)};
#endif
    Clock videoWriteClock;

    // receive until the peer closes the connection
    while (true)
    {
        loopClock.markEnd();
        loopClock.markStart();

        // update the gamepad
        gamepad.Update();

        // receive the latest message
        RobotMessage msg = robotLink.Receive();

        // save the specific type information in a last struct
        if (msg.type == RobotMessageType::IMU_DATA)
        {
            _lastIMUMessage = msg;
        }
        else if (msg.type == RobotMessageType::CAN_DATA)
        {
            _lastCanMessageMutex.lock();
            _lastCANMessage = msg;
            _lastCanMessageMutex.unlock();
        }

        // get the latest classification (very fast)
        VisionClassification classification = vision.ConsumeLatestClassification(drawingImage);

        // update the robot tracker positions
        UpdateRobotTrackers(classification);

        // run our robot controller loop
        DriveCommand response = RobotLogic();

        ApplyMoveScales(response);

        // send the response to the robot
        robotLink.Drive(response);

        DrawStatusIndicators();

        // update the mat + allow the user to adjust the crop of the field
        _fieldWidget.AdjustFieldCrop();
        _fieldWidget.UpdateMat(drawingImage);


    
    // if save video is enabled, save the frame
#ifdef SAVE_VIDEO
        if (videoWriteClock.getElapsedTime() > 1.0 / 60.0)
        {
            _videoWriter.write(drawingImage);
            videoWriteClock.markStart();
        }
#endif
    }
}

/**
 * Takes a vision classification result and updates the robot positions.
 * This function is called every frame. The classification might have nothing new,
 * in which case only imu is used.
 * 
 * @param classification The vision classification result
*/
void RobotController::UpdateRobotTrackers(VisionClassification classification)
{
    static int updatesWithoutOpponent = 0;

    // if we should use the rotation network
    if (ROTATION_NET_ENABLED)
    {
        // use the ml model to get the angle entirely
        RobotOdometry::Robot().UpdateForceSetAngle(CVRotation::GetInstance().ComputeRobotRotation(drawingImage, RobotOdometry::Robot().GetPosition()));
    }

    // if we didn't get a new image, don't update the robot trackers
    if (!classification.GetHadNewImage())
    {
        return;
    }

    // if vision detected our robot
    if (classification.GetRobotBlob() != nullptr)
    {
        MotionBlob robot = *classification.GetRobotBlob();
        cv::Mat& frame = *(classification.GetRobotBlob()->frame);

        RobotOdometry::Robot().UpdateVisionAndIMU(robot, frame);

        cv::rectangle(drawingImage, robot.rect, cv::Scalar(0, 255, 0, 1));
    }
    else
    {
        // otherwise just update using the imu
        // TODO: DONT JUST USE THE DRAWING IMAGE, that's hacky
        RobotOdometry::Robot().UpdateIMUOnly(drawingImage);

        // draw red circle on robot since we didn't detect it
        cv::circle(drawingImage, RobotOdometry::Robot().GetPosition(), 10, cv::Scalar(0, 0, 255), 4);
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
    cv::Point2f robotPos = RobotOdometry::Robot().GetPosition();
    Angle robotAngle = RobotOdometry::Robot().GetAngle();
    float robotAnglef = (double) robotAngle;
    cv::Point2f arrowEnd = robotPos + cv::Point2f{cos(robotAnglef), sin(robotAnglef)} * 50;
    cv::arrowedLine(drawingImage, robotPos, arrowEnd, cv::Scalar(0, 0, 255), 2);

    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = orbitMode.Execute(gamepad);
    // start with just manual control
    DriveCommand ret = responseManual;


    // if gamepad pressed left bumper, _orbiting = true
    if (gamepad.GetLeftBumper() && !gamepad.GetRightBumper())
    {
        _orbiting = true;
        _killing = false; 
    }
    else if (gamepad.GetRightBumper() && !gamepad.GetLeftBumper())
    {
        _killing = true;
        _orbiting = false;
    }

    LEAD_WITH_BAR = gamepad.GetRightStickY() >= 0.0;
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