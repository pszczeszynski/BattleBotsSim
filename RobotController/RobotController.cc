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
#include "Strategies/Avoid.h"
#include "Strategies/RobotMovement.h"
#include "UIWidgets/FieldWidget.h"

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
                                     overheadCamL_real{1},
                                     vision{overheadCamL_real}
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

    frontWeaponCurrRPMPercent = ret.motorRPM[2] / 88.0f;
    backWeaponCurrRPMPercent = ret.motorRPM[3] / 88.0f;
    // return the data
    return ret;
}

#define FPS_PRINT_TIME_SECONDS 5
static void LogFPS()
{
    static int frames = 0;
    static Clock c;
    // get the elapsed time
    double elapsed = c.getElapsedTime();

    // if enough time has passed, print the fps
    if (elapsed > FPS_PRINT_TIME_SECONDS && frames > 0)
    {
        std::cout << "fps: " << frames / elapsed << std::endl;
        frames = 0;
        c.markStart();
    }
    frames ++;
}

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

    RobotControllerGUI::GetInstance();

    // receive until the peer closes the connection
    while (true)
    {
        // log the frame rate
        LogFPS();

        // update the gamepad
        gamepad.Update();

        // receive the latest message
        RobotMessage msg = robotLink.Receive();
        // save the last message type
        _lastMessageType = msg.type;

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

        // in simulation, add a 5 millisecond wait and continue if we don't get a new image
#ifdef SIMULATION
        if (!classification.GetHadNewImage())
        {
            Sleep(5);
            continue;
        }
#endif

        // update the robot tracker positions
        UpdateRobotTrackers(classification);

        // run our robot controller loop
        DriveCommand response = RobotLogic();
        
        // send the response to the robot
        robotLink.Drive(response);

        // update the GUI
        GuiLogic();

        // update the gui system
        RobotControllerGUI::GetInstance().Update();
    }

    RobotControllerGUI::GetInstance().Shutdown();
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
    state.position = RobotOdometry::Robot().GetPosition();
    state.angle = RobotOdometry::Robot().GetAngle();
    state.velocity = RobotOdometry::Robot().GetVelocity();
    state.angularVelocity = RobotOdometry::Robot().GetAngleVelocity();

    // drive towards it
    ret = RobotMovement::DriveToPosition(state, targetPoint, RobotMovement::DriveDirection::Auto);

    return ret;
}


#define SECONDS_UNTIL_FULL_POWER 15.6
void RobotController::UpdateSpinnerPowers()
{
    static Clock updateTimer;
    static bool rampUpF = false;
    static bool rampUpB = false;
    static bool overrideRampLimit = false;

    // get the delta time
    double deltaTimeS = updateTimer.getElapsedTime();
    // mark the start of the update
    updateTimer.markStart();
    // reset the delta time if it is too large
    if (deltaTimeS > 10)
    {
        deltaTimeS = 0;
    }

    double scaleFront = _frontWeaponPower < 0.1 ? 1 : (_frontWeaponPower < 0.55 ? 2.2 : 1.2);
    double scaleBack = _backWeaponPower < 0.1 ? 1 : (_backWeaponPower < 0.55 ? 2.2 : 1.2);

    // if a pressed
    if (gamepad.GetButtonA() || ImGui::IsKeyDown(ImGuiKey_W))
    {
        rampUpB = true;
    }

    // if b pressed
    if (gamepad.GetButtonB() || ImGui::IsKeyDown(ImGuiKey_S))
    {
        rampUpB = false;
    }

    // if x pressed
    if (gamepad.GetButtonX() || ImGui::IsKeyDown(ImGuiKey_I))
    {
        rampUpF = true;
    }

    // if y pressed
    if (gamepad.GetButtonY() || ImGui::IsKeyDown(ImGuiKey_K))
    {
        rampUpF = false;
    }

    // if user toggles off the weapon, then set the power to 0
    if (ImGui::IsKeyDown(ImGuiKey_9))
    {
        if (rampUpF)
        {
            _frontWeaponPower = std::min(_frontWeaponPower, 0.55f);
        }
        else
        {
            _frontWeaponPower = 0;
        }
    }

    // if user toggles off the weapon, then set the power to 0
    if (ImGui::IsKeyDown(ImGuiKey_0))
    {
        if (rampUpF)
        {
            _backWeaponPower = std::min(_backWeaponPower, 0.55f);
        }
        else
        {
            _backWeaponPower = 0;
        }
    }

    if (ImGui::IsKeyDown(ImGuiKey_O))
    {
        overrideRampLimit = true;
    }

    if (ImGui::IsKeyDown(ImGuiKey_L))
    {
        overrideRampLimit = false;
    }

    // if we are ramping up and past 50% power with no curren (because of a fault), then keep it at 50%
    if (rampUpF)
    {
        _lastCanMessageMutex.lock();
        bool hasNoCurrent = _lastCANMessage.canData.motorCurrent[2] == 0.0f;
        _lastCanMessageMutex.unlock();

        // if we have no current and we are ramping up, back off to the current rpm percent (mins a bit)
        if (hasNoCurrent && !overrideRampLimit && _frontWeaponPower > 0.5)
        {
            _frontWeaponPower = std::max(0.5, frontWeaponCurrRPMPercent - 0.02);
        }
    }

    // same thing for the other one
    if (rampUpB)
    {
        _lastCanMessageMutex.lock();
        bool hasNoCurrent = _lastCANMessage.canData.motorCurrent[3] == 0.0f;
        _lastCanMessageMutex.unlock();

        // if we have no current and we are ramping up, back off to the current rpm percent (minus a bit)
        if (hasNoCurrent && !overrideRampLimit && _backWeaponPower > 0.5)
        {
            _backWeaponPower = std::max(0.5, backWeaponCurrRPMPercent - 0.02);
        }
    }

    _frontWeaponPower += (rampUpF ? 1 : -1) * scaleFront * deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    _backWeaponPower += (rampUpB ? 1 : -1) * scaleBack * deltaTimeS / SECONDS_UNTIL_FULL_POWER;

    // force weapon powers to be between 0 and 1
    if (_frontWeaponPower > 1) { _frontWeaponPower = 1; }
    if (_frontWeaponPower < 0) { _frontWeaponPower = 0; }
    if (_backWeaponPower > 1) { _backWeaponPower = 1; }
    if (_backWeaponPower < 0) { _backWeaponPower = 0; }
}


/**
 * Allows the spacebar to control switching the robot positions should the trackers swap
*/
void SpaceSwitchesRobots()
{
    static bool spacePressedLast = false;
    bool spacePressed = ImGui::IsKeyDown(ImGuiKey_Space);

    // if the space bar was just pressed down
    if (spacePressed && !spacePressedLast)
    {
        RobotClassifier::instance->SwitchRobots();
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
    UpdateSpinnerPowers();

    // apply weapon powers
    response.frontWeaponPower = _frontWeaponPower;
    response.backWeaponPower = _backWeaponPower;

    float power = (int) gamepad.GetDpadDown() - (int) gamepad.GetDpadUp();
    power += ImGui::IsKeyDown(ImGuiKey_J) - ImGui::IsKeyDown(ImGuiKey_U);

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

    // enforce the max speed
    response.movement *= MASTER_MOVE_SCALE_PERCENT / 100.0;
    response.turn *= MASTER_TURN_SCALE_PERCENT / 100.0;

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
    cv::Point2f robotPos = RobotOdometry::Robot().GetPosition();
    Angle robotAngle = RobotOdometry::Robot().GetAngle();
    float robotAnglef = (double) robotAngle;
    cv::Point2f arrowEnd = robotPos + cv::Point2f{cos(robotAnglef), sin(robotAnglef)} * 50;
    cv::arrowedLine(drawingImage, robotPos, arrowEnd, cv::Scalar(0, 0, 255), 2);

    Orbit orbitMode = Orbit{};
    Kill killMode = Kill{};
    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = orbitMode.Execute(gamepad);
    DriveCommand responseAvoid = AvoidMode();

    // start with just manual control
    DriveCommand ret = responseManual;

    _orbiting = false;
    _killing = false;

    // if the user activates kill mode
    if (gamepad.GetRightBumper())
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

void RobotController::GuiLogic()
{
    static int cornerToAdjust = -1;
    static cv::Point2f mousePosLast = cv::Point2f(0, 0);
    static cv::Point2f cornerHandles[4] = {cv::Point2f(0, 0),
                                           cv::Point2f(WIDTH, 0),
                                           cv::Point2f(WIDTH, HEIGHT),
                                           cv::Point2f(0, HEIGHT)};
    static Clock updateClock;

    // 1. update imu display
    // IMUWidget::GetInstance().Update();

    const double CORNER_DIST_THRESH = 20.0;

    // get the curr mouse position
    cv::Point2f currMousePos = FieldWidget::GetInstance().GetMousePos();

    // if the user isn't pressing shift
    if (!ImGui::IsKeyDown(ImGuiKey_LeftShift))
    {
        // corner adjustment

        // If the user left clicks near one of the corners
        if (ImGui::IsMouseDown(0))
        {
            if (cornerToAdjust == -1)
            {
                // Check each corner
                for (int i = 0; i < 4; i++)
                {
                    // if the user is near a corner
                    if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH)
                    {
                        // set the corner to adjust
                        cornerToAdjust = i;
                        break;
                    }
                }
            }
        }
        else
        {
            // otherwise set the corner to adjust to -1
            cornerToAdjust = -1;
        }

        // adjust the corner
        cv::Point2f adjustment = mousePosLast - currMousePos;

        if (cornerToAdjust == 0)
        {
            preprocess_tl_x += adjustment.x;
            preprocess_tl_y += adjustment.y;
        }
        else if (cornerToAdjust == 1)
        {
            preprocess_tr_x += adjustment.x;
            preprocess_tr_y += adjustment.y;
        }
        else if (cornerToAdjust == 2)
        {
            preprocess_br_x += adjustment.x;
            preprocess_br_y += adjustment.y;
        }
        else if (cornerToAdjust == 3)
        {
            preprocess_bl_x += adjustment.x;
            preprocess_bl_y += adjustment.y;
        }
        else
        {
            // robot tracker calibration
            // if the user left clicks, aren't pressing shift, and are over the image, and not near a corner
            if (ImGui::IsMouseDown(0))// && input.IsMouseOverImage())
            {
                // set the robot to the mouse position
                RobotOdometry::Robot().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }

            // if the user right clicks
            if (ImGui::IsMouseDown(1))// && input.IsMouseOverImage())
            {
                // set the opponent to the mouse position
                RobotOdometry::Opponent().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }
        }
    }
    else // else the user is pressing shift
    {
        // if the user presses the left mouse button with shift
        if (ImGui::IsMouseDown(0))
        {
            // set the robot angle
            cv::Point2f robotPos = RobotOdometry::Robot().GetPosition();
            double newAngle = atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
            RobotOdometry::Robot().UpdateForceSetAngle(newAngle);
        }
    }

    // allow the user to use the arrow keys to adjust the robot angle
    if (ImGui::IsKeyDown(ImGuiKey_LeftArrow))
    {
        RobotOdometry::Robot().UpdateForceSetAngle(Angle(RobotOdometry::Robot().GetAngle() - updateClock.getElapsedTime() * 30 * M_PI / 180.0));
    }

    if (ImGui::IsKeyDown(ImGuiKey_RightArrow))
    {
        RobotOdometry::Robot().UpdateForceSetAngle(Angle(RobotOdometry::Robot().GetAngle() + updateClock.getElapsedTime() * 30 * M_PI / 180.0));
    }
    updateClock.markStart();


    // SAFE_DRAW
    // cv::circle(drawingImage, mousePosLast, 3, cv::Scalar(0, 255, 0), 2);
    // END_SAFE_DRAW

    for (int i = 0; i < 4; i++)
    {
        if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH)
        {
            cv::circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH * 1.5, cv::Scalar(255, 100, 255), 2);
        }
        else
        {
            cv::circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH, cv::Scalar(255, 0, 255), 2);
        }
    }

    // save the last mouse position
    mousePosLast = currMousePos;
}

float& RobotController::GetFrontWeaponTargetPowerRef()
{
    return _frontWeaponPower;
}

float& RobotController::GetBackWeaponTargetPowerRef()
{
    return _backWeaponPower;
}

IRobotLink& RobotController::GetRobotLink()
{
    return robotLink;
}

cv::Mat& RobotController::GetDrawingImage()
{
    return drawingImage;
}