#include "Input/Gamepad.h"
#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "RobotClassifier.h"
#include "RobotLink.h"
#include "Vision.h"
#include <QApplication>
#include <QMainWindow>
#include <QThread>
#include <opencv2/core.hpp>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    RobotControllerGUI::GetInstance().SetApp(app);

    // Create a separate thread for RobotController
    QThread controllerThread;
    // get instance of RobotController
    RobotController& rc = RobotController::GetInstance();
    rc.moveToThread(&controllerThread);

    // Connect RobotController's signal to RobotControllerGUI's slot
    QObject::connect(&rc, &RobotController::RefreshFieldImageSignal,
                     &RobotControllerGUI::GetInstance(), &RobotControllerGUI::RefreshFieldImage,
                     Qt::QueuedConnection);

    QObject::connect(&controllerThread, &QThread::started, &rc, &RobotController::Run);
    RobotControllerGUI::GetInstance().ShowGUI();

    controllerThread.start();

    app.exec();
}

RobotController::RobotController() :
									drawingImage(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)),
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
 * Gets the most recent message from the robot
*/
RobotMessage& RobotController::GetLatestMessage()
{
    return state;
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

    // receive until the peer closes the connection
    while (true)
    {
        // log the frame rate
        LogFPS();

        // update the gamepad
        gamepad.Update();

        // get the latest classification
        VisionClassification classification = vision.ConsumeLatestClassification(drawingImage);
        UpdateRobotTrackers(classification);

        // receive state info from the robot
        state = robotLink.Receive();

        // run our robot controller loop
        DriveCommand response = RobotLogic();
        
        // send the response to the robot
        robotLink.Drive(response);

        // update the GUI
        GuiLogic();

        if (classification.GetHadNewImage())
        {
            // send the drawing image to the GUI
            ProduceDrawingImage();
        }
    }
}

void RobotController::ProduceDrawingImage()
{
    drawingImageQueue.produce(drawingImage);

    // mark the start of the vision update
    visionClock.markStart();

    // refresh the field image
    emit RefreshFieldImageSignal();
}

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
    }
    else
    {
        // otherwise just update using the imu
        RobotOdometry::Robot().UpdateIMUOnly();
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

/**
 * Uses the error between a currentPos and targetPos
 * and returns a power to drive towards the target at (with 2 thresholds)
 *
 * @param error The target position
 * @param threshold1 The first threshold (full power)
 * @param threshold2 The second threshold (min power)
 * @param minPower The minimum power to drive at
 */
static double DoubleThreshToTarget(double error,
                                   double threshold1, double threshold2,
                                   double minPower, double maxPower)
{
    double distance = std::abs(error);
    double ret = 0;

    if (distance >= threshold1)
    {
        // Move towards the target with full power
        ret = maxPower;
    }
    else if (distance < threshold1 && distance > threshold2)
    {
        // Scale linearly from maxPower to minPower
        ret = ((distance - threshold2) / (threshold1 - threshold2)) * (maxPower - minPower) + minPower;
    }
    else
    {
        // Scale linearly from minPower to 0
        ret = (distance / threshold2) * minPower;
    }

    // invert if we need to
    if (error < 0)
    {
        ret *= -1;
    }

    return ret;
}


/**
 * Drive the robot to a specified position
 * @param targetPos The target position
 * @param chooseNewTarget Whether to choose a new target
 */
DriveCommand RobotController::DriveToPosition(const cv::Point2f &targetPos, bool chooseNewTarget = false)
{
    static Clock c;
    static bool goToOtherTarget = true;

    cv::Point2f currPos = RobotOdometry::Robot().GetPosition();
    double currAngle = RobotOdometry::Robot().GetAngle();

    double deltaTime = c.getElapsedTime();
    c.markStart();

    double currAngleEx = exState.angle;
    cv::Point2f currPosEx = exState.position;

    // draw arrow from our position at our angle
    cv::Point2f arrowEnd = currPos + cv::Point2f(50.0 * cos(currAngle), 50.0 * sin(currAngle));

    cv::arrowedLine(drawingImage, currPos, arrowEnd, cv::Scalar(0, 0, 255), 1);
    // draw our position
    cv::circle(drawingImage, currPos, 5, cv::Scalar(0,0,255), 2);
    // draw circle with dotted line at extrapolated position
    cv::circle(drawingImage, currPosEx, 5, cv::Scalar(255,100,0), 1);

    // draw different colored arrow from our position at extrapolated angle
    cv::Point2f arrowEndEx = currPosEx + cv::Point2f(50.0 * cos(currAngleEx), 50.0 * sin(currAngleEx));
    cv::arrowedLine(drawingImage, currPosEx, arrowEndEx, cv::Scalar(255, 100, 0), 2);



    double angleToTarget1 = atan2(targetPos.y - currPosEx.y, targetPos.x - currPosEx.x);
    double angleToTarget2 = angle_wrap(angleToTarget1 + M_PI);
    double deltaAngleRad1 = angle_wrap(angleToTarget1 - currAngleEx);
    double deltaAngleRad2 = angle_wrap(angleToTarget2 - currAngleEx);
    double deltaAngleRad1_noex = angle_wrap(angleToTarget1 - currAngle);
    double deltaAngleRad2_noex = angle_wrap(angleToTarget2 - currAngle);
    if (chooseNewTarget)
    {
        goToOtherTarget = abs(deltaAngleRad1_noex) < abs(deltaAngleRad2_noex);
    }

    double deltaAngleRad = goToOtherTarget ? deltaAngleRad1 : deltaAngleRad2;

    DriveCommand response{0, 0};
    response.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                                TURN_THRESH_2_DEG * TO_RAD, MIN_TURN_POWER_PERCENT / 100.0, MAX_TURN_POWER_PERCENT / 100.0);


    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(scaleDownMovement, 1.0 - abs(response.turn) * scaleDownMovement) * 1.0;

    response.movement = goToOtherTarget ? drive_scale : -drive_scale;


    // Draw debugging information
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);

    response.turn *= -1;

    return response;
}


double radius = PURE_PURSUIT_RADIUS;
/**
 * OrbitMode
 * Orbits the robot around the opponent at a fixed distance.
 * @param message The current state of the robot
 */
DriveCommand RobotController::OrbitMode()
{
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};

    // our pos + angle
    cv::Point2f ourPosition = RobotOdometry::Robot().GetPosition();

    // opponent pos + angle
    cv::Point2f opponentPos = RobotOdometry::Opponent().GetPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    opponentPosEx = RobotOdometry::Opponent().GetVelocity() * OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 + opponentPos;

    // get the angle from us to the opponent
    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    // add pi to get the angle from the opponent to us
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);


    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, ORBIT_RADIUS, cv::Scalar(255, 0, 0), 1);
    // draw arrow from opponent position at opponent angle
    cv::Point2f arrowEnd = opponentPos + cv::Point2f(100.0 * cos(RobotOdometry::Opponent().GetAngle()), 100.0 * sin(RobotOdometry::Opponent().GetAngle()));
    cv::arrowedLine(drawingImage, opponentPos, arrowEnd, cv::Scalar(255, 0, 0), 2);
    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, ORBIT_RADIUS, cv::Scalar(255, 165, 0), 4);

    // get our velocity
    double velocityNorm = cv::norm(RobotOdometry::Robot().GetVelocity());
    // scale the radius based on our velocity
    double targetRadius = PURE_PURSUIT_RADIUS * velocityNorm / 200.0;
    double distToCenter = cv::norm(ourPosition - opponentPosEx);

    // if (distToCenter < ORBIT_RADIUS)
    // {
    //     targetRadius *= 0.5;
    // }

    // slowly change the radius to the target radius
    radius += (targetRadius - radius) * 0.05;

    // CLIP THE RADIUS TO NEVER BE TOO BIG AS TO GO MORE THAN 180
    double distanceToOtherEdgeOfCircle = distToCenter + ORBIT_RADIUS;
    if (radius > distanceToOtherEdgeOfCircle*0.99)
    {
        radius = distanceToOtherEdgeOfCircle * 0.99;
    }
    // default to the angle from the opponent to us
    cv::Point2f targetPoint = opponentPosEx + cv::Point2f(ORBIT_RADIUS * cos(angleOpponentToUs), ORBIT_RADIUS * sin(angleOpponentToUs));
    std::vector<cv::Point2f> circleIntersections = CirclesIntersect(ourPosition, radius, opponentPosEx, ORBIT_RADIUS);

    // draw circle at our position with radius PURE_PURSUIT_RADIUS_PX
    cv::circle(drawingImage, ourPosition, radius, cv::Scalar(0, 255, 0), 1);

    if (circleIntersections.size() > 0)
    {
        targetPoint = circleIntersections[0];
    }

    // calculate tangent points
    cv::Point2f tangent1;
    cv::Point2f tangent2;
    CalculateTangentPoints(opponentPosEx, ORBIT_RADIUS, ourPosition, tangent1, tangent2);

    double distToTargetPoint = cv::norm(targetPoint - ourPosition);
    double distToOpponent = cv::norm(ourPosition - opponentPosEx);
    double distToTangent1 = cv::norm(tangent1 - ourPosition);
    // if we are really far away from the opponent
    if (distToOpponent > ORBIT_RADIUS * 2)
    {
        // go to the tangent point
        targetPoint = tangent1;
    }

    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    bool allowReverse = false;

    // Drive to the opponent position
    DriveCommand response = DriveToPosition(targetPoint, allowReverse);

    response.movement *= MASTER_SPEED_SCALE_PERCENT / 100.0;
    response.turn *= MASTER_SPEED_SCALE_PERCENT / 100.0;

    return response;
}

#define SECONDS_UNTIL_FULL_POWER 2.0
void RobotController::UpdateSpinnerPowers()
{
    static Clock updateTimer;

    // get the delta time
    double deltaTimeS = updateTimer.getElapsedTime();
    // mark the start of the update
    updateTimer.markStart();
    // reset the delta time if it is too large
    if (deltaTimeS > 10)
    {
        deltaTimeS = 0;
    }

    // if a pressed
    if (gamepad.GetButtonA())
    {
        // invert powers
        _frontWeaponPower += deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // if b pressed
    if (gamepad.GetButtonB())
    {
        // invert powers
        _frontWeaponPower -= deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // if x pressed
    if (gamepad.GetButtonX())
    {
        // invert powers
        _backWeaponPower += deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // if y pressed
    if (gamepad.GetButtonY())
    {
        // invert powers
        _backWeaponPower -= deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // force weapon powers to be between 0 and 1
    if (_frontWeaponPower > 1) { _frontWeaponPower = 1; }
    if (_frontWeaponPower < 0) { _frontWeaponPower = 0; }
    if (_backWeaponPower > 1) { _backWeaponPower = 1; }
    if (_backWeaponPower < 0) { _backWeaponPower = 0; }
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

    return response;
}

/**
 * RobotLogic
 * The main logic for the robot
 */
DriveCommand RobotController::RobotLogic()
{
    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    RobotSimState currentState;
    currentState.position = RobotOdometry::Robot().GetPosition();
    currentState.angle = RobotOdometry::Robot().GetAngle();
    currentState.velocity = RobotOdometry::Robot().GetVelocity();
    currentState.angularVelocity = RobotOdometry::Robot().GetAngleVelocity() * ANGLE_EXTRAPOLATE_MS / POSITION_EXTRAPOLATE_MS;

    exState = robotSimulator.Simulate(currentState, POSITION_EXTRAPOLATE_MS / 1000.0, 50);

    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = OrbitMode();


    DriveCommand ret = responseManual;

    responseManual.movement = 1.0;

    if (true || gamepad.GetLeftBumper())
    {
        ret.turn = responseOrbit.turn;
        ret.movement = responseManual.movement * responseOrbit.movement;
    }

    if (gamepad.GetRightBumper())
    {
        DriveCommand responseGoToPoint = DriveToPosition(RobotOdometry::Opponent().GetPosition(), false);
        ret.turn = responseGoToPoint.turn;
    }

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

    // 1. update imu display
    IMUWidget::GetInstance().Update();

    const double CORNER_DIST_THRESH = 20.0;

    Input &input = Input::GetInstance();
    // get the curr mouse position
    cv::Point2f currMousePos = input.GetMousePosition();

    // if the user isn't pressing shift
    if (!input.IsKeyPressed(Qt::Key_Shift))
    {
        // corner adjustment

        // If the user left clicks near one of the corners
        if (input.IsLeftMousePressed())
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
            if (input.IsLeftMousePressed())
            {
                // set the robot to the mouse position
                RobotOdometry::Robot().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }

            // if the user right clicks
            if (input.IsRightMousePressed())
            {
                // set the opponent to the mouse position
                RobotOdometry::Opponent().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }
        }
    }
    else // else the user is pressing shift
    {
        // if the user presses the left mouse button with shift
        if (input.IsLeftMousePressed())
        {
            // set the robot angle
            cv::Point2f currMousePos = input.GetMousePosition();
            cv::Point2f robotPos = RobotOdometry::Robot().GetPosition();
            double newAngle = atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
            RobotOdometry::Robot().UpdateForceSetAngle(newAngle);
        }
    }

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