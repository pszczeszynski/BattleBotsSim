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
#include <algorithm>

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

#ifdef SAVE_VIDEO
    // Initialize VideoWriter
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // or use another codec
    double fps = 60.0;                                        // you can adjust this according to your needs
    cv::Size frameSize(drawingImage.cols, drawingImage.rows);
    cv::VideoWriter video("Recordings/outputVideo.avi", fourcc, fps, frameSize, true); // 'true' for color video
#endif

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
        // update the robot tracker positions
        UpdateRobotTrackers(classification);

        // run our robot controller loop
        DriveCommand response = RobotLogic();
        
        // send the response to the robot
        robotLink.Drive(response);

#ifdef SAVE_VIDEO
        if (!drawingImage.empty())
        {
            // write the frame to the video
            video.write(drawingImage);
        }
#endif

        // update the GUI
        GuiLogic();

        // if there was a new image
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

        cv::rectangle(drawingImage, robot.rect, cv::Scalar(255, 0, 0, 1));
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

    goToOtherTarget = gamepad.GetRightStickY() > 0;

    double deltaAngleRad = goToOtherTarget ? deltaAngleRad1 : deltaAngleRad2;

    DriveCommand response{0, 0};
    response.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                                TURN_THRESH_2_DEG * TO_RAD, MIN_TURN_POWER_PERCENT / 100.0, MAX_TURN_POWER_PERCENT / 100.0);


    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(0.0, 1.0 - abs(response.turn / (MAX_TURN_POWER_PERCENT/ 100.0)) * scaleDownMovement) * 1.0;

    response.movement = goToOtherTarget ? drive_scale : -drive_scale;

    // Draw debugging information
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);

    response.turn *= -1;

    return response;
}


/**
 * OrbitMode
 * Orbits the robot around the opponent at a fixed distance.
 * @param message The current state of the robot
 */
#define MAX_PURE_PURSUIT_RADIUS_SCALE 3.0
#define MIN_PURE_PURSUIT_RADIUS_SCALE 0.5
#define USE_TANGENT_POINTS_DIST ORBIT_RADIUS * 2

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

double RobotController::_CalculateOrbitRadius(cv::Point2f opponentPosEx)
{
    // our pos + angle
    cv::Point2f ourPosition = RobotOdometry::Robot().GetPosition();

    double orbitRadius = ORBIT_RADIUS;
    double distToOpponent = cv::norm(ourPosition - opponentPosEx);

    // scale the radius based on the triggers
    orbitRadius *= 1.0 + gamepad.GetRightTrigger();
    orbitRadius /= 1.0 + gamepad.GetLeftTrigger();

    // // grow orbit radius the further we are away
    // if (distToOpponent > orbitRadius)
    // {
    //     orbitRadius += (distToOpponent - orbitRadius) * 0.1;
    // }

    return orbitRadius;
}

/**
 * Makes sure the currentTargetPoint doesn't go more into the circle than the tangent point
*/
cv::Point2f RobotController::_NoMoreAggressiveThanTangent(cv::Point2f ourPosition, cv::Point2f opponentPosEx, double orbitRadius, cv::Point2f currentTargetPoint, bool circleDirection)
{
    double distToOpponent = cv::norm(ourPosition - opponentPosEx);

    bool direction = gamepad.GetRightStickY() >= 0;

    // if we are inside the circle, then just return the currentTargetPoint since can't take tangent
    if (distToOpponent <= orbitRadius)
    {
        return currentTargetPoint;
    }

    // calculate tangent points
    cv::Point2f tangent1;
    cv::Point2f tangent2;
    CalculateTangentPoints(opponentPosEx, orbitRadius, ourPosition, tangent1, tangent2);

    if (!circleDirection)
    {
        // swap
        cv::Point2f temp = tangent1;
        tangent1 = tangent2;
        tangent2 = temp;
    }


    double distToTangent1 = cv::norm(tangent1 - ourPosition);

    // calculate angle from opponent to us
    double angleOpponentToUs = atan2(ourPosition.y - opponentPosEx.y, ourPosition.x - opponentPosEx.x);
    // calculate angle from opponent to currentTargetPoint
    double angleOpponentToCurrTarget = atan2(currentTargetPoint.y - opponentPosEx.y, currentTargetPoint.x - opponentPosEx.x);
    // calculate angle from opponent to tangent1
    double angleOpponentToTangent1 = atan2(tangent1.y - opponentPosEx.y, tangent1.x - opponentPosEx.x);


    // if the tangent point is less aggressive than the currentTargetPoint
    bool lessAggressive = angle_wrap(angleOpponentToCurrTarget - angleOpponentToUs) < angle_wrap(angleOpponentToTangent1 - angleOpponentToUs);
    // flip use if applying negative power
    if (!circleDirection)
    {
        lessAggressive = !lessAggressive;
    }
    
    // if the tangent point is less aggressive than the currentTargetPoint
    if (lessAggressive)
    {
        // then use the tangent point
        currentTargetPoint = tangent1;
    }

    return currentTargetPoint;
}

DriveCommand RobotController::OrbitMode()
{
    static double purePursuitRadius = PURE_PURSUIT_RADIUS;
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};

    // our pos + angle
    cv::Point2f ourPosition = RobotOdometry::Robot().GetPosition();
    double ourAngle = RobotOdometry::Robot().GetAngle();

    // opponent pos + angle
    cv::Point2f opponentPos = RobotOdometry::Opponent().GetPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    opponentPosEx = RobotOdometry::Opponent().GetVelocity() * OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 + opponentPos;

    double orbitRadius = _CalculateOrbitRadius(opponentPosEx);

    // get the angle from us to the opponent
    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    // add pi to get the angle from the opponent to us
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);

    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, orbitRadius, cv::Scalar(255, 0, 0), 1);
    // // draw arrow from opponent position at opponent angle
    // cv::Point2f arrowEnd = opponentPos + cv::Point2f(100.0 * cos(RobotOdometry::Opponent().GetAngle()), 100.0 * sin(RobotOdometry::Opponent().GetAngle()));
    // cv::arrowedLine(drawingImage, opponentPos, arrowEnd, cv::Scalar(255, 0, 0), 2);
    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, orbitRadius, cv::Scalar(255, 165, 0), 4);

    // get our velocity
    double velocityNorm = cv::norm(RobotOdometry::Robot().GetVelocity());
    // scale the radius based on our velocity
    double targetPurePursuitRadius = PURE_PURSUIT_RADIUS * velocityNorm / 200.0;
    // calculate distance to the center of the circle
    double distToCenter = cv::norm(ourPosition - opponentPosEx);
    // the radius shouldn't be larger than the distance to the other edge of the circle
    double distanceToOtherEdgeOfCircle = distToCenter + orbitRadius;
    // enforce the targetRadius to be between the min and max scales
    targetPurePursuitRadius = std::min(targetPurePursuitRadius, PURE_PURSUIT_RADIUS * MAX_PURE_PURSUIT_RADIUS_SCALE);
    targetPurePursuitRadius = std::max(targetPurePursuitRadius, PURE_PURSUIT_RADIUS * MIN_PURE_PURSUIT_RADIUS_SCALE);
    targetPurePursuitRadius = std::min(targetPurePursuitRadius, distanceToOtherEdgeOfCircle - 5);
    // slowly change the radius to the target radius
    purePursuitRadius += (targetPurePursuitRadius - purePursuitRadius) * (ORBIT_RADIUS_MOVAVG_SPEED / 100.0);
    // re-enforce the smoothed radius to not engulf the circle
    purePursuitRadius = std::min(purePursuitRadius, distanceToOtherEdgeOfCircle - 5);

    // default the target to be radially from the angle from the opponent to us
    cv::Point2f targetPoint = opponentPosEx + cv::Point2f(orbitRadius * cos(angleOpponentToUs), orbitRadius * sin(angleOpponentToUs));

    // next find the intersection of the pure pursuit circle with the circle around the opponent
    std::vector<cv::Point2f> circleIntersections = CirclesIntersect(ourPosition, purePursuitRadius, opponentPosEx, orbitRadius);

    // draw circle at our position with radius PURE_PURSUIT_RADIUS_PX
    cv::circle(drawingImage, ourPosition, purePursuitRadius, cv::Scalar(0, 255, 0), 1);



    bool circleDirection = angle_wrap(ourAngle - angleToOpponent) < 0;

    if (gamepad.GetRightStickY() < 0)
    {
        circleDirection = !circleDirection;
    }

    // if there are 2 intersections
    if (circleIntersections.size() >= 2)
    {
        // then use the first intersection
        targetPoint = circleDirection ? circleIntersections[0] : circleIntersections[1];
    }
    else if (circleIntersections.size() == 1)
    {
        // if there is only 1 intersection, then use that
        targetPoint = circleIntersections[0];
    }
    

    // // enforce that the target point is not more aggressive than the tangent point towards the center of the circle
    targetPoint = _NoMoreAggressiveThanTangent(ourPosition, opponentPosEx, orbitRadius, targetPoint, circleDirection);

    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    // Drive to the opponent position
    bool allowReverse = false;
    DriveCommand response = DriveToPosition(targetPoint, allowReverse);

    // drawAndDisplayValue(drawingImage, cv::norm(RobotOdometry::Robot().GetVelocity()), 50, cv::Scalar(0, 255, 0));

    // // draw the result response.movement
    // drawAndDisplayValue(drawingImage, response.movement, WIDTH / 2, cv::Scalar(255, 0, 0));

    return response;
}

#define SECONDS_UNTIL_FULL_POWER 8.0
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

    double scaleFront = _frontWeaponPower < 0.1 ? 1 : (_frontWeaponPower < 0.5 ? 2 : 1);
    double scaleBack = _backWeaponPower < 0.1 ? 1 : (_backWeaponPower < 0.5 ? 2 : 1);

    // if a pressed
    if (gamepad.GetButtonA() || Input::GetInstance().IsKeyPressed(Qt::Key_W))
    {
        // invert powers
        _frontWeaponPower += scaleFront * deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // if b pressed
    if (gamepad.GetButtonB() || Input::GetInstance().IsKeyPressed(Qt::Key_S))
    {
        // invert powers
        _frontWeaponPower -= scaleFront * deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // if x pressed
    if (gamepad.GetButtonX() || Input::GetInstance().IsKeyPressed(Qt::Key_I))
    {
        // invert powers
        _backWeaponPower += scaleBack * deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    // if y pressed
    if (gamepad.GetButtonY() || Input::GetInstance().IsKeyPressed(Qt::Key_K))
    {
        // invert powers
        _backWeaponPower -= scaleBack * deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    }

    if (Input::GetInstance().IsKeyPressed(Qt::Key_9))
    {
        _frontWeaponPower = 0;
    }

    if (Input::GetInstance().IsKeyPressed(Qt::Key_0))
    {
        _backWeaponPower = 0;
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

    float power = (int) gamepad.GetDpadDown() - (int) gamepad.GetDpadUp();
    power += Input::GetInstance().IsKeyPressed(Qt::Key_O) - Input::GetInstance().IsKeyPressed(Qt::Key_L);

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

    response.turn *= 0.6;

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
    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    RobotSimState currentState;
    currentState.position = RobotOdometry::Robot().GetPosition();
    currentState.angle = RobotOdometry::Robot().GetAngle();
    currentState.velocity = RobotOdometry::Robot().GetVelocity();
    double angleExtrapolate = _orbiting ? ORBIT_ANGLE_EXTRAPOLATE_MS : GTP_ANGLE_EXTRAPOLATE_MS;
    currentState.angularVelocity = RobotOdometry::Robot().GetAngleVelocity() * angleExtrapolate / POSITION_EXTRAPOLATE_MS;

    // predict where the robot will be in a couple milliseconds
    exState = robotSimulator.Simulate(currentState, POSITION_EXTRAPOLATE_MS / 1000.0, NUM_PREDICTION_ITERS);

    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = OrbitMode();

    // start with just manual control
    DriveCommand ret = responseManual;

    _orbiting = false;
    _killing = false;

    // if the user activates kill mode
    if (gamepad.GetRightBumper())
    {
        // drive directly to the opponent
        DriveCommand responseGoToPoint = DriveToPosition(RobotOdometry::Opponent().GetPosition(), true);
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

    // enforce the max speed
    ret.movement *= MASTER_MOVE_SCALE_PERCENT / 100.0;
    ret.turn *= MASTER_TURN_SCALE_PERCENT / 100.0;

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
            if (input.IsLeftMousePressed() && input.IsMouseOverImage())
            {
                // set the robot to the mouse position
                RobotOdometry::Robot().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }

            // if the user right clicks
            if (input.IsRightMousePressed() && input.IsMouseOverImage())
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