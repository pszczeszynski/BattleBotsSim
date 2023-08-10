#include "Gamepad.h"
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
    RobotConfigWindow::GetInstance().SetApp(app);

    // Create a separate thread for RobotController
    QThread controllerThread;
    // get instance of RobotController
    RobotController& rc = RobotController::GetInstance();
    rc.moveToThread(&controllerThread);

    // Connect RobotController's signal to RobotConfigWindow's slot
    QObject::connect(&rc, &RobotController::RefreshFieldImageSignal,
                     &RobotConfigWindow::GetInstance(), &RobotConfigWindow::RefreshFieldImage,
                     Qt::QueuedConnection);

    QObject::connect(&controllerThread, &QThread::started, &rc, &RobotController::Run);
    RobotConfigWindow::GetInstance().ShowGUI();

    controllerThread.start();

    app.exec();
}

RobotController::RobotController() : gamepad{0},
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

        TIMER_START
        VisionClassification classification = vision.ConsumeLatestClassification();
        UpdateRobotTrackers(classification);
        TIMER_PRINT("Vision")

        // receive state info from the robot
        state = robotLink.Receive();

        // 3. run our robot controller loop
        TIMER_START
        DriveCommand response = RobotLogic();
        TIMER_PRINT("RobotLogic")

        TIMER_START
        // send the response to the robot
        robotLink.Drive(response);
        TIMER_PRINT("Drive")

        DRAWING_IMAGE_MUTEX.lock();

        if (classification.GetHadNewImage())
        {
            // mark that we can't draw again until we get a new image
            TIMER_START
            // refresh the field image
            emit RefreshFieldImageSignal();
            TIMER_PRINT("imshow()")

            CAN_DRAW = false;
        }

        DRAWING_IMAGE_MUTEX.unlock();
    }
}

void RobotController::UpdateRobotTrackers(VisionClassification classification)
{
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
    }
    else
    {
        // set the opponent to invalid (sets their velocity to 0)
        RobotOdometry::Opponent().Invalidate();
    }

    // if the user presses the left mouse button with shift
    if (Mouse::GetInstance().GetLeftDown() && shiftDown)
    {
        cv::Point2f currMousePos = Mouse::GetInstance().GetPos();
        cv::Point2f robotPos = RobotOdometry::Robot().GetPosition();
        double newAngle = atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
        RobotOdometry::Robot().UpdateForceSetAngle(newAngle);
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

    SAFE_DRAW
    cv::arrowedLine(drawingImage, currPos, arrowEnd, cv::Scalar(0, 0, 255), 1);
    // draw our position
    cv::circle(drawingImage, currPos, 5, cv::Scalar(0,0,255), 2);
    // draw circle with dotted line at extrapolated position
    cv::circle(drawingImage, currPosEx, 5, cv::Scalar(255,100,0), 1);

    // draw different colored arrow from our position at extrapolated angle
    cv::Point2f arrowEndEx = currPosEx + cv::Point2f(50.0 * cos(currAngleEx), 50.0 * sin(currAngleEx));
    cv::arrowedLine(drawingImage, currPosEx, arrowEndEx, cv::Scalar(255, 100, 0), 2);

    END_SAFE_DRAW



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

    // thrash angle
    // response.turn_amount = deltaAngleRad > 0 ? 1.0 : -1.0;
    // if (abs(deltaAngleRad) < 10 * TO_RAD)
    // {
    //     response.turn_amount = 0;
    // }

    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(scaleDownMovement, 1.0 - abs(response.turn) * scaleDownMovement) * 1.0;

    response.movement = goToOtherTarget ? -drive_scale : drive_scale;

    // Draw debugging information
    SAFE_DRAW
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);
    END_SAFE_DRAW

    response.movement *= -1;
    response.turn *= -1;

    return response;
}

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


    SAFE_DRAW
    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, ORBIT_RADIUS, cv::Scalar(255, 0, 0), 1);
    // draw arrow from opponent position at opponent angle
    cv::Point2f arrowEnd = opponentPos + cv::Point2f(100.0 * cos(RobotOdometry::Opponent().GetAngle()), 100.0 * sin(RobotOdometry::Opponent().GetAngle()));
    cv::arrowedLine(drawingImage, opponentPos, arrowEnd, cv::Scalar(255, 0, 0), 2);
    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, ORBIT_RADIUS, cv::Scalar(255, 165, 0), 4);
    END_SAFE_DRAW




    double velocityNorm = cv::norm(RobotOdometry::Robot().GetVelocity());

    double radius = PURE_PURSUIT_RADIUS;
    if (velocityNorm > 250)
    {
        radius *= 1.0 + (velocityNorm - 250.0) / 500.0;
    }

    // default to the angle from the opponent to us
    cv::Point2f targetPoint = opponentPosEx + cv::Point2f(ORBIT_RADIUS * cos(angleOpponentToUs), ORBIT_RADIUS * sin(angleOpponentToUs));
    std::vector<cv::Point2f> circleIntersections = CirclesIntersect(ourPosition, radius, opponentPosEx, ORBIT_RADIUS);

    SAFE_DRAW
    // draw circle at our position with radius PURE_PURSUIT_RADIUS_PX
    cv::circle(drawingImage, ourPosition, radius, cv::Scalar(0, 255, 0), 1);
    END_SAFE_DRAW

    if (circleIntersections.size() > 0)
    {
        targetPoint = circleIntersections[0];
    }


    SAFE_DRAW
    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);
    END_SAFE_DRAW

    bool allowReverse = false;

    // Drive to the opponent position
    DriveCommand response = DriveToPosition(targetPoint, allowReverse);

    response.movement *= MASTER_SPEED_SCALE_PERCENT / 100.0;
    response.turn *= MASTER_SPEED_SCALE_PERCENT / 100.0;




    // // compute the difference in our radios vs target radios
    // double deltaRadius = norm(exState.position - opponentPosEx) - ORBIT_RADIUS;

    // // get closest tangent direction to circle
    // double tangentDirection = angleOpponentToUs + M_PI / 2.0;

    // double targetDirection = tangentDirection + std::clamp(deltaRadius / PURE_PURSUIT_RADIUS, -M_PI / 2.0, M_PI / 2.0);

    // // draw arrow at targetDirection
    // SAFE_DRAW
    // cv::Point2f arrowEnd = exState.position + cv::Point2f(100.0 * cos(targetDirection), 100.0 * sin(targetDirection));
    // cv::arrowedLine(drawingImage, exState.position, arrowEnd, cv::Scalar(0, 255, 0), 2);
    // END_SAFE_DRAW

    // // compute the angle from the tangent to the robot
    // double deltaAngleRad = angle_wrap(exState.angle - targetDirection);

    // DriveCommand response;
    // response.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
    //                                             TURN_THRESH_2_DEG * TO_RAD, MIN_TURN_POWER_PERCENT / 100.0, MAX_TURN_POWER_PERCENT / 100.0);
    

    // double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;
    // // Slow down when far away from the target angle
    // double drive_scale = std::max(scaleDownMovement, 1.0 - abs(response.turn) * scaleDownMovement) * 1.0;

    // response.movement = drive_scale;


    return response;
}

/**
 * ManualMode
 * Allows the user to drive the robot manually
 */
DriveCommand RobotController::ManualMode()
{
    DriveCommand response{0, 0};
    response.movement = gamepad.GetRightStickY();
    response.turn = -gamepad.GetLeftStickX();


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

    if (gamepad.GetLeftBumper())
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