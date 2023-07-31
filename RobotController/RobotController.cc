#include "Extrapolator.h"
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
                                     overheadCamL_real{0},
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
    DriveCommand c{0, 0};
    robotLink.Drive(c);

    // receive until the peer closes the connection
    while (true)
    {
        // log the frame rate
        LogFPS();

        // update the gamepad
        gamepad.Update();

        // receive state info from the robot
        state = robotLink.Receive();

        TIMER_START
        // VisionClassification classification = vision.RunPipeline();
        // UpdateRobotTrackers(classification);
        TIMER_PRINT("Vision")

        // 3. run our robot controller loop
        TIMER_START
        DriveCommand response = RobotLogic();
        TIMER_PRINT("RobotLogic")

        TIMER_START
        // send the response to the robot
        robotLink.Drive(response);
        TIMER_PRINT("Drive")

        TIMER_START
        // refresh the field image
        // emit RefreshFieldImageSignal();
        TIMER_PRINT("imshow()")
    }
}

void RobotController::UpdateRobotTrackers(VisionClassification classification)
{
    // if vision detected our robot
    if (classification.GetRobotBlob() != nullptr)
    {
        MotionBlob robot = *classification.GetRobotBlob();
        cv::Mat& frame = *(classification.GetRobotBlob()->frame);
        RobotTracker::Robot().UpdateVisionAndIMU(robot, frame);
    }
    else
    {
        // otherwise just update using the imu
        RobotTracker::Robot().UpdateIMUOnly();
    }

    // if vision detected the opponent
    if (classification.GetOpponentBlob() != nullptr)
    {
        MotionBlob opponent = *classification.GetOpponentBlob();
        cv::Mat& frame = *(classification.GetOpponentBlob()->frame);
        RobotTracker::Opponent().UpdateVisionOnly(opponent, frame);
    }
    else
    {
        // set the opponent to invalid (sets their velocity to 0)
        RobotTracker::Opponent().invalidate();
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

    cv::Point2f currPos = RobotTracker::Robot().getPosition();
    double currAngle = RobotTracker::Robot().getAngle();

    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    double deltaTime = c.getElapsedTime();
    c.markStart();

    RobotSimState currentState;
    currentState.position = currPos;
    currentState.angle = currAngle;
    currentState.velocity = RobotTracker::Robot().GetVelocity();
    currentState.angularVelocity = RobotTracker::Robot().GetAngleVelocity();

    RobotSimState exState = robotSimulator.Simulate(currentState, POSITION_EXTRAPOLATE_MS / 1000.0, 50);
    double currAngleEx = exState.angle;
    cv::Point2f currPosEx = exState.position;

    // draw arrow from our position at our angle
    cv::Point2f arrowEnd = currPos + cv::Point2f(50.0 * cos(currAngle), 50.0 * sin(currAngle));
    cv::arrowedLine(DRAWING_IMAGE, currPos, arrowEnd, cv::Scalar(0, 0, 255), 1);
    // // draw different colored arrow from our position at extrapolated angle
    // cv::Point2f arrowEndEx = currPosEx + cv::Point2f(50.0 * cos(currAngleEx), 50.0 * sin(currAngleEx));
    // cv::arrowedLine(DRAWING_IMAGE, currPosEx, arrowEndEx, cv::Scalar(255, 100, 0), 2);

    // draw our position
    cv::circle(DRAWING_IMAGE, currPos, 5, cv::Scalar(0,0,255), 2);
    // draw circle with dotted line at extrapolated position
    cv::circle(DRAWING_IMAGE, currPosEx, 5, cv::Scalar(255,100,0), 1);

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
    cv::circle(DRAWING_IMAGE, targetPos, 10, cv::Scalar(0, 255, 0), 4);

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
    cv::Point2f ourPosition = RobotTracker::Robot().getPosition();

    // opponent pos + angle
    cv::Point2f opponentPos = RobotTracker::Opponent().getPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    opponentPosEx = RobotTracker::Opponent().GetVelocity() * OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 + opponentPos;

    // default just to drive to the center
    cv::Point2f targetPoint = opponentPosEx;

    // get the angle from us to the opponent
    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    // add pi to get the angle from the opponent to us
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);


    // draw blue circle around opponent
    cv::circle(DRAWING_IMAGE, opponentPos, ORBIT_RADIUS, cv::Scalar(255, 0, 0), 1);

    // draw arrow from opponent position at opponent angle
    cv::Point2f arrowEnd = opponentPos + cv::Point2f(100.0 * cos(RobotTracker::Opponent().angle), 100.0 * sin(RobotTracker::Opponent().angle));
    cv::arrowedLine(DRAWING_IMAGE, opponentPos, arrowEnd, cv::Scalar(255, 0, 0), 2);

    // draw orange circle around opponent to show evasion radius
    cv::circle(DRAWING_IMAGE, opponentPosEx, ORBIT_RADIUS, cv::Scalar(255, 165, 0), 4);


    bool orbitRight = gamepad.GetRightStickY() > 0;
    bool orbitLeft = gamepad.GetRightStickY() < 0;

    if (orbitRight || orbitLeft)
    {
        const double EVASION_DELTA_THETA = ORBIT_DTHETA_DEG * TO_RAD;
        // set target point to be on a circle around the opponent
        double angleToOpponent = atan2(opponentPosEx.y - ourPosition.y, opponentPosEx.x - ourPosition.x);
        double evasionAngle = angle_wrap(angleToOpponent + (orbitRight ? 1 : -1) * EVASION_DELTA_THETA);
        targetPoint = opponentPosEx - cv::Point2f(cos(evasionAngle), sin(evasionAngle)) * ORBIT_RADIUS;
    }

    // Draw the point
    cv::circle(DRAWING_IMAGE, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    bool allowReverse = false;

    // Drive to the opponent position
    DriveCommand response = DriveToPosition(targetPoint, allowReverse);

    response.movement *= MASTER_SPEED_SCALE_PERCENT / 100.0;
    response.turn *= MASTER_SPEED_SCALE_PERCENT / 100.0;

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
    
    DRAWING_IMAGE_MUTEX.lock();
    DriveCommand responseManual = ManualMode();
    DriveCommand responseOrbit = OrbitMode();
    DRAWING_IMAGE_MUTEX.unlock();

    DriveCommand ret = responseManual;

    if (gamepad.GetLeftBumper())
    {
        ret.turn = responseOrbit.turn;
        ret.movement = responseManual.movement * responseOrbit.movement;
    }

    return ret;
}