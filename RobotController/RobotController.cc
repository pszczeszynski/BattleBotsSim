#include "RobotController.h"
#include "MathUtils.h"
#include <QApplication>
#include <QMainWindow>
#include "RobotControllerGUI.h"
#include "RobotConfig.h"
#include "RobotLink.h"
#include <QThread>

// #define ENABLE_TIMERS
#ifdef ENABLE_TIMERS
    #define TIMER_INIT Clock c;
    #define TIMER_START c.markStart();
    #define TIMER_PRINT(msg) std::cout << msg << " time: " << c.getElapsedTime() << std::endl;
#else
    #define TIMER_INIT 
    #define TIMER_START 
    #define TIMER_PRINT(msg)
#endif

#ifdef ENABLE_VISION
#include "Vision.h"
#include <opencv2/core.hpp>
#endif

#include "Extrapolator.h"

int argc = 0;

QApplication app(argc, nullptr);
RobotConfigWindow robotControllerGUI;

int main(int argc, char *argv[])
{
    robotControllerGUI.SetApp(app);

    RobotController rc;
    // Create a separate thread for RobotController
    QThread controllerThread;
    rc.moveToThread(&controllerThread);

    QObject::connect(&controllerThread, &QThread::started, &rc, &RobotController::Run);
    robotControllerGUI.ShowGUI();

    controllerThread.start();

    app.exec();
}

RobotController::RobotController() :
    robotTracker{cv::Point2f(0, 0)},
    opponentTracker{cv::Point2f(10000, 10000)},
#ifdef ENABLE_VISION
#ifdef SIMULATION
      overheadCamL_sim{"overheadCamL"},
    //   ,overheadCamR_sim{"overheadCamR"}
      vision{overheadCamL_sim, robotTracker, opponentTracker}
#else
      overheadCamL_real{1},
    //   ,overheadCamR_real{1}
      vision{overheadCamL_real, robotTracker, opponentTracker} // overheadCamR_real
#endif
#endif
{

}

double stickAngle = 0;
#define ACCELEROMETER_TO_PX_SCALER 37.5

void RobotController::Run()
{
    TIMER_INIT
    std::cout << "running" << std::endl;
    Clock lastTime;
    lastTime.markStart();

    unsigned long frames = 0;
    // receive until the peer closes the connection
    while (true)
    {
        // 1. receive state info from the robot
        RobotMessage message = robotLink.Receive();
        frames ++;

        if (frames % 100 == 0)
        {
            std::cout << "fps: " << frames / lastTime.getElapsedTime() << std::endl;
            frames = 0;
            lastTime.markStart();
        }

        vision.angle = message.rotation * TO_RAD;

        TIMER_START
        bool updated    = vision.runPipeline();

        robotIMUData.velocity = cv::Point2f(message.velocity.x, -message.velocity.z) * ACCELEROMETER_TO_PX_SCALER;
        robotIMUData.angle = message.rotation * TO_RAD;

        if (!updated)
        {
            TIMER_PRINT("vision.runPipeline() no update")
            robotTracker.UpdateIMUOnly(robotIMUData);
        }
        else
        {
            TIMER_PRINT("vision.runPipeline() update")
        }

        // 3. run our robot controller loop
        TIMER_START
        DriveCommand response = loop(message);
        TIMER_PRINT("loop()")

        if (!IS_RUNNING)
        {
            response.movement = 0;
            response.turn = 0;
        }

        TIMER_START
        // send the response to the robot
        robotLink.Drive(response);
        TIMER_PRINT("Send to robot")

        TIMER_START
        if (updated)
        {
            robotControllerGUI.RefreshFieldImage();
        }
        TIMER_PRINT("imshow()")
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
double doubleThreshToTarget(double error,
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

bool danger = false;

/**
 * Drive the robot to a specified position
 * @param targetPos The target position to drive the robot to
 * @param state The current state of the robot and opponent
 * @param drawingImage The image for drawing debugging information
 * @return The response to send back to Unity
 */
DriveCommand RobotController::driveToPosition(const cv::Point2f currPos, const double currAngle, const cv::Point2f &targetPos, bool chooseNewTarget = false)
{
    static Extrapolator<Angle> currAngleExtrapolator{Angle(0.0)};
    static Extrapolator<cv::Point2f> currPositionExtrapolator{cv::Point2f(0, 0)};
    static Clock c;
    static bool goToOtherTarget = true;

    double deltaTime = c.getElapsedTime();
    c.markStart();

    currAngleExtrapolator.SetValue(Angle(currAngle));
    double currAngleEx = currAngleExtrapolator.Extrapolate(ANGLE_EXTRAPOLATE_MS / 1000.0);
    currPositionExtrapolator.SetValue(currPos);
    cv::Point2f currPosEx = currPositionExtrapolator.Extrapolate(POSITION_EXTRAPOLATE_MS / 1000.0);

    // override extrapolated pos using the imu velocity
    currPosEx = currPos + robotIMUData.velocity * POSITION_EXTRAPOLATE_MS / 1000.0;

    // draw arrow from our position at our angle
    cv::Point2f arrowEnd = currPos + cv::Point2f(100.0 * cos(currAngle), 100.0 * sin(currAngle));
    cv::arrowedLine(drawingImage, currPos, arrowEnd, cv::Scalar(0, 0, 255), 2);
    // draw different colored arrow from our position at extrapolated angle
    cv::Point2f arrowEndEx = currPosEx + cv::Point2f(100.0 * cos(currAngleEx), 100.0 * sin(currAngleEx));
    cv::arrowedLine(drawingImage, currPosEx, arrowEndEx, cv::Scalar(255, 100, 0), 1);

    // draw our position
    cv::circle(drawingImage, currPos, 5, cv::Scalar(0,0,255), 2);
    // draw circle with dotted line at extrapolated position
    cv::circle(drawingImage, currPosEx, 5, cv::Scalar(255,100,0), 1);

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
    response.turn = doubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                                TURN_THRESH_2_DEG * TO_RAD, MIN_TURN_POWER_PERCENT / 100.0, MAX_TURN_POWER_PERCENT / 100.0);

    // thrash angle
    // response.turn_amount = deltaAngleRad > 0 ? 1.0 : -1.0;
    // if (abs(deltaAngleRad) < 10 * TO_RAD)
    // {
    //     response.turn_amount = 0;
    // }

    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;//danger ? 0.8 : 0.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(scaleDownMovement, 1.0 - abs(response.turn) * scaleDownMovement) * 1.0;

    response.movement = goToOtherTarget ? -drive_scale : drive_scale;

    // Draw debugging information
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);

    response.movement *= -1;
    response.turn *= -1;

    return response;
}

Clock runAwayClock;
bool timing = false;

/**
 * This is the main robot controller loop. It is called once per frame.
 * @param message The current state of the robot and opponent
 * @return The response to send back to unity
 */
DriveCommand RobotController::loop(RobotMessage &message)
{
    static Extrapolator<cv::Point2f> ourPositionExtrapolator{cv::Point2f(0,0)};
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};
    static bool shouldRunClockwiseLast = false;

    // our pos + angle
    cv::Point2f ourPosition = vision.GetRobotPosition();
    ourPositionExtrapolator.SetValue(ourPosition);
    cv::Point2f ourPositionEx = ourPositionExtrapolator.Extrapolate(POSITION_EXTRAPOLATE_MS / 1000.0);

    // opponent pos + angle
    cv::Point2f opponentPos = vision.GetOpponentPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    opponentPosEx = opponentTracker.GetVelocity() * OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 + opponentPos;
    double stickAngleRad = stickAngle;//vision.GetOpponentAngle();

    // default just to drive to the center
    cv::Point2f targetPoint = opponentPosEx;

    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);


    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, ORBIT_RADIUS, cv::Scalar(255, 0, 0), 1);

    // draw arrow from opponent position at opponent angle
    cv::Point2f arrowEnd = opponentPos + cv::Point2f(100.0 * cos(vision.GetOpponentAngle()), 100.0 * sin(vision.GetOpponentAngle()));
    cv::arrowedLine(drawingImage, opponentPos, arrowEnd, cv::Scalar(255, 0, 0), 2);

    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, ORBIT_RADIUS, cv::Scalar(255, 165, 0), 4);

    bool orbitRight = abs(angle_wrap(stickAngleRad - 0)) < (30 * TO_RAD) && stickAngleRad != 0;
    bool orbitLeft = abs(angle_wrap(stickAngleRad - M_PI)) < (30 * TO_RAD);

    if (orbitRight || orbitLeft)
    {
        const double EVASION_DELTA_THETA = ORBIT_DTHETA_DEG * TO_RAD;
        // set target point to be on a circle around the opponent
        double angleToOpponent = atan2(opponentPosEx.y - ourPosition.y, opponentPosEx.x - ourPosition.x);
        double evasionAngle = angle_wrap(angleToOpponent + (orbitRight ? 1 : -1) * EVASION_DELTA_THETA);
        targetPoint = opponentPosEx - cv::Point2f(cos(evasionAngle), sin(evasionAngle)) * ORBIT_RADIUS;
    }

    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    bool allowReverse = false;

    // Drive to the opponent position
    DriveCommand response = driveToPosition(ourPosition, vision.GetRobotAngle(), targetPoint, allowReverse);

    response.movement *= MASTER_SPEED_SCALE_PERCENT / 100.0;
    response.turn *= MASTER_SPEED_SCALE_PERCENT / 100.0;

    return response;
}
