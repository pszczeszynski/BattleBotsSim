#include "RobotController.h"
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include "MathUtils.h"
#include "StateMachine.h"

#ifdef ENABLE_VISION
#include "Vision.h"
#include <opencv2/core.hpp>
#endif

#include "Extrapolator.h"

int main()
{
    RobotController rc{};
    rc.Run();

    return 0;
}

RobotController::RobotController()
    : socket{"11115"}
#ifdef ENABLE_VISION
#ifdef SIMULATION
      ,overheadCamL_sim{"overheadCamL"}
    //   ,overheadCamR_sim{"overheadCamR"}
      ,vision{overheadCamL_sim}
#else
      ,overheadCamL_real{0}
      ,overheadCamR_real{1}
      ,vision{overheadCamL_real, overheadCamR_real}
#endif
#endif
{

}

bool pause = true;

double extrapolateAmount = 0;

enum class State
{
    DRIVE_DIRECTLY_NO_DANGER,
    PURE_PURSUIT,
    GET_AWAY,
};

StateMachine<State> stateMachine;

void RobotController::Run()
{
    std::cout << "running" << std::endl;
    Clock lastTime;
    lastTime.markStart();

    Clock c;
    unsigned long frames = 0;
    // receive until the peer closes the connection
    while (true)
    {

#ifdef SIMULATION
        // 1. receive state info from unity
        std::string received = socket.receive();
        if (received == "")
        {
            std::cout << "continuing" << std::endl;
            continue;
        }
        frames ++;
        if (frames % 100 == 0)
        {
            std::cout << "fps: " << frames / lastTime.getElapsedTime() << std::endl;
            frames = 0;
            lastTime.markStart();
        }

        // 2. parse state info
        RobotState state = RobotStateParser::parse(received);
#endif

#ifdef ENABLE_VISION
#ifdef SIMULATION
        vision.angle = state.robot_orientation * TO_RAD;
        vision.opponent_angle = state.opponent_orientation * TO_RAD;
        vision.position = cv::Point2f(state.robot_position.x + 20, -state.robot_position.z + 13) * 30;
        vision.opponent_position = cv::Point2f(state.opponent_position.x + 20, -state.opponent_position.z + 13) * 30;
#endif

        c.markStart();
        vision.runPipeline();
        std::cout << "vision time: " << c.getElapsedTime() << std::endl;

        char key = cv::waitKey(1);
#endif

        c.markStart();
        // get birds eye view image
        cv::Mat& drawingImage = (cv::Mat&) vision.GetBirdsEyeImage().clone();
        std::cout << "clone time: " << c.getElapsedTime() << std::endl;
        // cv::Mat drawingImage = frame.clone();

#ifdef SIMULATION

        c.markStart();
        // 3. run our robot controller loop
        RobotControllerMessage response = loop(state, drawingImage);
        std::cout << "loop time: " << c.getElapsedTime() << std::endl;

        // check if space pressed
        if (key == ' ')
        {
            pause = !pause;
        }

        // if up arrow
        if (key == 'i')
        {
            extrapolateAmount += 0.1;
            std::cout << "extrapolate: " << extrapolateAmount << std::endl;
        }

        if (key == 'k')
        {
            extrapolateAmount -= 0.1;
            std::cout << "extrapolate: " << extrapolateAmount << std::endl;
        }

        if (pause)
        {
            response.drive_amount = 0;
            response.turn_amount = 0;
        }
        // send the response back to unity (tell it how much to drive and turn)
        socket.reply_to_last_sender(RobotStateParser::serialize(response));

#endif
        // // draw the opponent profile graphic
        // // init square mat of 400 x 400
        // cv::Mat drawing_image = cv::Mat::zeros(400, 400, CV_8UC3);
        // p.DrawGraphic(drawing_image);

        c.markStart();
        // DrawGraphic(drawingImage);
        cv::imshow("drawing", drawingImage);
        std::cout << "draw time: " << c.getElapsedTime() << std::endl;
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
RobotControllerMessage RobotController::driveToPosition(const cv::Point2f currPos, const double currAngle, const cv::Point2f &targetPos, const RobotState &state, cv::Mat &drawingImage, bool chooseNewTarget = false)
{
    static Extrapolator<double> currAngleExtrapolator{0};

    static bool goToOtherTarget = false;

    currAngleExtrapolator.SetValue(currAngle);
    double currAngleEx = currAngleExtrapolator.Extrapolate(0.1);

    // draw our position and extrapolated position
    cv::circle(drawingImage, currPos, 5, cv::Scalar(0,0,255), 2);

    double angleToTarget1 = atan2(targetPos.y - currPos.y, targetPos.x - currPos.x);
    double angleToTarget2 = angle_wrap(angleToTarget1 + M_PI);
    double deltaAngleRad1 = angle_wrap(angleToTarget1 - currAngleEx);
    double deltaAngleRad2 = angle_wrap(angleToTarget2 - currAngleEx);
    double deltaAngleRad1_noex = angle_wrap(angleToTarget1 - currAngle);
    double deltaAngleRad2_noex = angle_wrap(angleToTarget2 - currAngle);
    if (chooseNewTarget)
    {
        goToOtherTarget = abs(deltaAngleRad1) < abs(deltaAngleRad2);
    }

    double deltaAngleRad = goToOtherTarget ? deltaAngleRad1 : deltaAngleRad2;
    double deltaAngleRad_noex = goToOtherTarget ? deltaAngleRad1_noex : deltaAngleRad2_noex;

    const double TURN_THRESH1_ANG = 50 * TO_RAD;
    const double TURN_THRESH2_ANG = 10 * TO_RAD;
    const double MAX_TURN_POWER = 1.0;
    const double MIN_TURN_POWER = 0.3;

    RobotControllerMessage response{0, 0};
    response.turn_amount = doubleThreshToTarget(deltaAngleRad, TURN_THRESH1_ANG,
        TURN_THRESH2_ANG, MIN_TURN_POWER, MAX_TURN_POWER);

    double scaleDownMovement = 0;//danger ? 0.8 : 0.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(scaleDownMovement, 1.0 - abs(response.turn_amount) * scaleDownMovement) * 1.0;

    response.drive_amount = goToOtherTarget ? -drive_scale : drive_scale;

    // Draw debugging information
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);

    response.drive_amount *= -1;
    response.turn_amount *= -1;

    return response;
}

Clock runAwayClock;
bool timing = false;

/**
 * This is the main robot controller loop. It is called once per frame.
 * @param state The current state of the robot and opponent
 * @return The response to send back to unity
 */
RobotControllerMessage RobotController::loop(RobotState &state, cv::Mat &drawingImage)
{
    double SAFE_RADIUS = 100;
    double RADIUS_OF_NO_RETURN = SAFE_RADIUS / 2;
    // Check if opponent is facing us
    const double ANGLE_TOLERANCE = 85 * TO_RAD;

    static Extrapolator<cv::Point2f> ourPositionExtrapolator{cv::Point2f(0,0)};
    static Extrapolator<double> ourAngleExtrapolator{0};
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};
    static Extrapolator<double> opponentAngleExtrapolator{0};
    static bool shouldRunClockwiseLast = false;

    // our pos + angle
    cv::Point2f ourPosition = vision.GetRobotPosition();
    ourPositionExtrapolator.SetValue(ourPosition);
    cv::Point2f ourPositionEx = ourPositionExtrapolator.Extrapolate(0.1);
    ourAngleExtrapolator.SetValue(vision.GetRobotAngle());
    double ourAngleEx = ourAngleExtrapolator.Extrapolate(0.3);
    double velocity = norm(ourPositionExtrapolator.GetVelocity());

    // opponent pos + angle
    cv::Point2f opponentPos = vision.GetOpponentPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(0.0 * norm(opponentPos - ourPosition) / SAFE_RADIUS);
    double opponentAngleRad = vision.GetOpponentAngle();
    opponentAngleExtrapolator.SetValue(opponentAngleRad);
    double opponentAngleEx = angle_wrap(opponentAngleExtrapolator.Extrapolate(0.0 * norm(opponentPos - ourPosition) / SAFE_RADIUS));
    
    // default just to drive to the center
    cv::Point2f targetPoint = opponentPosEx;

    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);

    
    // // make sure opponentAngleEx doesn't cross angleFromFacingUs
    // if (opponentAngleExtrapolator.GetVelocity() > 0 &&
    //     angle_wrap(opponentAngleRad - angleOpponentToUs) < 0 &&
    //     angle_wrap(opponentAngleEx - angleOpponentToUs) > 0)
    // {
    //     opponentAngleEx = angleOpponentToUs;
    //     std::cout << "limiting1" << std::endl;
    // }
    // else if (opponentAngleExtrapolator.GetVelocity() < 0 &&
    //     angle_wrap(opponentAngleRad - angleOpponentToUs) > 0 &&
    //     angle_wrap(opponentAngleEx - angleOpponentToUs) < 0)
    // {
    //     opponentAngleEx = angleOpponentToUs;
    //     std::cout << "limiting2" << std::endl;
    // }
    opponentAngleRad = opponentAngleEx;

    double angleFromFacingUs = angle_wrap(angleToOpponent - opponentAngleEx - M_PI);

    double angle1 = angle_wrap(opponentAngleEx + ANGLE_TOLERANCE);
    double angle2 = angle_wrap(opponentAngleEx - ANGLE_TOLERANCE);

    // calculate the follow point if we go right
    cv::Point2f point1 = opponentPosEx + cv::Point2f(cos(angle1), sin(angle1)) * SAFE_RADIUS;
    // calculate the follow point if we go left
    cv::Point2f point2 = opponentPosEx + cv::Point2f(cos(angle2), sin(angle2)) * SAFE_RADIUS;
    // draw line at each angle
    cv::line(drawingImage, opponentPosEx, point1, cv::Scalar(0,255,0), 4);
    cv::line(drawingImage, opponentPosEx, point2, cv::Scalar(0,255,0), 4);
    
    // draw red circle at RADIUS_OF_NO_RETURN
    cv::circle(drawingImage, opponentPosEx, RADIUS_OF_NO_RETURN, cv::Scalar(0, 0, 255), 4);

    if (norm(usToOpponent) < RADIUS_OF_NO_RETURN)
    {
        // fill the circle red
        cv::circle(drawingImage, opponentPosEx, RADIUS_OF_NO_RETURN, cv::Scalar(0, 0, 255), -1);
    }

    // if opponent's weapon is facing us
    if (abs(angleFromFacingUs) < ANGLE_TOLERANCE && norm(usToOpponent) > RADIUS_OF_NO_RETURN)
    {
        // set danger flag
        danger = true;
        // Set the follow point's angle delta (added to the angle to the opponent)
        double MARGIN_ANGLE = 8 * TO_RAD;
        // default to going the same direction as last time
        bool goClockwise = shouldRunClockwiseLast;
        // if we're not moving very fast, go towards the closer point
        if (velocity < 20)
        {
            goClockwise = cv::norm(point1 - ourPosition) < cv::norm(point2 - ourPosition);
        }
        // save the direction we're going for next time
        shouldRunClockwiseLast = goClockwise;
        // select the point to go to
        cv::Point2f wayPoint = goClockwise ? point1 : point2;

        targetPoint = wayPoint;

        // draw orange circle around opponent to show evasion radius
        cv::circle(drawingImage, opponentPosEx, SAFE_RADIUS, cv::Scalar(0, 165, 255), 4);

        // if we are within the safe radius, we need to evade
        if (norm(ourPosition - opponentPosEx) < SAFE_RADIUS)
        {
            const double EVASION_DELTA_THETA = 20 * TO_RAD;
            // set target point to be on a circle around the opponent
            double angleToOpponent = atan2(opponentPosEx.y - ourPosition.y, opponentPosEx.x - ourPosition.x);
            double evasionAngle = angle_wrap(angleToOpponent + (goClockwise ? 1 : -1) * EVASION_DELTA_THETA);
            targetPoint = opponentPosEx - cv::Point2f(cos(evasionAngle), sin(evasionAngle)) * SAFE_RADIUS;
        }
        // else go tangent to the evasion circle
        else
        {
            // calculate tangent points
            cv::Point2f tangentPoint1;
            cv::Point2f tangentPoint2;
            calculateTangentPoints(opponentPosEx, SAFE_RADIUS, ourPosition, tangentPoint1, tangentPoint2);

            // select the tangent point to go to using goClockwise
            targetPoint = goClockwise ? tangentPoint1 : tangentPoint2;
            // draw line
            cv::line(drawingImage, ourPosition, targetPoint, cv::Scalar(0,255,255), 2);
        }

        // draw dot at target point
        cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 165, 255), 4);
    }

    const double ALLOW_REVERSE_MAX_VELOCITY = 20;
    const double MIN_VELOCITY_TIME_UNTIL_RUN_AWAY = 1.5;
    const double RUN_AWAY_DIST = 100;

    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    bool allowReverse = velocity < ALLOW_REVERSE_MAX_VELOCITY;

    // targetPoint = cv::Point2f(622, 410);
    // Drive to the opponent position
    RobotControllerMessage response = driveToPosition(ourPosition, vision.GetRobotAngle(), targetPoint, state, drawingImage, allowReverse);

    return response;
}
