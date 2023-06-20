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
      ,overheadCam{"overheadCam"},
      vision{overheadCam}
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
    Clock lastTime;
    lastTime.markStart();
    unsigned long frames = 0;
    // receive until the peer closes the connection
    while (true)
    {
        // 1. receive state info from unity
        std::string received = socket.receive();
        if (received == "")
        {
            continue;
        }
        frames ++;
        if (frames % 100 == 0)
        {
            std::cout << "fps: " << frames / lastTime.getElapsedTime() << std::endl;
        }

        // 2. parse state info
        RobotState state = RobotStateParser::parse(received);

#ifdef ENABLE_VISION
        vision.angle = state.robot_orientation * TO_RAD;
        vision.opponent_angle = state.opponent_orientation * TO_RAD;
        vision.position = cv::Point2f(state.robot_position.x + 20, -state.robot_position.z + 13) * 30;
        vision.opponent_position = cv::Point2f(state.opponent_position.x + 20, -state.opponent_position.z + 13) * 30;

        vision.runPipeline();
        char key = cv::waitKey(1);
#endif

        // get birds eye view image
        cv::Mat& drawingImage = (cv::Mat&) vision.GetBirdsEyeImage();
        // cv::Mat drawingImage = frame.clone();

        // 3. run our robot controller loop
        RobotControllerMessage response = loop(state, drawingImage);

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


        // // draw the opponent profile graphic
        // // init square mat of 400 x 400
        // cv::Mat drawing_image = cv::Mat::zeros(400, 400, CV_8UC3);
        // p.DrawGraphic(drawing_image);

        // DrawGraphic(drawingImage);
        cv::imshow("drawing", drawingImage);

    }
}

void RobotController::DrawGraphic(const cv::Mat& frame)
{
    // clone the frame so we can draw on it
    cv::Mat frameWithCircles = frame.clone();

    const cv::Point2f robotPosition = vision.GetRobotPosition();
    const cv::Point2f opponentPosition = vision.GetOpponentPosition();
    const double robotAngle = vision.GetRobotAngle();
    const double opponentAngle = vision.GetOpponentAngle();

    // draw lines starting at the center of each robot showing their angles
    const double RADIUS = 30;

    cv::circle(frameWithCircles, robotPosition, RADIUS, cv::Scalar(255,0,0), 4);
    cv::circle(frameWithCircles, opponentPosition, RADIUS, cv::Scalar(0,0,255), 4);

    cv::line(frameWithCircles, robotPosition, robotPosition + cv::Point2f(cos(robotAngle) * RADIUS, sin(robotAngle) * RADIUS), cv::Scalar(255,0,0), 4);
    cv::line(frameWithCircles, opponentPosition, opponentPosition + cv::Point2f(cos(opponentAngle) * RADIUS, sin(opponentAngle) * RADIUS), cv::Scalar(0,0,255), 4);

    // draw the vector lines
    PathFinder pathFinder;

    // draw the safety at each position
    int iterate_size = 10;
    int circle_radius = 3;
    int lineLength = 10;
    for (int y = 0; y < frameWithCircles.rows; y += iterate_size)
    {
        for (int x = 0; x < frameWithCircles.cols; x += iterate_size)
        {
            cv::Point2f attackVector = pathFinder.GetMotionVector(opponentPosition, opponentAngle, p, cv::Point2f(x,y));

            // draw a line
            cv::line(frameWithCircles, cv::Point2f(x,y), cv::Point2f(x,y) + attackVector * lineLength, cv::Scalar(0,255,0), 1);
        }
    }

    cv::imshow("keypoints", frameWithCircles);
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
    double currAngleEx = currAngleExtrapolator.Extrapolate(0.15);

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

    const double MAX_POWER_ANGLE = 80 * TO_RAD;
    RobotControllerMessage response{0, 0};
    response.turn_amount = std::clamp(deltaAngleRad / MAX_POWER_ANGLE, -1.0, 1.0) * 1.0;

    double scaleDownMovement = danger ? 0.8 : 0.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(scaleDownMovement, 1.0 - abs(deltaAngleRad_noex) / MAX_POWER_ANGLE) * 1.0;

    response.drive_amount = goToOtherTarget ? -drive_scale : drive_scale;

    // Draw debugging information
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);

    return response;
}

Clock runAwayClock;
bool timing = false;
bool isRunningAway = false;

/**
 * This is the main robot controller loop. It is called once per frame.
 * @param state The current state of the robot and opponent
 * @return The response to send back to unity
 */
RobotControllerMessage RobotController::loop(RobotState &state, cv::Mat &drawingImage)
{
    double SAFE_RADIUS = 100;
    // Check if opponent is facing us
    const double ANGLE_TOLERANCE = 70 * TO_RAD;

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
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(0.25 * norm(opponentPos - ourPosition) / SAFE_RADIUS);
    double opponentAngleRad = vision.GetOpponentAngle();
    opponentAngleExtrapolator.SetValue(opponentAngleRad);
    double opponentAngleEx = angle_wrap(opponentAngleExtrapolator.Extrapolate(0.1 * norm(opponentPos - ourPosition) / SAFE_RADIUS));
    
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
    // draw line at each angle
    cv::line(drawingImage, opponentPosEx, opponentPosEx + cv::Point2f(cos(angle1), sin(angle1)) * SAFE_RADIUS, cv::Scalar(0,255,0), 4);
    cv::line(drawingImage, opponentPosEx, opponentPosEx + cv::Point2f(cos(angle2), sin(angle2)) * SAFE_RADIUS, cv::Scalar(0,255,0), 4);
    
    isRunningAway = false;
    if (abs(angleFromFacingUs) < ANGLE_TOLERANCE)
    {
        danger = true;
        const double MARGIN_ANGLE = 8 * TO_RAD;
        cv::Point2f point1 = opponentPosEx + cv::Point2f(cos(angle1 + MARGIN_ANGLE), sin(angle1 + MARGIN_ANGLE)) * SAFE_RADIUS;
        cv::Point2f point2 = opponentPosEx + cv::Point2f(cos(angle2 - MARGIN_ANGLE), sin(angle2 - MARGIN_ANGLE )) * SAFE_RADIUS;

        bool goClockwise = shouldRunClockwiseLast;
        if (velocity < 20)
        {
            goClockwise = cv::norm(point1 - ourPosition) < cv::norm(point2 - ourPosition);
        }
        shouldRunClockwiseLast = goClockwise;

        cv::Point2f wayPoint = goClockwise ? point1 : point2;


        targetPoint = wayPoint;


        const double EVASION_RADIUS = SAFE_RADIUS * 0.85;
        const double EVASION_DELTA_THETA = 20 * TO_RAD;

        cv::Point2f ourPositionExMore = ourPositionExtrapolator.Extrapolate(0);
        // set target point to be on a circle around the opponent
        double angleToOpponent = atan2(opponentPosEx.y - ourPositionExMore.y, opponentPosEx.x - ourPositionExMore.x);

        double evasionAngle = angle_wrap(angleToOpponent + (goClockwise ? 1 : -1) * EVASION_DELTA_THETA);
        targetPoint = opponentPosEx - cv::Point2f(cos(evasionAngle), sin(evasionAngle)) * EVASION_RADIUS;

        // draw orange circle around opponent to show evasion radius
        cv::circle(drawingImage, opponentPosEx, EVASION_RADIUS, cv::Scalar(0, 165, 255), 4);
        // draw dot at target point
        cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 165, 255), 4);
    }

    const double ALLOW_REVERSE_MAX_VELOCITY = 20;
    const double MIN_VELOCITY_TIME_UNTIL_RUN_AWAY = 1.5;


    const double RUN_AWAY_DIST = 100;

    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);



    bool allowReverse = velocity < ALLOW_REVERSE_MAX_VELOCITY;

    // Drive to the opponent position
    RobotControllerMessage response = driveToPosition(ourPosition, ourAngleExtrapolator.Extrapolate(0), targetPoint, state, drawingImage, allowReverse);

    return response;
}
