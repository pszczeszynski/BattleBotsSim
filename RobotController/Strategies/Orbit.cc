#include "Orbit.h"
#include "../Extrapolator.h"
#include "../RobotConfig.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotController.h"

Orbit::Orbit()
{
}

RobotSimState Orbit::_ExtrapolateOurPos(double seconds_position, double seconds_angle)
{
    #define NUM_PREDICTION_ITERS 1

    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    RobotSimState currentState;
    currentState.position = RobotOdometry::Robot().GetPosition();
    currentState.angle = RobotOdometry::Robot().GetAngle();
    currentState.velocity = RobotOdometry::Robot().GetVelocity();
    double angleExtrapolate = KILL_ANGLE_EXTRAPOLATE_MS;
    currentState.angularVelocity = RobotOdometry::Robot().GetAngleVelocity() * seconds_angle / seconds_position;

    // predict where the robot will be in a couple milliseconds
    RobotSimState exState = robotSimulator.Simulate(currentState, seconds_position, NUM_PREDICTION_ITERS);

    return exState;
}


/**
 * OrbitMode
 * Orbits the robot around the opponent at a fixed distance.
 * @param message The current state of the robot
 */
#define MAX_PURE_PURSUIT_RADIUS_SCALE 3.0
#define MIN_PURE_PURSUIT_RADIUS_SCALE 0.5
#define USE_TANGENT_POINTS_DIST ORBIT_RADIUS * 2
DriveCommand Orbit::Execute(Gamepad& gamepad)
{
    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();


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

    double orbitRadius = _CalculateOrbitRadius(opponentPosEx, gamepad);

    // get the angle from us to the opponent
    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    // add pi to get the angle from the opponent to us
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);

    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, orbitRadius, cv::Scalar(255, 0, 0), 2);
    // // draw arrow from opponent position at opponent angle
    // cv::Point2f arrowEnd = opponentPos + cv::Point2f(100.0 * cos(RobotOdometry::Opponent().GetAngle()), 100.0 * sin(RobotOdometry::Opponent().GetAngle()));
    // cv::arrowedLine(drawingImage, opponentPos, arrowEnd, cv::Scalar(255, 0, 0), 2);

    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, orbitRadius, cv::Scalar(255, 165, 0), 1);


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
    // enforce pure pursuit radius to be at least 1
    purePursuitRadius = std::max(purePursuitRadius, 1.0);

    // default the target to be radially from the angle from the opponent to us
    cv::Point2f targetPoint = opponentPosEx + cv::Point2f(orbitRadius * cos(angleOpponentToUs), orbitRadius * sin(angleOpponentToUs));

    // next find the intersection of the pure pursuit circle with the circle around the opponent
    std::vector<cv::Point2f> circleIntersections = CirclesIntersect(ourPosition, purePursuitRadius, opponentPosEx, orbitRadius);

#ifndef HARDCORE
    // draw circle at our position with radius PURE_PURSUIT_RADIUS_PX
    cv::circle(drawingImage, ourPosition, purePursuitRadius, cv::Scalar(0, 255, 0), 1);
#endif

    bool circleDirection = angle_wrap(ourAngle - angleToOpponent) < 0;
    bool drivingBackwards = gamepad.GetRightStickY() < -0.03;


    // // if we are FAR to the center of the circle
    // if (distToCenter > orbitRadius * 1.4)
    // {
    //     // then don't drive backwards
    //     drivingBackwards = abs(angle_wrap(ourAngle - angleToOpponent)) > 90 * TO_RAD;
    // }


    static Clock circleDirectionTimer;

    if (RobotController::GetInstance().gamepad.GetDpadLeft())
    {
        circleDirection = true;
    }
    else if (RobotController::GetInstance().gamepad.GetDpadRight())
    {
        circleDirection = false;
    }

    // check if the user wants to drive backwards, invert the circle direction
    if (drivingBackwards)
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

    // enforce that the target point is not more aggressive than the tangent point towards the center of the circle
    targetPoint = _NoMoreAggressiveThanTangent(gamepad,
                                               ourPosition,
                                               opponentPosEx,
                                               orbitRadius,
                                               targetPoint,
                                               circleDirection);

#ifndef HARDCORE
    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);
#endif

    bool allowReverse = false;
    RobotSimState exState = _ExtrapolateOurPos(POSITION_EXTRAPOLATE_MS / 1000.0, ORBIT_ANGLE_EXTRAPOLATE_MS / 1000.0);

    // choose the direction to drive in
    RobotMovement::DriveDirection direction = drivingBackwards ? RobotMovement::DriveDirection::Backward : RobotMovement::DriveDirection::Forward;

    // draw arrow at the ex state's angle
    cv::Point2f arrowEnd = exState.position + cv::Point2f(100.0 * cos(exState.angle), 100.0 * sin(exState.angle));
    cv::arrowedLine(drawingImage, exState.position, arrowEnd, cv::Scalar(0, 255, 0), 2);

    DriveCommand response = DriveToPosition(exState, targetPoint, direction);

    // if on inside of circle and pointed outwards
    if (distToCenter < orbitRadius && abs(angle_wrap(angleToOpponent + M_PI - ourAngle)) < 45 * TO_RAD)
    {
        // force 100% move power
        response.movement = 1.0;
        response.turn *= 0.5;
    }

    return response;
}

double Orbit::_CalculateOrbitRadius(cv::Point2f opponentPosEx, Gamepad& gamepad)
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
 * 
 * @param gamepad The gamepad
 * @param ourPosition Our position
 * @param opponentPosEx The opponent's position
 * @param orbitRadius The radius of the orbit
 * @param currentTargetPoint The current target point
 * @param circleDirection The direction of the circle
 * 
 * @return The new target point
 */
cv::Point2f Orbit::_NoMoreAggressiveThanTangent(Gamepad &gamepad,
                                                cv::Point2f ourPosition,
                                                cv::Point2f opponentPosEx,
                                                double orbitRadius,
                                                cv::Point2f currentTargetPoint,
                                                bool circleDirection)
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