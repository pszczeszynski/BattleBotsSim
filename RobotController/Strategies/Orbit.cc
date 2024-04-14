#include "Orbit.h"
#include "../Extrapolator.h"
#include "../RobotConfig.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotController.h"
#include "../UIWidgets/GraphWidget.h"

Orbit::Orbit()
{
}

void Orbit::StartOrbit()
{
    _orbitState = OrbitState::LARGE_CIRCLE;

    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();
    opponentData.Extrapolate(ClockWidget::programClock.getElapsedTime() + OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0);

    // initialize orbit radius to the distance to the opponent
    _startingOrbitRadius = cv::norm(RobotController::GetInstance().odometry.Robot().robotPosition -
                                    opponentData.robotPosition);
}

void Orbit::StopOrbit()
{
    _orbitState = OrbitState::IDLE;
}

RobotSimState Orbit::_ExtrapolateOurPos(double seconds_position, double seconds_angle)
{
    #define NUM_PREDICTION_ITERS 1

    // get odometry data
    OdometryData odoData =  RobotController::GetInstance().odometry.Robot();

    // simulates the movement of the robot
    RobotSimulator robotSimulator;

    RobotSimState currentState;
    currentState.position = odoData.robotPosition;
    currentState.angle = odoData.robotAngle;
    currentState.velocity = odoData.robotVelocity;
    currentState.angularVelocity = odoData.robotAngleVelocity * seconds_angle / seconds_position;

    // predict where the robot will be in a couple milliseconds
    RobotSimState exState = robotSimulator.Simulate(currentState, seconds_position, NUM_PREDICTION_ITERS);

    currentState.angularVelocity = 0;
    // project velocity onto angle
    currentState.velocity = cv::Point2f(cos(currentState.angle), sin(currentState.angle)) * cv::norm(currentState.velocity);
    if (!LEAD_WITH_BAR)
    {
        currentState.velocity *= -1;
    }
    RobotSimState exStateNoAngle = robotSimulator.Simulate(currentState, seconds_position, NUM_PREDICTION_ITERS);

    // force the projected position to to be with no angle
    exState.position = exStateNoAngle.position;

    return exState;
}


/**
 * Calculates the current pure pursuit radius given the velocity of the robot
 * 
 * @param ourPosition Our position
 * @param orbitCenter The center of the orbit
 * @param orbitRadius The radius of the orbit
 * @return The pure pursuit radius
*/
#define MAX_PURE_PURSUIT_RADIUS_SCALE 3.0
#define MIN_PURE_PURSUIT_RADIUS_SCALE 1.0
#define TIME_BETWEEN_RADIUS_UPDATES 0.01
double Orbit::_CalculatePurePursuitRadius(cv::Point2f ourPosition, cv::Point2f orbitCenter, double orbitRadius)
{
    // get odometry data
    OdometryData odoData =  RobotController::GetInstance().odometry.Robot();

    static double purePursuitRadius = PURE_PURSUIT_RADIUS;
    static Clock updateClock;
    static GraphWidget purePursuitRadiusGraph("Pure Pursuit Radius", 0, 200, "px", 1000);
    double deltaTime = updateClock.getElapsedTime();

    // if the time since the last update is too small, then don't update the radius
    if (deltaTime < TIME_BETWEEN_RADIUS_UPDATES)
    {
        return purePursuitRadius;
    }

    // restart the clock
    updateClock.markStart();

    // get our velocity
    double velocityNorm = cv::norm(odoData.robotVelocity);
    // scale the radius based on our velocity
    double targetPurePursuitRadius = PURE_PURSUIT_RADIUS * velocityNorm / 200.0;
    // calculate distance to the center of the circle
    double distToCenter = cv::norm(ourPosition - orbitCenter);
    // the radius shouldn't be larger than the distance to the other edge of the circle
    double distanceToOtherEdgeOfCircle = distToCenter + orbitRadius;
    // enforce the targetRadius to be between the min and max scales
    targetPurePursuitRadius = std::min(targetPurePursuitRadius, PURE_PURSUIT_RADIUS * MAX_PURE_PURSUIT_RADIUS_SCALE);
    targetPurePursuitRadius = std::max(targetPurePursuitRadius, PURE_PURSUIT_RADIUS * MIN_PURE_PURSUIT_RADIUS_SCALE);
    // make sure to never engulf the orbit circle
    targetPurePursuitRadius = std::min(targetPurePursuitRadius, distanceToOtherEdgeOfCircle - 5);
    // more interpolation if more time has passed
    double interPolateAmount = std::min(1.0, deltaTime / ORBIT_RADIUS_MOVAVG_SPEED);
    // slowly change the radius to the target radius
    purePursuitRadius += (targetPurePursuitRadius - purePursuitRadius) * interPolateAmount;
    // re-enforce the smoothed radius to not engulf the circle
    purePursuitRadius = std::min(purePursuitRadius, distanceToOtherEdgeOfCircle - 5);
    // enforce pure pursuit radius to be at least 1
    purePursuitRadius = std::max(purePursuitRadius, 1.0);

    // add the radius to the graph
    purePursuitRadiusGraph.AddData(purePursuitRadius);

    // return the radius
    return purePursuitRadius;
}

/**
 * Projects a point onto a line with a given angle
 * 
 * @param linePoint The point on the line
 * @param lineAngle The angle of the line
 * @param point The point to project
 * @return The projected point
*/
cv::Point2f ProjectPointOntoLineWithAngle(cv::Point2f linePoint, double lineAngle, cv::Point2f point) {
    // Direction vector of the line
    cv::Point2f lineDir(cos(lineAngle), sin(lineAngle));

    // Vector from linePoint to point
    cv::Point2f pointVector = point - linePoint;

    // Project pointVector onto lineDir
    double dotProduct = pointVector.x * lineDir.x + pointVector.y * lineDir.y;
    cv::Point2f projection = linePoint + dotProduct * lineDir;

    return projection;
}

/**
 * OrbitMode
 * Orbits the robot around the opponent at a fixed distance.
 * @param message The current state of the robot
 */
DriverStationMessage Orbit::Execute(Gamepad& gamepad)
{
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};

    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();

    // our pos + angle
    OdometryData odoData =  RobotController::GetInstance().odometry.Robot();

    cv::Point2f ourPosition = odoData.robotPosition;
    double ourAngle = odoData.robotAngle;

    // opponent pos + angle
    OdometryData opponentData =  RobotController::GetInstance().odometry.Opponent();
    cv::Point2f opponentPos = opponentData.robotPosition;
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    opponentPosEx = opponentData.robotVelocity * OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 + opponentPos;

    // the orbit center is the opponent position
    cv::Point2f orbitCenter = opponentPosEx;

    double orbitRadius = _CalculateOrbitRadius(orbitCenter, gamepad);

    // get the angle from us to the center
    cv::Point2f usToCenter = orbitCenter - ourPosition;
    double angleToCenter = atan2(usToCenter.y, usToCenter.x);
    // add pi to get the angle from the center to us
    double angleCenterToUs = angle_wrap(angleToCenter + M_PI);
    // calculate distance to the center of the circle
    double distToCenter = cv::norm(ourPosition - orbitCenter);
    // enforce pure pursuit radius to be at least 1
    double purePursuitRadius = _CalculatePurePursuitRadius(ourPosition, orbitCenter, orbitRadius);


    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, orbitRadius, cv::Scalar(255, 0, 0), 2);
    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, orbitRadius, cv::Scalar(255, 165, 0), 1);


    // default the target to be radially from the angle from the center to us
    cv::Point2f targetPoint = orbitCenter + cv::Point2f(orbitRadius * cos(angleCenterToUs), orbitRadius * sin(angleCenterToUs));

    // next find the intersection of the pure pursuit circle with the circle around the opponent
    std::vector<cv::Point2f> circleIntersections = CirclesIntersect(ourPosition, purePursuitRadius, orbitCenter, orbitRadius);

#ifndef HARDCORE
    // draw circle at our position with radius PURE_PURSUIT_RADIUS_PX
    cv::circle(drawingImage, ourPosition, purePursuitRadius, cv::Scalar(0, 255, 0), 1);
#endif

    bool circleDirection = angle_wrap(ourAngle - angleToCenter) < 0;

    if (RobotController::GetInstance().gamepad.GetDpadLeft())
    {
        circleDirection = true;
    }
    else if (RobotController::GetInstance().gamepad.GetDpadRight())
    {
        circleDirection = false;
    }

    // check if the user wants to drive backwards, invert the circle direction
    if (!LEAD_WITH_BAR)
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

    cv::Point2f targetPointBeforeTangent = targetPoint;
    // enforce that the target point is not more aggressive than the tangent point towards the center of the circle
    targetPoint = _NoMoreAggressiveThanTangent(gamepad,
                                               ourPosition,
                                               orbitCenter,
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
    RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    // draw arrow at the ex state's angle
    cv::Point2f arrowEnd = exState.position + cv::Point2f(100.0 * cos(exState.angle), 100.0 * sin(exState.angle));
    cv::arrowedLine(drawingImage, exState.position, arrowEnd, cv::Scalar(0, 255, 0), 2);

    DriverStationMessage response = HoldAngle(exState.position, targetPoint,
                                              ORBIT_ANGLE_EXTRAPOLATE_MS,
                                              TURN_THRESH_1_DEG_ORBIT, TURN_THRESH_2_DEG_ORBIT,
                                              MAX_TURN_POWER_PERCENT_ORBIT, MIN_TURN_POWER_PERCENT_ORBIT,
                                              SCALE_DOWN_MOVEMENT_PERCENT_ORBIT,
                                              direction);

    // compute absolute angle
    double deltaAngle = angle_wrap(atan2(targetPoint.y - exState.position.y, targetPoint.x - exState.position.x) - ourAngle);

    if (circleDirection)
    {
        deltaAngle *= -1;
    }

    if (distToCenter < orbitRadius && deltaAngle > TO_RAD * 40)
    {
        // PROHIB TURNING
        response.autoDrive.MAX_TURN_POWER_PERCENT = 0;
        response.autoDrive.MIN_TURN_POWER_PERCENT = 0;
        response.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT = 0;
    }

    // // if on inside of circle and pointed outwards
    // if (distToCenter < orbitRadius && abs(angle_wrap(angleToCenter + M_PI - ourAngle)) < 45 * TO_RAD)
    // {
    //     // force 100% move power
    //     response.autoDrive.movement = 1.0;
    //     // response.turn *= 0.5;
    // }

    return response;
}


// time to grow to the regular orbit radius
#define SPEED_TO_GROW_ORBIT_RADIUS_PX_P_S 100.0

double Orbit::_CalculateOrbitRadius(cv::Point2f opponentPosEx, Gamepad& gamepad)
{
    static Clock updateClock;
    // get odometry data
    OdometryData odoData = RobotController::GetInstance().odometry.Robot();
    // our pos + angle
    cv::Point2f ourPosition = odoData.robotPosition;

    double orbitRadius = ORBIT_RADIUS;
    double distToOpponent = cv::norm(ourPosition - opponentPosEx);

    // scale the radius based on the triggers
    orbitRadius *= 1.0 + gamepad.GetRightTrigger();
    orbitRadius /= 1.0 + gamepad.GetLeftTrigger();

    if (_startingOrbitRadius < orbitRadius)
    {
        // orbitRadius = _startingOrbitRadius;
    }

    // move the _startingOrbitRadius towards the orbitRadius
    _startingOrbitRadius += SPEED_TO_GROW_ORBIT_RADIUS_PX_P_S * updateClock.getElapsedTime();
    updateClock.markStart();


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
