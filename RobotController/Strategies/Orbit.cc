#include "Orbit.h"
#include "../Extrapolator.h"
#include "../RobotConfig.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotController.h"


Orbit::Orbit()
{
}

void Orbit::StartOrbit()
{
    _orbitState = OrbitState::LARGE_CIRCLE;
}

void Orbit::StopOrbit()
{
    _orbitState = OrbitState::IDLE;
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

    currentState.angularVelocity = 0;
    // project velocity onto angle
    currentState.velocity = cv::Point2f(cos(currentState.angle), sin(currentState.angle)) * cv::norm(currentState.velocity);
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
#define MIN_PURE_PURSUIT_RADIUS_SCALE 0.5
double Orbit::_CalculatePurePursuitRadius(cv::Point2f ourPosition, cv::Point2f orbitCenter, double orbitRadius)
{
    static double purePursuitRadius = PURE_PURSUIT_RADIUS;
    // get our velocity
    double velocityNorm = cv::norm(RobotOdometry::Robot().GetVelocity());
    // scale the radius based on our velocity
    double targetPurePursuitRadius = PURE_PURSUIT_RADIUS * velocityNorm / 200.0;
    // calculate distance to the center of the circle
    double distToCenter = cv::norm(ourPosition - orbitCenter);
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
    // return the radius
    return purePursuitRadius;
}

/**
 * OrbitMode
 * Orbits the robot around the opponent at a fixed distance.
 * @param message The current state of the robot
 */
DriveCommand Orbit::Execute(Gamepad& gamepad)
{
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};

    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();

    // our pos + angle
    cv::Point2f ourPosition = RobotOdometry::Robot().GetPosition();
    double ourAngle = RobotOdometry::Robot().GetAngle();

    // opponent pos + angle
    cv::Point2f opponentPos = RobotOdometry::Opponent().GetPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    opponentPosEx = RobotOdometry::Opponent().GetVelocity() * OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 + opponentPos;


    // the orbit center is the opponent position for the large circle, otherwise it's the go around center
    cv::Point2f orbitCenter = _orbitState != OrbitState::GO_AROUND ? opponentPosEx : _goAroundCenter;

    double orbitRadiusLargeCircle = _CalculateOrbitRadius(orbitCenter, gamepad);
    // calculate the radius of the orbit. Depending on the state, it could be the large circle radius or the go around radius
    double orbitRadius = _orbitState != OrbitState::GO_AROUND ? orbitRadiusLargeCircle : GO_AROUND_RADIUS;

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
    bool drivingBackwards = gamepad.GetRightStickY() < -0.03;

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

    cv::Point2f targetPointBeforeTangent = targetPoint;
    // enforce that the target point is not more aggressive than the tangent point towards the center of the circle
    targetPoint = _NoMoreAggressiveThanTangent(gamepad,
                                               ourPosition,
                                               orbitCenter,
                                               orbitRadius,
                                               targetPoint,
                                               circleDirection);

    cv::line(drawingImage, cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_TOP), cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_TOP), cv::Scalar(0, 0, 255), 2);
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_TOP), cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_BOTTOM), cv::Scalar(0, 0, 255), 2);
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_BOTTOM), cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_BOTTOM), cv::Scalar(0, 0, 255), 2);
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_BOTTOM), cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_TOP), cv::Scalar(0, 0, 255), 2);

    static cv::Point2f lastTargetPointBeforeTangent = cv::Point2f(0, 0);
    static cv::Point2f ourPosLast = ourPosition;
    // if we're in the large circle state
    if (_orbitState == OrbitState::LARGE_CIRCLE && gamepad.GetDpadUp())
    {
        std::vector<cv::Point2f> goAroundCenters = _ComputeGoAroundCircleCenters(opponentPosEx, orbitRadius);
        // draw line from last target point to current target point
        cv::line(drawingImage, lastTargetPointBeforeTangent, targetPoint, cv::Scalar(255, 255, 255), 2);

        for (cv::Point2f goAroundCenter : goAroundCenters)
        {
            // draw the go around circle
            cv::circle(drawingImage, goAroundCenter, GO_AROUND_RADIUS, cv::Scalar(255, 255, 255), 2);

            // draw line from opponent to go around center
            cv::line(drawingImage, opponentPosEx, goAroundCenter, cv::Scalar(100, 255, 100), 2);

            cv::Point2f intersection;
            // IF THE TARGET POINT CROSSES the line from the opponent pos to the go around center pos
            if (SegmentsIntersect(lastTargetPointBeforeTangent, targetPointBeforeTangent, opponentPosEx, goAroundCenter, intersection))
            {
                // initiate go around (follow the smaller circle)
                _orbitState = OrbitState::GO_AROUND;
                _goAroundCenter = goAroundCenter;
            }
        }
    }
    else if (_orbitState == OrbitState::GO_AROUND)
    {
        // draw the go around circle in purple
        cv::circle(drawingImage, _goAroundCenter, GO_AROUND_RADIUS, cv::Scalar(255, 0, 255), 2);
        cv::Point2f intersection;
        // exit orbit when you cross the line from the opponent to the go around center
        // you must also be on the other side of the go around circle
        if (SegmentsIntersect(ourPosLast, ourPosition, _goAroundCenter,
                              _goAroundCenter + (_goAroundCenter - opponentPosEx), intersection))
        {
            _orbitState = OrbitState::LARGE_CIRCLE;
        }
    }

    lastTargetPointBeforeTangent = targetPointBeforeTangent;
    ourPosLast = ourPosition;

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
    if (distToCenter < orbitRadius && abs(angle_wrap(angleToCenter + M_PI - ourAngle)) < 45 * TO_RAD)
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

bool Orbit::_IsPointOutOfBounds(cv::Point2f point)
{
    // draw the bounds on the drawing image
    cv::Mat &drawingImage = RobotController::GetInstance().GetDrawingImage();
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_TOP), cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_TOP), cv::Scalar(0, 0, 255), 2);
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_TOP), cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_BOTTOM), cv::Scalar(0, 0, 255), 2);
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_RIGHT, WALL_BOUNDS_BOTTOM), cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_BOTTOM), cv::Scalar(0, 0, 255), 2);
    cv::line(drawingImage, cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_BOTTOM), cv::Point(WALL_BOUNDS_LEFT, WALL_BOUNDS_TOP), cv::Scalar(0, 0, 255), 2);

    return point.x < WALL_BOUNDS_LEFT || point.x > WALL_BOUNDS_RIGHT || point.y < WALL_BOUNDS_TOP || point.y > WALL_BOUNDS_BOTTOM;
}

/**
 * Computes the centers of the go around circles
 * 
 * @param opponentPos The opponent's position
 * @param orbitRadius The radius of the orbit
 * @param wallX The x position of the wall (0 if not vertical)
 * @param wallY The y position of the wall (0 if not horizontal)
 * @param vertical Whether the wall is vertical
 * 
 * @return The centers of the go around circles
 */
std::vector<cv::Point2f> computeGoAroundCircleCenters(const cv::Point2f &opponentPos,
                                                      int orbitRadius,
                                                      int wallX, int wallY,
                                                      bool vertical) {
    std::vector<cv::Point2f> returnedPoints;
    double sumOfRadii = GO_AROUND_RADIUS + orbitRadius;
    int distToWall = vertical ? std::abs(wallY - opponentPos.y) : std::abs(wallX - opponentPos.x);
    int goAroundCirclePrimary = wallX + (vertical ? 0 : (wallX > opponentPos.x ? -GO_AROUND_RADIUS : GO_AROUND_RADIUS));
    int goAroundCircleSecondary = wallY + (vertical ? (wallY > opponentPos.y ? -GO_AROUND_RADIUS : GO_AROUND_RADIUS) : 0);
    int triangleWidth = distToWall - GO_AROUND_RADIUS;
    double toSquareRoot = pow(sumOfRadii, 2) - pow(triangleWidth, 2);

    if (toSquareRoot < 0) {
        // No valid circles can be drawn if the square root is negative.
        return returnedPoints;
    }

    int delta = static_cast<int>(sqrt(toSquareRoot));
    if (vertical) {
        returnedPoints.emplace_back(opponentPos.x - delta, goAroundCircleSecondary);
        returnedPoints.emplace_back(opponentPos.x + delta, goAroundCircleSecondary);
    } else {
        returnedPoints.emplace_back(goAroundCirclePrimary, opponentPos.y - delta);
        returnedPoints.emplace_back(goAroundCirclePrimary, opponentPos.y + delta);
    }

    return returnedPoints;
}

std::vector<cv::Point2f> Orbit::_ComputeGoAroundCircleCenters(cv::Point2f opponentPosEx, double orbitRadius)
{
    std::vector<cv::Point2f> goAroundCirclePoints;

    // if the orbit doesn't cross the boundary, then don't bother with the go around circles
    if (!_IsPointOutOfBounds(opponentPosEx + cv::Point2f(orbitRadius, orbitRadius)) &&
        !_IsPointOutOfBounds(opponentPosEx - cv::Point2f(orbitRadius, orbitRadius)))
    {
        return goAroundCirclePoints;
    }

    std::vector<cv::Point2f> points1 = computeGoAroundCircleCenters(opponentPosEx, orbitRadius, WALL_BOUNDS_RIGHT, 0, false);
    std::vector<cv::Point2f> points2 = computeGoAroundCircleCenters(opponentPosEx, orbitRadius, WALL_BOUNDS_LEFT, 0, false);
    std::vector<cv::Point2f> points3 = computeGoAroundCircleCenters(opponentPosEx, orbitRadius, 0, WALL_BOUNDS_BOTTOM, true);
    std::vector<cv::Point2f> points4 = computeGoAroundCircleCenters(opponentPosEx, orbitRadius, 0, WALL_BOUNDS_TOP, true);

    // combine all the points
    goAroundCirclePoints.insert(goAroundCirclePoints.end(), points1.begin(), points1.end());
    goAroundCirclePoints.insert(goAroundCirclePoints.end(), points2.begin(), points2.end());
    goAroundCirclePoints.insert(goAroundCirclePoints.end(), points3.begin(), points3.end());
    goAroundCirclePoints.insert(goAroundCirclePoints.end(), points4.begin(), points4.end());

    // filter out the circles that exceed the bounds
    for (int i = goAroundCirclePoints.size() - 1; i >= 0; i--)
    {
        if (_IsPointOutOfBounds(goAroundCirclePoints[i] + cv::Point2f(GO_AROUND_RADIUS, GO_AROUND_RADIUS)) ||
            _IsPointOutOfBounds(goAroundCirclePoints[i] - cv::Point2f(GO_AROUND_RADIUS, GO_AROUND_RADIUS)))
        {
            goAroundCirclePoints.erase(goAroundCirclePoints.begin() + i);
        }
    }

    // choose the 2 circles that
    // 1. give us the most room to orbit normally (segment length that is all within bounds)
    // 2. give us the most 

    // draw all the go around circles
    cv::Mat &drawingImage = RobotController::GetInstance().GetDrawingImage();
    for (int i = 0; i < goAroundCirclePoints.size(); i++)
    {
        cv::circle(drawingImage, goAroundCirclePoints[i], GO_AROUND_RADIUS, cv::Scalar(255, 0, 255), 2);
    }

    return goAroundCirclePoints;
}