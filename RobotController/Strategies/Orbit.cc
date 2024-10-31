#include "Orbit.h"
#include "../Extrapolator.h"
#include "../RobotConfig.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotController.h"
#include "../UIWidgets/GraphWidget.h"
#include "../PurePursuit.h"
// clock
#include "../UIWidgets/ClockWidget.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"

Orbit::Orbit()
{
}

void Orbit::StartOrbit()
{
    OdometryData opponentData = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0);

    // initialize orbit radius to the distance to the opponent
    _startingOrbitRadius = cv::norm(RobotController::GetInstance().odometry.Robot().robotPosition -
                                    opponentData.robotPosition);
}

void Orbit::StopOrbit()
{
}



RobotSimState Orbit::_ExtrapolateOurPos(double seconds_position, double seconds_angle)
{
    #define NUM_PREDICTION_ITERS 10

    // get odometry data
    OdometryData odoData = RobotController::GetInstance().odometry.Robot();

    // simulates the movement of the robot
    RobotSimulator robotSimulator;


    // remember if we had 0 seconds of pos extrap
    bool zero_position_seconds = seconds_position <= 0;

    // make sure seconds_position isn't exactly 0 so division doesn't blow up
    if (zero_position_seconds)
    {
        seconds_position = 0.01;
    }

    // now it is safe to compute the angle velocity scale
    double angle_vel_scale = seconds_angle / seconds_position;

    RobotSimState currentState;
    currentState.position = odoData.robotPosition;
    currentState.angle = odoData.robotAngle;
    currentState.velocity = odoData.robotVelocity;
    currentState.angularVelocity = odoData.robotAngleVelocity * angle_vel_scale;

    // predict where the robot will be in a couple milliseconds
    RobotSimState exState = robotSimulator.Simulate(currentState, seconds_position, NUM_PREDICTION_ITERS);

    // reset the position if it was originally 0.
    if (zero_position_seconds)
    {
        exState.position = odoData.robotPosition;
    }

    currentState.angularVelocity = 0;
    // project velocity onto angle
    // currentState.velocity = cv::Point2f(cos(currentState.angle), sin(currentState.angle)) * cv::norm(currentState.velocity);
    if (!LEAD_WITH_BAR)
    {
        currentState.velocity *= -1;
    }
    // RobotSimState exStateNoAngle = robotSimulator.Simulate(currentState, seconds_position, NUM_PREDICTION_ITERS);

    // // force the projected position to to be with no angle
    // exState.position = exStateNoAngle.position;

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
    static GraphWidget purePursuitRadiusGraph("Pure Pursuit Radius", 0, 100, "px", 1000);

    static double purePursuitRadius = PURE_PURSUIT_RADIUS;
    static Clock updateClock;
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
    double targetPurePursuitRadius = PURE_PURSUIT_RADIUS + (pow(velocityNorm, 1.5) * PP_RADIUS_VEL_SCALE);
    // calculate distance to the center of the circle
    double distToCenter = cv::norm(ourPosition - orbitCenter);
    // enforce the targetRadius to be between the min and max scales
    targetPurePursuitRadius = std::min(targetPurePursuitRadius, PURE_PURSUIT_RADIUS * MAX_PURE_PURSUIT_RADIUS_SCALE);
    targetPurePursuitRadius = std::max(targetPurePursuitRadius, PURE_PURSUIT_RADIUS * MIN_PURE_PURSUIT_RADIUS_SCALE);
    // make sure to never engulf the opponent
    targetPurePursuitRadius = std::min(targetPurePursuitRadius, distToCenter - 5);
    // more interpolation if more time has passed
    double interPolateAmount = std::min(1.0, deltaTime / PP_MOVAVG_TIME);
    // slowly change the radius to the target radius
    purePursuitRadius += (targetPurePursuitRadius - purePursuitRadius) * interPolateAmount;
    // re-enforce the smoothed radius to not engulf the opponent
    purePursuitRadius = std::min(purePursuitRadius, distToCenter - 5);
    // enforce pure pursuit radius to be at least 1
    purePursuitRadius = std::max(purePursuitRadius, 1.0);
    purePursuitRadiusGraph.AddData(purePursuitRadius);

    // check if nan or inf
    if (std::isnan(purePursuitRadius) || std::isinf(purePursuitRadius))
    {
        purePursuitRadius = PURE_PURSUIT_RADIUS;
    }

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
cv::Point2f ProjectPointOntoLineWithAngle(cv::Point2f linePoint, double lineAngle, cv::Point2f point)
{
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
 * Calculates the center of a robot's weapon given the robot's position and angle
 * 
 * @param pos The robot's position
 * @param angle The robot's angle
 * @param weapon_offset_from_center The distance from the center of the robot to the weapon
*/
cv::Point2f GetWeaponPos(cv::Point2f pos, double angle, double weapon_offset_from_center = 25)
{
    cv::Point2f weaponOffset = cv::Point2f(weapon_offset_from_center * cos(angle),
                                           weapon_offset_from_center * sin(angle));
    return pos + weaponOffset;
}

/**
 * Radially defines the orbit path.
 *
 * @param angle The input world angle
 * @param robotPosEx where our robot is (but angle might point to something different)
 * @param opponentWeaponPosEx The extrapolated position of the opponent's weapon
 * @param opponentCenterEx The center of the opponent's robot pos extrapolated
 * @param opponentAngleEx The angle of the opponent's robot extrapolated
 * @param circleDirection The direction of the circle
 *
 * @return The path point
 */
cv::Point2f Orbit::_CalculatePathPoint(double angle,
                                       cv::Point2f robotPosEx,
                                       cv::Point2f opponentWeaponPosEx,
                                       cv::Point2f opponentCenterEx,
                                       double opponentAngleEx,
                                       bool circleDirection,
                                       double* outCurrRadius,
                                       cv::Point2f* outCurrOrbitCenter)
{
    // calculate the danger level from 0 to 1
    double dangerLevel = _GetDangerLevel(angle, opponentAngleEx, circleDirection);
    // calculate the orbit radius
    double orbitRadius = _CalculateOrbitRadius(angle,
                                               robotPosEx,
                                               opponentWeaponPosEx,
                                               opponentAngleEx,
                                               RobotController::GetInstance().gamepad,
                                               circleDirection);

    // orbit around the opponent center when danger is low (killing)
    // otherwise orbit around the opponent weapon
    cv::Point2f currOrbitCenter = InterpolatePoints(opponentCenterEx, opponentWeaponPosEx, dangerLevel);

    // calculate the point on the path
    cv::Point2f point = currOrbitCenter + orbitRadius * cv::Point2f(cos(angle), sin(angle));


    // return the radius and orbit center if requested
    if (outCurrRadius != nullptr)
    {
        *outCurrRadius = orbitRadius;
    }

    if (outCurrOrbitCenter != nullptr)
    {
        *outCurrOrbitCenter = currOrbitCenter;
    }

    // return the point
    return point;
}

/**
 * Returns the linear velocity of us - opponent.
*/
double CalcVelocityTowardsOpponent()
{
    static double lastDistToOpponent = 0;
    static Clock deltaTimeClock;

    OdometryData odoData = RobotController::GetInstance().odometry.Robot();
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();

    double distToOpponent = cv::norm(odoData.robotPosition - opponentData.robotPosition);
    double deltaTime = deltaTimeClock.getElapsedTime();
    deltaTimeClock.markStart();
    double distToOpponentVel = 0;

    if (deltaTime > 0)
    {
        distToOpponentVel = (distToOpponent - lastDistToOpponent) / deltaTime;
    }

    lastDistToOpponent = distToOpponent;

    return -distToOpponentVel;
}

/**
 * Calculates the path to orbit given a direction
 * 
 * @param opponentWeaponPosEx The extrapolated position of the opponent's weapon
 * @param opponentCenterEx The center of the opponent's robot pos extrapolated
*/
std::vector<cv::Point2f> Orbit::_CalculateOrbitPath(cv::Point2f opponentWeaponPosEx,
                                                    cv::Point2f opponentCenterEx,
                                                    double opponentAngleEx,
                                                    cv::Point2f robotPosition,
                                                    bool circleDirection,
                                                    bool draw)
{
    // get the robot angle
    double robotAngle = RobotController::GetInstance().odometry.Robot().robotAngle;

    double angleToUs = atan2(robotPosition.y - opponentWeaponPosEx.y, robotPosition.x - opponentWeaponPosEx.x);

    const double ANGLE_STEP = 1.0 * TO_RAD;

    std::vector<cv::Point2f> path;
    // iterate for at most 180 degrees
    for (double angleOffset = 0; angleOffset < M_PI * 2; angleOffset += ANGLE_STEP)
    {
        double angle = angleToUs + angleOffset * (circleDirection ? 1 : -1);

        double orbitRadius = 0;
        // calculate the path point
        cv::Point2f point = _CalculatePathPoint(angle,
                                                robotPosition,
                                                opponentWeaponPosEx,
                                                opponentCenterEx,
                                                opponentAngleEx,
                                                circleDirection,
                                                &orbitRadius,
                                                nullptr);

        // push the point to the path
        path.push_back(point);

        // exit when the orbit radius hits 0 (we are at the opponent's position)
        if (orbitRadius == 0)
        {
            break;
        }
    }

    if (draw)
    {
        // now display the path by drawing line segments
        for (size_t i = 0; i < path.size() - 1; i++)
        {
            double progress = (double)i / path.size();
            cv::Scalar color = circleDirection ? cv::Scalar(0, progress * 255, (1.0 - progress) * 255) : cv::Scalar(progress * 255, (1.0 - progress) * 255, 0);
            
            cv::line(RobotController::GetInstance().GetDrawingImage(), path[i], path[i + 1], color, 2);
        }
    }

    return path;
}

/**
 * Calculates which direction we should be orbiting.
 * Defaults to the closest direction. Allows the user to force the direction using the dpad.
 * 
 * @param robotAngle The angle of the robot
 * @param angleToCenter The angle from the robot to the center of the orbit
*/
bool CalculateOrbitDirection(double robotAngle, double angleToCenter)
{
    // default the circule direction to be the closest
    bool circleDirection = angle_wrap(robotAngle - angleToCenter) < 0;
    // allow the user to force the direction
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

    // return the direction
    return circleDirection;
}

cv::Point2f Orbit::_GetOrbitFollowPoint(bool circleDirection, double& outCost, bool draw)
{
    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();

    ////////////////// GET POSITION DATA //////////////////
    // 1. get robot datas
    OdometryData odoData = RobotController::GetInstance().odometry.Robot();
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();
    // opponentData.robotAngle = Angle(opponentRotationSim); // TODO: don't just hardcode

    ////////////////// EXTRAPOLATE DATA //////////////////
    // 2. Extrapolate our position
    RobotSimState exState = _ExtrapolateOurPos(POSITION_EXTRAPOLATE_MS / 1000.0, ORBIT_ANGLE_EXTRAPOLATE_MS / 1000.0);
    // 3. Extrapolate opponent position
    OdometryData opponentExState = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0);
    // opponentExState.angle = Angle(opponentRotationSim); // TODO: don't just hardcode
    /////////////////////////////////////////////////////////

    // the orbit center is the opponent position
    cv::Point2f opponentWeaponPosEx = GetWeaponPos(opponentExState.robotPosition, opponentData.robotAngle, OPPONENT_WEAPON_OFFSET);

    ///////////////// CALCULATE ORBIT DIRECTION ///////////////
    // get the angle from us to the center
    cv::Point2f usToWeapon = opponentWeaponPosEx - odoData.robotPosition;
    double angleToWeapon = atan2(usToWeapon.y, usToWeapon.x);
    // add pi to get the angle from the center to us
    double angleWeaponToUs = angle_wrap(angleToWeapon + M_PI);
    ////////////////////////////////////////////////////////////////

    Gamepad& gamepad = RobotController::GetInstance().gamepad;
    // 4. Calculate the orbit radius
    double orbitRadius = _CalculateOrbitRadius(angleWeaponToUs, exState.position, opponentExState.robotPosition, opponentExState.robotAngle, gamepad, circleDirection);
    double dangerLevel = _GetDangerLevel(angleWeaponToUs, opponentExState.robotAngle, circleDirection);
    // calculate the pure pursuit radius given the velocity of the robot
    double purePursuitRadius = _CalculatePurePursuitRadius(odoData.robotPosition, opponentExState.robotPosition, orbitRadius);

    ////////////////// DRAWING //////////////////
    if (draw)
    {
        // draw a dot at the opponent's weapon
        safe_circle(drawingImage, opponentWeaponPosEx, 5, cv::Scalar(0, 0, 255), 2);
        // // draw blue circle around opponent
        // safe_circle(drawingImage, orbitCenter, orbitRadius, cv::Scalar(255, 0, 0), 2);
        // // draw light blue circle around opponent to show evasion radius
        // safe_circle(drawingImage, orbitCenter, orbitRadius, cv::Scalar(255, 165, 0), 1);
        // draw circle at our position with radius PURE_PURSUIT_RADIUS_PX
        safe_circle(drawingImage, odoData.robotPosition, purePursuitRadius, cv::Scalar(0, 255, 0), 1);
        // draw arrow at the ex state's angle
        cv::Point2f arrowEnd = exState.position + cv::Point2f(100.0 * cos(exState.angle), 100.0 * sin(exState.angle));
        safe_arrow(drawingImage, exState.position, arrowEnd, cv::Scalar(0, 255, 0), 2);

        // draw arrow from from opponent ex state to their weapon
        safe_arrow(drawingImage, opponentExState.robotPosition,
                        opponentExState.robotPosition + cv::Point2f(cos(opponentExState.robotAngle) * 50,
                                                               sin(opponentExState.robotAngle) * 50),
                        cv::Scalar(0, 255, 255), 2);
    }
    /////////////////////////////////////////////




    ///////////////////// CALCULATE TARGET POINT /////////////////////
    // calculate distance to the center of the circle
    double distToCenter = cv::norm(odoData.robotPosition - opponentWeaponPosEx);


    // get the path of the orbit (to use pure pursuit with)
    std::vector<cv::Point2f> path = _CalculateOrbitPath(opponentWeaponPosEx,
                                                        opponentExState.robotPosition,
                                                        opponentExState.robotAngle,
                                                        odoData.robotPosition,
                                                        circleDirection, draw);
    // calculate tangent points
    cv::Point2f tangent1;
    cv::Point2f tangent2;
    PurePursuit::CalculateTangentPoints(path, odoData.robotPosition, opponentWeaponPosEx, tangent1, tangent2);
    // choose the tangent based on the specified circle direction
    cv::Point2f tangentToConsider = circleDirection ? tangent1 : tangent2;

    // default the target to be tangent to the path
    cv::Point2f targetPoint = tangentToConsider;

    // // find the closest point on the path
    // double currPathAngle = PurePursuit::GetAngleOfClosestPathSegment(path, odoData.robotPosition);

    // next find the intersection of the pure pursuit circle with the circle around the opponent
    std::vector<cv::Point2f> circleIntersections = PurePursuit::followPath(odoData.robotPosition, path, purePursuitRadius);


    // if there are 2 intersections
    if (circleIntersections.size() >= 2)
    {
        // then use the first intersection
        targetPoint = circleDirection ? circleIntersections[0] : circleIntersections[1];
        if (draw) { cv::putText(drawingImage, "2 intersections", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2); }
    }
    else if (circleIntersections.size() == 1)
    {
        // if there is only 1 intersection, then use that
        targetPoint = circleIntersections[0];
        if (draw) { cv::putText(drawingImage, "1 intersection", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2); }
    }

    // if there is a tangent to consider
    if (tangentToConsider != cv::Point2f(0, 0))
    {
        // choose the further point (between target and targetPoint)
        if (cv::norm(odoData.robotPosition - tangentToConsider) > cv::norm(odoData.robotPosition - targetPoint))
        {
            targetPoint = tangentToConsider;
        }
    }

    // if distance to center is less than pure pursuit radius
    if (cv::norm(odoData.robotPosition - opponentData.robotPosition) < purePursuitRadius)
    {
        // if dangerous, go forwards => because otherwise will pure pursuit to center and we will die (high danger)
        if (dangerLevel > 0.95)
        {
            // // put text
            // if (draw) { cv::putText(drawingImage, "Close + danger -> forwards", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2); }
            // // cv::Point2f radial = orbitCenter + cv::Point2f(orbitRadius * cos(angleCenterToUs), orbitRadius * sin(angleCenterToUs));
            // // targetPoint = radial;

            // cv::Point2f pointAwayFromUs = odoData.robotPosition + cv::Point2f(100 * cos(odoData.robotAngle), 100 * sin(odoData.robotAngle));
            // targetPoint = pointAwayFromUs;
        }
        // otherwise, attack
        else
        {
            if (draw) { cv::putText(drawingImage, "Close + no danger -> attack", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2); }
            targetPoint = opponentExState.robotPosition;
        }
    }

    ////////////////////// FIRST CHECK IF WE ARE INSIDE THE PATH //////////////////////
    double currOrbitRadius = 0;
    cv::Point2f currOrbitCenter = cv::Point2f(0, 0);
    cv::Point2f closestPathPoint = _CalculatePathPoint(angleWeaponToUs,
                                                       exState.position,
                                                       opponentWeaponPosEx,
                                                       opponentExState.robotPosition,
                                                       opponentExState.robotAngle,
                                                       circleDirection,
                                                       &currOrbitRadius,
                                                       &currOrbitCenter);
    double distToOrbitCenter = cv::norm(odoData.robotPosition - currOrbitCenter);
    bool isInsidePath = distToOrbitCenter < currOrbitRadius;
    //////////////////////////////////////////////////////////////////////////////


    // // if we are inside the path
    // if (isInsidePath)
    // {
    //     // put text
    //     if (draw) { cv::putText(drawingImage, "Inside", cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2); }


    //     // check if going forwards would make us curve out less

    //     // calculate angle to target
    //     double angleToTarget = atan2(targetPoint.y - odoData.robotPosition.y, targetPoint.x - odoData.robotPosition.x);
    //     double angleDiff = angle_wrap(angleToTarget - odoData.robotAngle);

    //     // invert if negative
    //     if (!circleDirection)
    //     {
    //         angleDiff *= -1;
    //     }

    //     // if angleDiff is negative, then we are curving outwards => just go forwards instead
    //     if (angleDiff < 0)
    //     {
    //         // put text
    //         if (draw) { cv::putText(drawingImage, "Pure pursuit -> forwards", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2); }
    //         // cv::Point2f radial = orbitCenter + cv::Point2f(orbitRadius * cos(angleCenterToUs), orbitRadius * sin(angleCenterToUs));
    //         // targetPoint = radial;

    //         cv::Point2f pointAwayFromUs = odoData.robotPosition + cv::Point2f(100 * cos(odoData.robotAngle), 100 * sin(odoData.robotAngle));
    //         targetPoint = pointAwayFromUs;
    //     }
    // }











    /////////////////////////////// SCORING ///////////////////////////////

    // add turning score
    double angleToTarget = atan2(targetPoint.y - odoData.robotPosition.y, targetPoint.x - odoData.robotPosition.x);
    double angleDiff = angle_wrap(angleToTarget - odoData.robotAngle + (LEAD_WITH_BAR ? 0 : M_PI));
    double angleCost = abs(angleDiff) / (180 * TO_RAD);

    double angleToButt = angle_wrap(opponentExState.robotAngle + M_PI);
    // 0 -> 2pi
    double angleFromUsToButt = angle_wrap(angleToButt - angleWeaponToUs - M_PI) + M_PI;

    // draw an arrow from the opponent to the weapon
    safe_arrow(drawingImage, opponentExState.robotPosition,
                    opponentExState.robotPosition + cv::Point2f(cos(opponentExState.robotAngle + M_PI) * 50,
                                                           sin(opponentExState.robotAngle + M_PI) * 50),
                    cv::Scalar(50, 150, 50), 2);

    // draw an arrow from weapon to us
    safe_arrow(drawingImage, opponentWeaponPosEx,
                    odoData.robotPosition,
                    cv::Scalar(0, 100, 255), 2);

    // if we are going the other way, we must invert the angle
    if (!circleDirection)
    {
        angleFromUsToButt = 2 * M_PI - angleFromUsToButt;
    }


    // now let's score
    outCost = angleFromUsToButt / (2 * M_PI) + angleCost * ORBIT_PRESERVE_CURR_ANGLE_WEIGHT;

    // return the target point
    return targetPoint;
}

void DecideDirectionWithTriggers()
{
    static bool _lastLeftTrigger = false;
    static bool _lastRightTrigger = false;

    Gamepad& gamepad1 = RobotController::GetInstance().gamepad;

    // if the left trigger is pressed
    if (gamepad1.GetLeftTrigger() > 0.5)
    {
        // if the left trigger was not pressed last frame
        if (!_lastLeftTrigger)
        {
            // invert the direction
            LEAD_WITH_BAR = false;
        }
        // set the last left trigger to true
        _lastLeftTrigger = true;
    }
    else
    {
        // set the last left trigger to false
        _lastLeftTrigger = false;
    }

    // if the right trigger is pressed
    if (gamepad1.GetRightTrigger() > 0.5)
    {
        // if the right trigger was not pressed last frame
        if (!_lastRightTrigger)
        {
            // invert the direction
            LEAD_WITH_BAR = true;
        }
        // set the last right trigger to true
        _lastRightTrigger = true;
    }
    else
    {
        // set the last right trigger to false
        _lastRightTrigger = false;
    }
}

double Orbit::_CalcSpiralAggressionPreset()
{
    static bool l_trigger_last = false;
    static bool r_trigger_last = false;

    const double PRESET_STEP_SIZE = 0.3333;
    Gamepad& gamepad1 = RobotController::GetInstance().gamepad;
    bool l_trigger = gamepad1.GetLeftTrigger() > 0.5;
    bool r_trigger = gamepad1.GetRightTrigger() > 0.5;


    if (l_trigger && !l_trigger_last)
    {
        _spiralAggression -= PRESET_STEP_SIZE;
        _spiralAggression = std::max(0, _spiralAggression);
    }

    if (r_trigger && !r_trigger_last)
    {
        _spiralAggression += PRESET_STEP_SIZE;
        _spiralAggression = std::min(1, _spiralAggression);
    }


    l_trigger_last = l_trigger;
    r_trigger_last = r_trigger;


    return _spiralAggression;
}

void Orbit::_CalcStartAndEndAngle()
{
    static double smooth_agro_amount = 0;
    static Clock agroClock;
    const double AGRO_AMOUNT_TIME_FILTER = 0.06;

    // calculate the aggrsesion preset
    double aggression = _CalcSpiralAggressionPreset();

    // the minimum will be interpolated based on the aggression preset
    const double MIN_START_ANGLE = Interpolate(OPPONENT_SPIRAL_START_DEG, OPPONENT_SPIRAL_END_DEG, aggression) * TO_RAD;     // most dangerous condition => end shrinking at 90, closer to the weapon
    // the max will be set by 
    const double MAX_START_ANGLE = OPPONENT_SPIRAL_END_DEG;

    const double FULL_AGRO_VEL = 300; // px/s

    double velocityTowardsOpponent = CalcVelocityTowardsOpponent();

    double agro_amount = velocityTowardsOpponent / FULL_AGRO_VEL;
    // clip between 0 and 1
    agro_amount = std::max(0.0, agro_amount);
    agro_amount = std::min(1.0, agro_amount);

    double blendAmount = agroClock.getElapsedTime() / AGRO_AMOUNT_TIME_FILTER;
    agroClock.markStart();
    smooth_agro_amount = smooth_agro_amount * (1.0 - blendAmount) + agro_amount * blendAmount;

    // calculate the start angle
    // std::cout << "Velocity towards opponent: " << velocityTowardsOpponent << std::endl;
    // std::cout << "Agro amount: " << agro_amount << std::endl;

    // when past 90 degrees, the orbit radius shrinks. At 180, it is 0
    _START_ANGLE = MAX_START_ANGLE + (MIN_START_ANGLE - MAX_START_ANGLE) * smooth_agro_amount;
    // std::cout << "Start angle: " << _START_ANGLE << std::endl;
    _END_ANGLE = OPPONENT_SPIRAL_END_DEG * TO_RAD;
}



/**
 * OrbitMode
 * Orbits the robot around the opponent at a fixed distance.
 * @param message The current state of the robot
 */
DriverStationMessage Orbit::Execute(Gamepad& gamepad)
{
    _CalcStartAndEndAngle();
    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();
    // 2. Extrapolate our position
    RobotSimState exState = _ExtrapolateOurPos(POSITION_EXTRAPOLATE_MS / 1000.0, ORBIT_ANGLE_EXTRAPOLATE_MS / 1000.0);

    DecideDirectionWithTriggers();

    double out_cost1 = 0;
    bool circleDirection = true;
    cv::Point2f targetPoint1 = _GetOrbitFollowPoint(circleDirection, out_cost1, false);
    double out_cost2 = 0;
    circleDirection = false;
    cv::Point2f targetPoint2 = _GetOrbitFollowPoint(circleDirection, out_cost2, false);

    // put costs on screen
    // cv::putText(drawingImage, "Cost left: " + std::to_string(out_cost1), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    // cv::putText(drawingImage, "Cost right: " + std::to_string(out_cost2), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

    cv::Point2f targetPoint;
    if (out_cost1 < out_cost2)
    {
        circleDirection = true;
        // recalculate with draw
        targetPoint = _GetOrbitFollowPoint(circleDirection, out_cost1, true);
    }
    else
    {
        circleDirection = false;
        // recalculate with draw
        targetPoint = _GetOrbitFollowPoint(circleDirection, out_cost2, true);
    }
    
    // Draw the point
    safe_circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    bool allowReverse = false;

    // choose the direction to drive in
    RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    DriverStationMessage response = RobotMovement::HoldAngle(exState.position, targetPoint,
                                                             ORBIT_KD_PERCENT,
                                                             TURN_THRESH_1_DEG_ORBIT, TURN_THRESH_2_DEG_ORBIT,
                                                             MAX_TURN_POWER_PERCENT_ORBIT, MIN_TURN_POWER_PERCENT_ORBIT,
                                                             SCALE_DOWN_MOVEMENT_PERCENT_ORBIT,
                                                             direction);

    // // compute absolute angle
    // double deltaAngle = angle_wrap(atan2(targetPoint.y - exState.position.y, targetPoint.x - exState.position.x) - odoData.robotAngle);

    // if (circleDirection)
    // {
    //     deltaAngle *= -1;
    // }

    // if (distToCenter < orbitRadius && deltaAngle > TO_RAD * 40)
    // {
    //     // PROHIB TURNING
    //     response.autoDrive.MAX_TURN_POWER_PERCENT = 0;
    //     response.autoDrive.MIN_TURN_POWER_PERCENT = 0;
    //     response.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT = 0;
    // }



    // // if on inside of circle and pointed outwards
    // if (distToCenter < orbitRadius && abs(angle_wrap(angleToCenter + M_PI - odoData.robotAngle)) < 45 * TO_RAD)
    // {
    //     // force 100% move power
    //     response.autoDrive.movement = 1.0;
    //     // response.turn *= 0.5;
    // }

    return response;
}

/**
 * Returns a scalar from 0 to 1 that represents the danger level (the orbit radius fraction)
 * 
 * @param angleOpponentToPoint the world angle from the opponent to the point you're interrested in
 * @param opponentAngle the world angle of the opponent
 * @param orbitDirection which direction you are orbiting
 */
double Orbit::_GetDangerLevel(double angleOpponentToPoint,
                              double opponentAngle,
                              bool orbitDirection)
{
    // calculate the number of radians to the weapon
    double angleToWeaponAbs = angle_wrap(angleOpponentToPoint - opponentAngle) * (orbitDirection ? 1 : -1);

    double shrinkAmount = (angleToWeaponAbs - _START_ANGLE) / (_END_ANGLE - _START_ANGLE);

    shrinkAmount = std::max(0.0, shrinkAmount);
    shrinkAmount = std::min(1.0, shrinkAmount);

    return 1.0 - shrinkAmount;
}

// time to grow to the regular orbit radius
#define SPEED_TO_GROW_ORBIT_RADIUS_PX_P_S 100.0

/**
 * Calculates the current radius given:
 * 
 * 1. angleOpponentToPoint => the input angle to our radial function. Angle originates from the opponent
 * 2. robotPosEx => where our robot
 * 3. opponentWeaponPosEx => where the opponent's weapon is
 * 4. opponentAngle => the angle the opponent is facing their weapon
 * 5. gamepad => the gamepad
 * 6. orbitDirection => the direction of the orbit
 */
double Orbit::_CalculateOrbitRadius(double angleOpponentToPoint,
                                    cv::Point2f robotPosEx,
                                    cv::Point2f opponentWeaponPosEx,
                                    double opponentAngle,
                                    Gamepad &gamepad,
                                    bool orbitDirection)
{
    // scale the radius based on the triggers
    double orbitRadius = ORBIT_RADIUS;

    // get the danger level
    double dangerLevel = _GetDangerLevel(angleOpponentToPoint, opponentAngle, orbitDirection);


    // multiply the radius by the danger level
    orbitRadius *= dangerLevel;

    // 
    double angleOpponentToRobot = atan2(robotPosEx.y - opponentWeaponPosEx.y, robotPosEx.x - opponentWeaponPosEx.x);
    double deltaAngle = abs(angle_wrap(angleOpponentToPoint - angleOpponentToRobot));
    double robotsRadius = cv::norm(robotPosEx - opponentWeaponPosEx); // calculate the distance to the center of the opponent

    // calculate how much distance there is from the robot to the outside of the path
    double distanceToOutSideOfPath = orbitRadius - robotsRadius;

    // If we are inside the path, we should slowly grow the path from us out to
    // the our target radius. the more inside the path we are, the more angle it
    // will take to transition
    const double ANGLE_TRANSITION_PERIOD = (distanceToOutSideOfPath / ORBIT_RADIUS) * 150 * TO_RAD;

    if (deltaAngle < ANGLE_TRANSITION_PERIOD)
    {
        double transitionFactor = deltaAngle / ANGLE_TRANSITION_PERIOD;
        // BLEND from our distance to the opponent to whatever above wants us to follow at based on the transition factor
        double blended = transitionFactor * orbitRadius  + (1.0 - transitionFactor) * robotsRadius;
        
        orbitRadius = std::min(blended, orbitRadius);
    }

    // orbitRadius = robotsRadius;

    // return the radius
    return orbitRadius;
}

/**
 * Makes sure the currentTargetPoint doesn't go more into the circle than the tangent point
 * 
 * @param gamepad The gamepad
 * @param ourPosition Our position
 * @param opponentWeaponPosEx The opponent's position
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
