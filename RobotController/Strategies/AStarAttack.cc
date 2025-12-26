#include "AStarAttack.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../UIWidgets/GraphWidget.h"
#include "../UIWidgets/ClockWidget.h"
#include "../MathUtils.h"
#include <cstdlib>
#include "DisplayUtils.h"

AStarAttack* AStarAttack::_instance = nullptr;

AStarAttack::AStarAttack()
{
    _instance = this;
    // define the field
    float minX = 65.0f;
    float maxX = 675.0f;
    float minY = 55.0f;
    float maxY = 655.0f;
    float shelfStartX = 210.0f;
    float shelfEndX = 525.0f;
    float shelfY = 215.0f;
    float screwX1 = 110.0f;
    float screwX2 = 620.0f;
    float screwY1 = 140.0f;
    float screwY2 = 220.0f;
    float screwY3 = 470.0f;
    float screwY4 = 560.0f;
    float gateY = 615.0f;
    float gateX1 = 115.0f;
    float gateX2 = 620.0f;


    // no shelf corners
    fieldBoundPoints = {
        cv::Point(screwX1, shelfY),
        cv::Point2f(screwX2, shelfY),
        cv::Point2f(screwX2, screwY3),
        cv::Point2f(maxX, screwY4),
        cv::Point2f(maxX, gateY),
        cv::Point2f(gateX2, maxY),
        cv::Point2f(gateX1, maxY),
        cv::Point2f(minX, gateY),
        cv::Point2f(minX, screwY4),
        cv::Point2f(screwX1, screwY3),
        cv::Point2f(screwX1, shelfY), // include first point again to complete last line
    };

    // lines are defined by connected adjacent points
    for(int i = 0; i < fieldBoundPoints.size() - 1; i++) {
        Line addLine = Line(fieldBoundPoints[i], fieldBoundPoints[i + 1]);
        fieldBoundLines.emplace_back(addLine);
    }


    // points that stick out
    convexPoints.emplace_back(cv::Point2f(screwX2, screwY3));
    convexPoints.emplace_back(cv::Point2f(screwX1, screwY3));


    // initialize filters
    orbFiltered = FilteredRobot(1.0f, 50.0f, 500.0f, 200.0f, 2.0f*360.0f*TO_RAD, 80.0f*360.0f*TO_RAD, 45.0f*TO_RAD, 40.0f*TO_RAD, 20.0f);
    oppFiltered = FilteredRobot(1.0f, 50.0f, 470.0f, 300.0f, 2.0f*360.0f*TO_RAD, 200.0f*360.0f*TO_RAD, 55.0f*TO_RAD, 40.0f*TO_RAD, 25.0f); // 25, 15, 25 last three
}





DriverStationMessage AStarAttack::Execute(Gamepad &gamepad)
{
    // track loop time
    static Clock updateClock;
    static ClockWidget processingTimeVisualizer("Orbit");
    processingTimeVisualizer.markStart();

    double deltaTime = updateClock.getElapsedTime(); // reset every loop so elapsed time is loop time
    if(deltaTime == 0) { deltaTime = 0.001; } // broski what
    updateClock.markStart(); // reset for next loop



    // get odometry data for orb and opp
    OdometryData orbData = RobotController::GetInstance().odometry.Robot();
    OdometryData oppData = RobotController::GetInstance().odometry.Opponent();

    // update filtered positions/velocities and paths
    orbFiltered.updateFilters(deltaTime, orbData.robotPosition, orbData.GetAngle()); orbFiltered.updatePath();
    oppFiltered.updateFilters(deltaTime, oppData.robotPosition, oppData.GetAngle()); 



    // extrapolate opp, extrapolate different amounts of time depending on if we attack forward or backward
    std::vector<cv::Point2f> oppSimPathForward = {};
    std::vector<cv::Point2f> oppSimPathBackward = {};

    FilteredRobot oppExtrapForward = orbFiltered.createVirtualOpp(oppFiltered, true, 0.25f, oppSimPathForward);
    FilteredRobot oppExtrapBackward = orbFiltered.createVirtualOpp(oppFiltered, false, 0.25f, oppSimPathBackward);




    // raw follow points in every direction
    // std::vector<bool> pointsCW = {true, true, false, false};
    // std::vector<bool> pointsForward = {true, false, true, false};
    std::vector<bool> pointsCW = {true, false};
    std::vector<bool> pointsForward = {true, true};
    // std::vector<bool> pointsCW = {false};
    // std::vector<bool> pointsForward = {true};


    // generate every possible follow point
    std::vector<FollowPoint> follows = {};
    for(int i = 0; i < pointsCW.size(); i++) {

        FilteredRobot targetOpp = pointsForward[i] ? oppExtrapForward : oppExtrapBackward;
        std::vector<cv::Point2f> oppSimPath = pointsForward[i] ? oppSimPathForward : oppSimPathBackward;

        follows.emplace_back(followPointDirection(targetOpp, deltaTime, pointsCW[i], pointsForward[i], oppSimPath));
    }



    // choose which way to circle the opp/drive
    bool forwardInput = (gamepad.GetRightStickY() >= 0.0f);
    FollowPoint follow = chooseBestPoint(follows, forwardInput, deltaTime);


    // commitToTarget(follow, deltaTime, 0.5f);
    enforceTurnDirection(follow); // force that we turn away from the opp if needed


    float angle = driveAngle(follow);
    angle = avoidBoundsVector(angle, follow);






    bool colliding = orbFiltered.colliding(oppFiltered, 10.0f);
    bool facing = orbFiltered.facing(oppFiltered, follow.forward);
    bool oppFacing = oppFiltered.facing(orbFiltered, true);

    bool hitOpp = colliding && facing && !oppFacing;
    bool hitOrb = colliding && oppFacing;



    // colors
    cv::Scalar colorOrb = follow.forward ? cv::Scalar(255, 200, 0) : cv::Scalar(0, 230, 255);
    cv::Scalar colorOrbLight = follow.forward ? cv::Scalar(255, 230, 200) : cv::Scalar(200, 230, 255);

    cv::Scalar colorOpp = cv::Scalar(0, 0, 255);
    cv::Scalar colorOppLight = cv::Scalar(200, 200, 255);



    // turn opp green if we hit them
    if(hitOpp) { 
        colorOpp = cv::Scalar(200, 255, 200); 
        colorOppLight = cv::Scalar(200, 255, 200); 
        DisplayUtils::emote(); // DO NOT DELETE
    }

    // turn orb red if we get hit
    if(hitOrb) {
        colorOrb = cv::Scalar(0, 50, 255);
        colorOrbLight = cv::Scalar(0, 50, 255);
    }


    // display circles
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.point, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.opp.position(), follow.radius, cv::Scalar(200, 200, 255), 2); // draw radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRad(), colorOrbLight, 2); // draw pp radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRadWall(), colorOrbLight, 1); // draw pp wall radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.opp.position(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.opp.position(), follow.opp.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.orbSimPath.back(), 10, cv::Scalar(255, 255, 255), 5);
    if(follow.enforceTurnDirection != 0) { safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.point, 10, cv::Scalar(255, 255, 255), 5); } // bold the follow point if we're enforcing a turn direction


    // display paths
    DisplayUtils::displayPath(follow.orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(follow.oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path
    DisplayUtils::displayLines(fieldBoundLines, cv::Scalar(0, 0, 255)); // draw field bound lines
    DisplayUtils::displayPoints(convexPoints, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255), 4); // draw field bound points
    DisplayUtils::displayPath(orbFiltered.getPath(), cv::Scalar(100, 100, 100), colorOrbLight, 3); // display orb path
    DisplayUtils::displayPath(follow.opp.getPath(), cv::Scalar(200, 200, 200), colorOppLight, 3); // display opp path


    // display lines
    cv::Point2f oppOrientationEnd = cv::Point2f(follow.opp.position().x + follow.opp.getSizeRadius()*cos(follow.opp.angle(true)), follow.opp.position().y + follow.opp.getSizeRadius()*sin(follow.opp.angle(true)));
    cv::line(RobotController::GetInstance().GetDrawingImage(), follow.opp.position(), oppOrientationEnd, colorOpp, 2);


    // display text
    std::string forwardStatus = follow.forward ? "Forward" : "Backward"; 
    std::string CWStatus = follow.CW ? "CW" : "CCW";

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Orbiting", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), CWStatus, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);




    // calculate drive inputs based on curvature controller,           0.8, 0.06
    std::vector<float> driveInputs = orbFiltered.curvatureController(angle, 0.6f*follow.controllerGain, 0.04f*follow.controllerGain, gamepad.GetRightStickY(), deltaTime, follow.enforceTurnDirection, follow.forward);




    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];


    std::cout << std::endl;
    
    
    processingTimeVisualizer.markEnd();
    return ret;
}






















// calculates raw follow point in a given direction
FollowPoint AStarAttack::followPointDirection(FilteredRobot opp, float deltaTime, bool CW, bool forward, std::vector<cv::Point2f> oppSimPath) {

    FollowPoint follow = FollowPoint(forward, CW, opp, oppSimPath); // create a follow point
    radiusEquation(follow); //  set attack radius
    

    follow.opp = oppFiltered; // extrapolation is only used for radius calc
    
    tangentPoint(follow); // main case is drive towards the tangent point on the attack radius


    bool outsideCircle = follow.radius < orbFiltered.distanceTo(follow.opp.position()); // if orb is outside of the attack radius

    // if we're outside the circle but the tangent point is too close then enforce the pure pursuit radius
    if(outsideCircle && orbFiltered.distanceTo(follow.point) < ppRad()) { follow.point = ppPoint(follow); }

    // if we're inside the circle then use this algorithm
    if(!outsideCircle) { followPointInsideCircle(follow); }
    
    return follow;
}



// returns angle to drive along by canceling opponents velocity
float AStarAttack::driveAngle(FollowPoint follow) {

    cv::Point2f oppVel = follow.opp.moveVel();
    float oppSpeed = cv::norm(oppVel); // how fast the opp is moving in absolute
    float maxCancel = orbFiltered.getMaxMoveSpeed() * 0.9f; // reserve some speed margin
    if(oppSpeed > maxCancel) { oppVel *= abs(maxCancel / oppSpeed); }

    float pointAngle = angle(cv::Point2f(orbFiltered.getPosFiltered()[0], orbFiltered.getPosFiltered()[1]), follow.point); // absolute angle to the point

    float p = oppVel.x*cos(pointAngle) + oppVel.y*sin(pointAngle);
    float s = sqrt(pow(orbFiltered.getMaxMoveSpeed(), 2) - pow(oppVel.x*sin(pointAngle) - oppVel.y*cos(pointAngle), 2));

    float pointSpeed = 0.0f;
    if(s > p) { pointSpeed = -p + s; } // thanks chat, ensures vector sum is always to maxMoveSpeed

    cv::Point2f pointVel = cv::Point2f(pointSpeed*cos(pointAngle), pointSpeed*sin(pointAngle)); // vector in the direction of the followPoint
    cv::Point2f totalVel = oppVel + pointVel; // sum the opp's vel with our desired vel in their frame

    return atan2(totalVel.y, totalVel.x); // target angle is in the direction of the total vector
}



// if we ever directly target the opp, enforce that we keep targetting them for a minimum time
void AStarAttack::commitToTarget(FollowPoint &follow, double deltaTime, float targetTime) {

    static bool enforceTarget = false; // if we're currently running the clock to force target the opp
    static float timeTargetting = 0.0f; // how long we've been force targetting the opp

    // if the follow point is basically right on the opp, turn on force targeting and reset the clock
    if(follow.opp.distanceTo(follow.point) < follow.opp.getSizeRadius() * 0.2f) { 
        enforceTarget = true;
        timeTargetting = 0.0f;
    }

    // if we've been targetting for long enough, don't enforce the target anymore and reset clock
    if(timeTargetting > targetTime) { 
        enforceTarget = false;
        timeTargetting = 0.0f;
    }

    // if we're currently enforcing target, then set followPoint and increment the clock
    if(enforceTarget) { 
        follow.point = follow.opp.position(); 
        timeTargetting += deltaTime;
    }

    // std::cout << "timeTargetting = " << timeTargetting << "     " << std::endl;
}




void AdjustRadiusWithBumpers() {
    static bool prevRightBumper = false;
    static bool prevLeftBumper = false;
    bool currRightBumper = RobotController::GetInstance().gamepad.GetRightBumper();
    bool currLeftBumper = RobotController::GetInstance().gamepad.GetLeftBumper();
    if (currRightBumper && !prevRightBumper) {
        RADIUS_CURVE_Y1 += 5;
    }
    if (currLeftBumper && !prevLeftBumper) {
        RADIUS_CURVE_Y1 -= 5;
    }

    RADIUS_CURVE_Y1 = fmin(RADIUS_CURVE_Y1, RADIUS_CURVE_Y2 - 10);
    RADIUS_CURVE_Y1 = fmax(RADIUS_CURVE_Y1, 0);

    prevRightBumper = currRightBumper;
    prevLeftBumper = currLeftBumper;
}



// equation for determining size of tangent circle
void AStarAttack::radiusEquation(FollowPoint &follow) {

    // calculate times for orb and opp
    float orbETA = orbFiltered.ETASim(follow.opp, follow.orbSimPath, true, true, follow.forward);

    float oppETA = follow.opp.turnTimeSimple(follow.orbSimPath.back(), follow.opp.getWeaponAngleReach(), true, true);
    // float oppETA = oppFiltered.turnTimeSimple(orbFiltered.position(), oppFiltered.getWeaponAngleReach(), true, true);

    // fraction is the main metric for radius
    float fraction = 999999999.0f;
    if (oppETA != 0.0f) { fraction = std::max((orbETA - oppETA) / oppETA, 0.0f); }


    AdjustRadiusWithBumpers(); // let driver adjust aggressiveness with bumpers


    // input fraction to output radius using RobotConfig variables
    std::vector<cv::Point2f> radiusCurve = {
        cv::Point2f(RADIUS_CURVE_X0, RADIUS_CURVE_Y0),
        cv::Point2f(RADIUS_CURVE_X1, RADIUS_CURVE_Y1),
        cv::Point2f(RADIUS_CURVE_X2, RADIUS_CURVE_Y2)
    };

    // radius is piecewise output
    // follow.radius = piecewise(radiusCurve, fraction);
    // follow.radius = 120.0f;
    // follow.radius = std::max(120.0f * (-pow(1.15f, -fraction) + 1.0f), 0.0f);
    // follow.radius = 130.0f * sin(std::min(0.16 * fraction, 90.0f*TO_RAD)); // 0.125 higher is more conservative
    follow.radius = 130.0f * sin(std::min(0.16 * fraction, 90.0f*TO_RAD)); // 0.125 higher is more conservative
}


// evalutates a piecewise function defined by the point array
float AStarAttack::piecewise(std::vector<cv::Point2f> points, float x) {

    // Clamp below
    if (x <= points.front().x) { return points.front().y; }

    // Clamp above
    if (x >= points.back().x) { return points.back().y; }

    // Interpolate between points
    for (int i = 0; i < points.size() - 1; i++) {
        cv::Point2f p0 = points[i];
        cv::Point2f p1 = points[i + 1];

        if (x >= p0.x && x < p1.x) {
            float percent = (x - p0.x) / (p1.x - p0.x);
            return p0.y + percent * (p1.y - p0.y);
        }
    }

    return 0.0f; // fallback
}


// returns the tangent point of a circle with set radius around the opponent
void AStarAttack::tangentPoint(FollowPoint &follow) {

    // circle is at the desired radius
    std::vector<cv::Point2f> circle = arcPointsFromCenter(follow.radius, 2*M_PI, 5.0f);
    transformList(circle, follow.opp.position(), 0.0f);

    // find the tangent point
    int tangentIndex = 0;
    
    // check first index to set initial values
    float mostTangentAngle = angle(orbFiltered.position(), circle[0]);

    for(int i = 1; i < circle.size(); i++) {
        float currentAngle = angle(orbFiltered.position(), circle[i]);
        float angleDiff = angleWrapRad(currentAngle - mostTangentAngle);
        if((angleDiff < 0 && follow.CW) || (angleDiff > 0 && !follow.CW)) {
            mostTangentAngle = currentAngle;
            tangentIndex = i;
        }
    }

    follow.point = circle[tangentIndex];
}


// returns intersection of pp radius with circle of set radius around opponent in the correction direction
cv::Point2f AStarAttack::ppPoint(FollowPoint follow) {

    std::vector<cv::Point2f> circle = arcPointsFromCenter(follow.radius, 2*M_PI, 5.0f);
    transformList(circle, follow.opp.position(), 0.0f);

    // find the greatest or least index point in the pp radius, thats the one to follow
    int defaultIndex = -1;
    int followIndex = defaultIndex;

    for(int i = 0; i < circle.size(); i++) {
        if(cv::norm(circle[i] - orbFiltered.position()) < ppRad()) {

            int indexDistance = i - followIndex;
            if(indexDistance > circle.size()/2.0f) { indexDistance -= circle.size(); }

            if((indexDistance > 0 && follow.CW) || (indexDistance < 0 && !follow.CW) || followIndex == defaultIndex) {
                followIndex = i;
            }
        }
    }

    if(followIndex == defaultIndex) { followIndex = 0; } // no crashy
    return circle[followIndex];
}



// does a test line intersect any boundary line
bool AStarAttack::intersectsAnyBound(Line testLine) {
    for(int i = 0; i < fieldBoundLines.size(); i++) {
        float howCloseFirstPoint = fieldBoundLines[i].howClosePoint(testLine.getLinePoints().first);
        float howCloseSecondPoint = fieldBoundLines[i].howClosePoint(testLine.getLinePoints().second);
        if(fieldBoundLines[i].doesIntersectLine(testLine) || howCloseFirstPoint < 0.1f || howCloseSecondPoint < 0.1f) { // add tolerance in case point is exactly on line (clipped points often are)
            return true;
        }
    }
    return false;
}



// generates a list of arc points from the center
std::vector<cv::Point2f> AStarAttack::arcPointsFromCenter(float radius, float angle, float pointSpacing) {

    float arcLength = radius * angle;
    float thetaIncrement = pointSpacing / radius;
    int increments = (int) (angle / thetaIncrement);

    std::vector<cv::Point2f> points;

    for (int i = 0; i < increments; i++) {
        float currentAngle = i * thetaIncrement;
        float currentX = radius*cos(currentAngle);
        float currentY = radius*sin(currentAngle);
        points.emplace_back(cv::Point2f(currentX, currentY));
    }

    // make sure the exact final point is added
    points.emplace_back(cv::Point2f(radius*cos(angle), radius*sin(angle)));
    return points;
}


// moves a list of points to the input point and angle, rewrites to the existing list
void AStarAttack::transformList(std::vector<cv::Point2f>& list, cv::Point2f startPoint, float angle) {

    for(int i = 0; i < list.size(); i++) {

        cv::Point2f currentPoint = list[i];
        float newX = startPoint.x + currentPoint.x*cos(angle) - currentPoint.y*sin(angle);
        float newY = startPoint.y + currentPoint.x*sin(angle) + currentPoint.y*cos(angle);

        list[i] = cv::Point2f(newX, newY);
    }
}



// absolute angle made by 2 points
float AStarAttack::angle(cv::Point2f point1, cv::Point2f point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}



// distance to the closest line from the list
std::pair<float, int> AStarAttack::closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point) {

    float closest = 999999999.9f; // will always be further away than anything else
    int index = -1;

    // check the whole list and record closest distance
    for (int i = 0; i < lineList.size(); i++) {
        float distance = lineList[i].howClosePoint(point);
        if (distance < closest) { 
            closest = distance; 
            index = i;
        }
    }
    return std::pair<float, int>(closest, index);
}


// finds the closest point that's on a boundary line
cv::Point2f AStarAttack::closestBoundPoint(cv::Point2f point) {

    std::pair<float, int> closestFieldBound = closestFromLineList(fieldBoundLines, point);
    std::pair<cv::Point2f, cv::Point2f> closestLinePoints = fieldBoundLines[closestFieldBound.second].getLinePoints();

    // highlight closest line
    cv::line(RobotController::GetInstance().GetDrawingImage(), closestLinePoints.first, closestLinePoints.second, cv::Scalar(220, 220, 255), 2);
    
    cv::Point2f closestWallPoint = fieldBoundLines[closestFieldBound.second].closestLinePoint(point);
    return closestWallPoint;
}


// if the point is inside the shape of the field
bool AStarAttack::insideFieldBounds(cv::Point2f point) {

    // define horizontal line from point
    Line testLine(point, cv::Point2f(point.x + 999999.0f, point.y));

    // count the number of intersections with the bounds
    int intersections = 0;
    for(int i = 0; i < fieldBoundLines.size(); i++) {
        if(testLine.doesIntersectLine(fieldBoundLines[i])) { intersections++; }
    }

    // point is in the field if there's an odd number of intersections
    return intersections % 2 != 0;
}


// does a point list include this point, returns -1 if not found
int AStarAttack::vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint) {

    for(int i = 0; i < pointList.size(); i++) {
        if(pointList[i] == testPoint) { return i; }
    }
    return -1;
}


// clips a point to be fully in bounds if needed
cv::Point2f AStarAttack::clipPointInBounds(cv::Point2f testPoint) {

    cv::Point2f returnPoint = testPoint; 
    if(!insideFieldBounds(returnPoint)) { 
        cv::Point2f onBoundStart = closestBoundPoint(testPoint);

        // 8 surrounding points
        double increment = 0.1f;
        std::vector<cv::Point2f> increments = {
            cv::Point2f(increment, increment),   cv::Point2f(increment, 0),
            cv::Point2f(increment, -increment),  cv::Point2f(0, -increment),
            cv::Point2f(-increment, -increment), cv::Point2f(-increment, 0.0),
            cv::Point2f(-increment, increment),  cv::Point2f(0.0, increment)};
        for (int i = 0; i < increments.size(); i++) {
          cv::Point2f inBound = cv::Point2f(onBoundStart.x + increments[i].x,
                                            onBoundStart.y + increments[i].y);
          if (insideFieldBounds(inBound)) {
            returnPoint = inBound;
          }
        }
    }
    return returnPoint;
}




// spirals out from our current radius from opp to see how walls will affect us
float AStarAttack::wallScore(FollowPoint follow) {

    const float maxPathLength = 500.0f; // length of predicted path we'll sweep

    std::vector<cv::Point2f> scanPoints;
    std::vector<Line> scanLines = {};
    const float sweepIncrement = follow.CW ? 1.0f*TO_RAD : -1.0f*TO_RAD; // how much to increment sweep angle for each point

    float startAngle = angle(follow.opp.position(), orbFiltered.position());


    float score = 0.0f; // integrated score of all the points
    float pathLength = 0.0f; // accumlated predicted path length

    // scan up to a full circle
    for (float sweptAngle = 0; abs(sweptAngle) < 2*M_PI; sweptAngle += sweepIncrement) {

        float currAngle = angleWrapRad(startAngle + sweptAngle); // current angle we're scanning at

        cv::Point2f testPoint = clipPointInBounds(follow.opp.position()); // point to ray cast
        constexpr int kMaxRaycastSteps = 400;


        // increment the point outwards until it's out of bounds or further than our spiral
        for(int i = 0; i < kMaxRaycastSteps; i++) {

            testPoint += 5.0f * cv::Point2f(cos(currAngle), sin(currAngle));

            // if the ray cast has exited the field
            if(!insideFieldBounds(testPoint)) {
                testPoint = closestBoundPoint(testPoint); // raycasted point might lie slightly outside, so make sure to re-clip after

                float gapSize = std::max(follow.opp.distanceTo(testPoint) - follow.opp.getSizeRadius(), 0.0f); // how big is the gap to drive in
                float margin = std::max(gapSize - 2*orbFiltered.getSizeRadius(), 0.01f); // how much margin would we have if we had to drive past this point
                float sweptPercent = pathLength / maxPathLength; // roughly what percentage has been swept so far

                float rangeWeight = cos(-sweptPercent * 0.5f * M_PI);

                score += pow(margin, -0.6f) + rangeWeight; // score function is integrated based on how much gap we have

                break;
            }

            float spiralRadius = abs(sweptAngle)*110.0f + follow.opp.distanceTo(orbFiltered.position()); // slope of predicted spiral
            
            // if the point has gotten to the spiral, don't add to score
            if(follow.opp.distanceTo(testPoint) > spiralRadius) { break; }
        }

        // total up the path length
        if(!scanPoints.empty()) {
            pathLength += cv::norm(scanPoints.back() - testPoint);
        }
        
        
        scanPoints.emplace_back(testPoint); // add to the list to display later
        scanLines.emplace_back(Line(follow.opp.position(), testPoint));

        // if the points are so far away now then stop counting
        if(pathLength > maxPathLength) { break; }

    }

    score /= pathLength; // normalize to sweep range so we can change that without losing tune

    cv::Scalar color = cv::Scalar(150, 255, 150, 255);
    if(!follow.CW) { color = cv::Scalar(150, 150, 255, 255); }


    DisplayUtils::displayPoints(scanPoints, color, color, 2); // display scan points on walls


    std::cout << "    score CW " << follow.CW << " = " << score << "     ";

    return score;
}




// weighs how bad the walls are based on pinch points
float AStarAttack::wallScorePinch(FollowPoint follow) {

    // angular range to scan
    const float sweepEnd = 179.0f*TO_RAD;
    const float sweepStart = 40.0f*TO_RAD;

    std::vector<cv::Point2f> scanPoints = {}; // all points that scan along the wall
    std::vector<cv::Point2f> pinchPoints = {}; // points that are a local min distance
    std::vector<Line> scanLines = {}; // lines that connect to the scan points from the opp

    float startAngle = angle(follow.opp.position(), orbFiltered.position()); // start scanning along the angle connecting us

    // sweep through the whole angle around by incrementing
    float previousDistance = 0.0f;
    bool previousDecreasing = false; 
    float highestScore = 0.0f;

    const float sweepIncrement = follow.CW ? 1.0f*TO_RAD : -1.0f*TO_RAD; // how much to increment sweep angle for each point
    // scan the specified angle range
    for (float angleOffset = follow.CW ? sweepStart : -sweepStart; abs(angleOffset) < abs(sweepEnd);
         angleOffset += follow.CW ? 1.0f*TO_RAD : -1.0f*TO_RAD)
    {
        float currAngle = angleWrapRad(startAngle + angleOffset);
        cv::Point2f testPoint = clipPointInBounds(follow.opp.position());

        constexpr int kMaxRaycastSteps = 400;

        // increment the point outwards until it's out of bounds
        for(int i = 0; i < kMaxRaycastSteps; i++) {

            testPoint += 5.0f * cv::Point2f(cos(currAngle), sin(currAngle));

            if(!insideFieldBounds(testPoint)) { break; }
        }
        
        testPoint = closestBoundPoint(testPoint); // raycasted point might lie slightly outside, so make sure to re-clip after.
        scanPoints.emplace_back(testPoint);
        scanLines.emplace_back(Line(follow.opp.position(), testPoint));

        float pointDistance = std::max(follow.opp.distanceTo(testPoint) - follow.opp.getSizeRadius(), 0.01f);
        
        // if the points are getting further away
        if(pointDistance > previousDistance) { 
            // if it was previously decreasing then it's a pinch point
            if(previousDecreasing) {

                // track the worst case pinch point
                float score = (sweepEnd - abs(angleOffset)) / pointDistance;
                if(score > highestScore) { highestScore = score; }

                pinchPoints.emplace_back(testPoint); 
            }

            previousDecreasing = false; // mark it wasn't decreasing last time
        }
        else {
            previousDecreasing = true; 
        }

        previousDistance = pointDistance;
    }

    cv::Scalar color = cv::Scalar(150, 255, 150, 255);
    if(!follow.CW) { color = cv::Scalar(150, 150, 255, 255); }


    DisplayUtils::displayPoints(scanPoints, color, color, 2);
    DisplayUtils::displayPoints(pinchPoints, color, color, 10);

    // displayLineList(scanLines, color);


    std::cout << "    score CW " << follow.CW << " = " << highestScore << "     ";

    return highestScore;

}




// returns sign of the input
int AStarAttack::sign(float num) {
    if(num < 0) { return -1; }
    return 1;
}


// enforces (or doesn't) a turn direction so we turn away from the op when desired
// 1 = always turn right, -1 = always turn left
void AStarAttack::enforceTurnDirection(FollowPoint &follow) {

    float angleToOpp = orbFiltered.angleTo(follow.opp.position(), follow.forward);
    float angleToPoint = orbFiltered.angleTo(follow.point, follow.forward);

    // if we naturally turn away from the opp, leave it as 0
    if(sign(angleToOpp) != sign(angleToPoint)) { return; }

    // if we turn by the opp in the first few degrees anyway just turn normally
    if(abs(angleToOpp) < 15.0f*TO_RAD) { return; } // 20.0f

    // if we don't have to turn past the opp anyway (or just barely turn past) just turn normally
    if(abs(angleToPoint) < abs(angleToOpp) + 30.0f*TO_RAD) { return; } // 40


    // otherwise enforce a turn direction opposite from what you'd normally do
    follow.enforceTurnDirection = -sign(angleToPoint);
}


// calculates pure pursuit radius based on speeds
float AStarAttack::ppRad() {
    float radSlow = ASTAR_PP_RAD_SLOW;
    float radFast = ASTAR_PP_RAD_FAST;
    float speedFast = ASTAR_PP_SPEED_FAST;
    return radSlow + ((radFast - radSlow) / speedFast) * orbFiltered.moveSpeedSlow();
}


// pp radius used for walls
float AStarAttack::ppRadWall() {
    float radSlow = 40.0f;
    float radFast = 110.0f;
    float speedFast = 400.0f;
    return radSlow + ((radFast - radSlow) / speedFast) * orbFiltered.moveSpeedSlow();
}



// adjusts drive angle if needed to obey pure pursuit radius on walls
float AStarAttack::avoidBoundsVector(float driveAngle, FollowPoint &follow) {

    std::vector<cv::Point2f> ppWallPoints = PurePursuit::followPath(orbFiltered.position(), fieldBoundPoints, ppRadWall());

    // if we're not hitting a wall and in the field don't change the drive angle
    if(ppWallPoints.size() == 0 && insideFieldBounds(orbFiltered.position())) { return driveAngle; }

    cv::Point2f boundPoint = closestBoundPoint(orbFiltered.position()); // get the closest point to us thats on a field bound

    // if we're not hitting a wall and fully out of the field then drive back in to closest bound point
    if(ppWallPoints.size() == 0 && !insideFieldBounds(orbFiltered.position())) { 
        safe_circle(RobotController::GetInstance().GetDrawingImage(), boundPoint, 8, cv::Scalar(255, 255, 255), 3);
        follow.enforceTurnDirection = 0; // don't worry about turning away from opp, need to return optimally
        return angle(orbFiltered.position(), boundPoint);
    }

    // from here we're guarenteed to have pure pursuit intersections with the wall







    // now let's find which pp points actually define the region

    float angleToBound = angle(orbFiltered.position(), closestBoundPoint(orbFiltered.position())); // angle to the closest bound point

    // min and max offsets from the line to the bound point
    float mostPosOffset = 0;
    float mostNegOffset = 0;

    // find the most extreme offsets, those will be the outer ones used for pure pursuit
    for(int i = 0; i < ppWallPoints.size(); i++) {
        float pointAngle = angle(orbFiltered.position(), ppWallPoints[i]);
        float offset = angleWrapRad(pointAngle - angleToBound); // offset from the line to the center of the field

        if(offset > mostPosOffset) { mostPosOffset = offset; }
        if(offset < mostNegOffset) { mostNegOffset = offset; }
    }

    // angle region in which drive angle must lie
    float minAngle = angleWrapRad(angleToBound + mostNegOffset);
    float maxAngle = angleWrapRad(angleToBound + mostPosOffset);


    // points at the min and max angles
    cv::Point2f minPoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(minAngle), sin(minAngle));
    cv::Point2f maxPoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(maxAngle), sin(maxAngle));

    safe_circle(RobotController::GetInstance().GetDrawingImage(), minPoint, 8, cv::Scalar(255, 255, 255), 3);
    safe_circle(RobotController::GetInstance().GetDrawingImage(), maxPoint, 8, cv::Scalar(255, 255, 255), 3);









    // now change the drive angle if needed

    // intersection of pp radius and drive angle
    cv::Point2f drivePoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(driveAngle), sin(driveAngle));

    // if drive point isn't in the field we need to change it
    if(!insideFieldBounds(drivePoint)) {

        // set it to the closest bound of the region
        float minAngleOffset = angleWrapRad(driveAngle - minAngle);
        float maxAngleOffset = angleWrapRad(driveAngle - maxAngle);

        driveAngle = maxAngle; // default

        if(abs(minAngleOffset) < abs(maxAngleOffset)) { 
            // safe_circle(RobotController::GetInstance().GetDrawingImage(), minPoint, 8, cv::Scalar(180, 180, 255), 3);
            // std::cout << "using minPoint = " << minAngle*TO_DEG << std::endl;
            driveAngle = minAngle;
        }
        // safe_circle(RobotController::GetInstance().GetDrawingImage(), maxPoint, 8, cv::Scalar(180, 180, 255), 3);
        // std::cout << "using maxPoint = " << maxAngle*TO_DEG << std::endl;
        // return maxAngle;
    }









    // make sure we turn the right direction to the drive angle

    float orbOffset = angleWrapRad(angleToBound - orbFiltered.angle(angleToBound));
    float driveOffset = angleWrapRad(angleToBound - driveAngle);

    // only change the direction enforcement if turning the wrong way would make us go way out of the field
    if(abs(orbOffset) > 8.0f*TO_RAD) { 

        // stop enforcing direction bc that might make it go out
        follow.enforceTurnDirection = 0;

        if(orbOffset > 0 && driveOffset < 0) { follow.enforceTurnDirection = -1; }
        if(orbOffset < 0 && driveOffset > 0) { follow.enforceTurnDirection = 1; }

        if(!insideFieldBounds(orbFiltered.position())) { follow.enforceTurnDirection *= -1; } // needs to invert if out of the field
    }

    std::cout << "enforcing " << follow.enforceTurnDirection << std::endl;


    return driveAngle;
}



// score function for determining how good a follow point is, used for CW/CCW decision
float AStarAttack::directionScore(FollowPoint follow, float deltaTime, bool forwardInput) {
    static GraphWidget angleToPointGraph("wallWeight*wallGain", -100, 100, "deg");

    // rough scale to decide when we're reasonably far from the opponent that turning delays and stuff aren't a risk
    float farAway = 500.0f;
    float collisionRadius = orbFiltered.getSizeRadius() + follow.opp.getSizeRadius();
    float closeness = std::clamp(1.0f - ((orbFiltered.distanceTo(follow.opp.position()) - collisionRadius) / farAway), 0.0f, 1.0f);


    // how far the robot has to turn to this point
    float angleToPoint = abs(orbFiltered.angleTo(follow.point, follow.forward));    
    float turnGain = 35.0f; // 30


    // how far around the circle we have to go for each direction
    float directionSign = 1.0f; if(!follow.CW) { directionSign = -1.0f; }
    float goAroundAngle = M_PI - directionSign*follow.opp.angleTo(orbFiltered.position(), true);
    float goAroundGain = 15.0f + 0.4f * follow.opp.tangentVel(true); // 12



    // how close is the nearest wall in this direction
    float wallGain = 260.0f;
       

    // how much velocity we already have built up in a given direction, ensures we don't switch to other direction randomly
    float raw = orbFiltered.tangentVel(follow.forward);
    float tanVel = pow(raw, 2.0f) * sign(raw);
    float momentumWeight = -0.01f * closeness; // 0.2


    // penalty for using the back bc we want to use front more often
    int directionInput = 1; if(!forwardInput) { directionInput = -1; }
    float backWeight = 200.0f*directionInput; // 40.0f

    // angleToPointGraph.AddData(wallWeight*wallGain);

    
    // sum up components for score
    return follow.radius + goAroundAngle*goAroundGain + wallScore(follow)*wallGain + angleToPoint*turnGain + tanVel*momentumWeight + backWeight*!follow.forward;
}




// determines follow point when we're inside the main attack radius
void AStarAttack::followPointInsideCircle(FollowPoint &follow) {

    float direction = 1.0f;
    if(follow.CW) { direction = -1.0f; }

    float angleToOrb = follow.opp.angleTo(orbFiltered.position(), true); 
    float distanceToOpp = orbFiltered.distanceTo(follow.opp.position());
    float collisionRadius = orbFiltered.getSizeRadius() + follow.opp.getSizeRadius();
    

    // first define parameters at collision radius
    float minAngleFront = -30.0f*TO_RAD; // -80
    float maxAngleFront = -0.0f*TO_RAD; // 0

    float minAngleSide = -90.0f*TO_RAD; // -90
    float maxAngleSide = -0.0f*TO_RAD; // -0

    float sideAngle = -110.0f*TO_RAD;



    // linearly interpolate min and max angles at collision radius based on the current angle to orb
    float anglePercent = std::clamp((angleToOrb * direction) / sideAngle, 0.0f, 1.0f);
    float minAngleClose = anglePercent * (minAngleSide - minAngleFront) + minAngleFront;
    float maxAngleClose = anglePercent * (maxAngleSide - maxAngleFront) + maxAngleFront;



    
    // both angles are linearly interpolated to -90 degrees at the outer radius
    float angleAtRadius = -90.0f*TO_RAD;
    float radiusPercent = std::max((distanceToOpp - collisionRadius), 0.0f) / std::max((follow.radius - collisionRadius), 0.01f);
    float minAngle = radiusPercent * (angleAtRadius - minAngleClose) + minAngleClose;
    minAngle = minAngleClose;
    float maxAngle = radiusPercent * (angleAtRadius - maxAngleClose) + maxAngleClose;


    // controller gain is less and less the further you get into the circle
    float minGain = 0.6f; // 0.3
    follow.controllerGain = radiusPercent*(1.0f - minGain) + minGain;


    // convert to world angle
    float minWorldAngle = angleWrapRad(minAngle*direction + angleToOrb + follow.opp.angle(true));
    float maxWorldAngle = angleWrapRad(maxAngle*direction + angleToOrb + follow.opp.angle(true));



    // if the pure pursuit point might be usable, check
    if(distanceToOpp > follow.radius - ppRad()) {
        cv::Point2f ppFollow = ppPoint(follow);
        float angleToPP = angle(orbFiltered.position(), ppFollow);
        float maxAngleDifference = angleWrapRad(maxWorldAngle - angleToPP);
        float minAngleDifference = angleWrapRad(minWorldAngle - angleToPP);


        // if(CW) { safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 5, cv::Scalar(255, 255, 255), 5); }


        
        if(follow.CW && maxAngleDifference < 0) { maxWorldAngle = angleToPP; }
        if(follow.CW && minAngleDifference < 0) { minWorldAngle = angleToPP; }
        if(!follow.CW && maxAngleDifference > 0) { maxWorldAngle = angleToPP; }
        if(!follow.CW && minAngleDifference > 0) { minWorldAngle = angleToPP; }
    }




    // angle to follow
    float angleRange = angleWrapRad(maxWorldAngle - minWorldAngle) * direction;
    float midAngle = minWorldAngle + (angleRange / 2) * direction;
    float angleError = angleWrapRad(orbFiltered.angle(follow.forward) - midAngle) * direction;


    // default: assume we're in the angle range so just go forward
    float followAngle = orbFiltered.angle(follow.forward);
    if(angleError < -angleRange/2) { followAngle = minWorldAngle; }
    if(angleError > angleRange/2) { followAngle = maxWorldAngle; }


    // if(forward && CW) {
    //     cv::Point2f minAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(minWorldAngle), orbFiltered.position().y + ppRadius*sin(minWorldAngle));
    //     cv::Point2f maxAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(maxWorldAngle), orbFiltered.position().y + ppRadius*sin(maxWorldAngle));
    //     cv::Point2f midAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(midAngle), orbFiltered.position().y + ppRadius*sin(midAngle));
    //     cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), minAnglePoint, cv::Scalar(255, 50, 50), 2);
    //     cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), maxAnglePoint, cv::Scalar(255, 50, 50), 2);
    //     cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), midAnglePoint, cv::Scalar(255, 150, 150), 2);

    //     // std::cout << "     minAngle = " << minAngle*TO_DEG << ", maxAngle = " << maxAngle*TO_DEG << "      ";
    // }
    

    // follow point is pp radius away at the calculated angle
    follow.point = cv::Point2f(orbFiltered.position().x + ppRad()*cos(followAngle), orbFiltered.position().y + ppRad()*sin(followAngle));
}


// returns the best follow point with the best score, also sets CW and forward by reference
FollowPoint AStarAttack::chooseBestPoint(std::vector<FollowPoint> follows, bool forwardInput, float deltaTime) {

    if(follows.size() == 0) { return FollowPoint(); } // no crashy

    // find follow point with best score
    int bestIndex = -1;
    float bestScore = 0.0f;

    for(int i = 0; i < follows.size(); i++) {

        float newScore = directionScore(follows[i], deltaTime, forwardInput);
        if(newScore < bestScore || i == 0) {
            bestScore = newScore;
            bestIndex = i;
        }
    }

    // return best index
    return follows[bestIndex];
}



// Field boundary editing interface implementation
AStarAttack* AStarAttack::GetInstance() {
    return _instance;
}

std::vector<cv::Point2f>& AStarAttack::GetFieldBoundaryPoints() {
    return fieldBoundPoints;
}

void AStarAttack::SetFieldBoundaryPoints(const std::vector<cv::Point2f>& points) {
    fieldBoundPoints = points;
    RegenerateFieldBoundaryLines();
}

// Radius curve parameter interface implementation
void AStarAttack::GetRadiusCurvePoints(float radiusCurveX[3], float radiusCurveY[3]) {
    radiusCurveX[0] = RADIUS_CURVE_X0;
    radiusCurveX[1] = RADIUS_CURVE_X1;
    radiusCurveX[2] = RADIUS_CURVE_X2;
    radiusCurveY[0] = RADIUS_CURVE_Y0;
    radiusCurveY[1] = RADIUS_CURVE_Y1;
    radiusCurveY[2] = RADIUS_CURVE_Y2;
}

void AStarAttack::SetRadiusCurvePoints(const float radiusCurveX[3], const float radiusCurveY[3]) {
    RADIUS_CURVE_X0 = radiusCurveX[0];
    RADIUS_CURVE_X1 = radiusCurveX[1];
    RADIUS_CURVE_X2 = radiusCurveX[2];
    RADIUS_CURVE_Y0 = radiusCurveY[0];
    RADIUS_CURVE_Y1 = radiusCurveY[1];
    RADIUS_CURVE_Y2 = radiusCurveY[2];
}

void AStarAttack::ResetRadiusCurveToDefault() {
    RADIUS_CURVE_X0 = 0.0f;   RADIUS_CURVE_Y0 = 0.0f;
    RADIUS_CURVE_X1 = 15.0f;  RADIUS_CURVE_Y1 = 85.0f;
    RADIUS_CURVE_X2 = 100.0f; RADIUS_CURVE_Y2 = 150.0f;
}

void AStarAttack::RegenerateFieldBoundaryLines() {
    fieldBoundLines.clear();
    
    // Regenerate lines from connected adjacent points
    for(int i = 0; i < fieldBoundPoints.size() - 1; i++) {
        Line addLine = Line(fieldBoundPoints[i], fieldBoundPoints[i + 1]);
        fieldBoundLines.emplace_back(addLine);
    }
    
    // Update convex points (points that stick out)
    convexPoints.clear();
    if (fieldBoundPoints.size() >= 11) {
        // Assuming the same structure as the original field bounds
        convexPoints.emplace_back(fieldBoundPoints[2]); // screwX2, screwY3
        convexPoints.emplace_back(fieldBoundPoints[9]); // screwX1, screwY3
    }
}

void AStarAttack::ResetFieldBoundariesToDefault() {
    // Restore default field boundaries
    float minX = 65.0f;
    float maxX = 675.0f;
    float minY = 55.0f;
    float maxY = 655.0f;
    float shelfStartX = 210.0f;
    float shelfEndX = 525.0f;
    float shelfY = 215.0f;
    float screwX1 = 110.0f;
    float screwX2 = 620.0f;
    float screwY1 = 140.0f;
    float screwY2 = 220.0f;
    float screwY3 = 470.0f;
    float screwY4 = 560.0f;
    float gateY = 615.0f;
    float gateX1 = 115.0f;
    float gateX2 = 620.0f;

    fieldBoundPoints = {
        cv::Point(screwX1, shelfY),
        cv::Point2f(screwX2, shelfY),
        cv::Point2f(screwX2, screwY3),
        cv::Point2f(maxX, screwY4),
        cv::Point2f(maxX, gateY),
        cv::Point2f(gateX2, maxY),
        cv::Point2f(gateX1, maxY),
        cv::Point2f(minX, gateY),
        cv::Point2f(minX, screwY4),
        cv::Point2f(screwX1, screwY3),
        cv::Point2f(screwX1, shelfY), // include first point again to complete last line
    };
    
    RegenerateFieldBoundaryLines();
}
