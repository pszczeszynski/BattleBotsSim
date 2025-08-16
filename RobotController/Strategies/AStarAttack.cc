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


    CW = true; // default to clockwise
    currForward = true; // default to leading with bar
    prevAngleError = 0.0f; // initialize


    // initialize filters
    orbFiltered = FilteredRobot(1.0f, 100.0f, 430.0f, 200.0f, 2.0f*360.0f*TO_RAD, 80.0f*360.0f*TO_RAD, 45.0f*TO_RAD, 40.0f*TO_RAD, 20.0f);
    oppFiltered = FilteredRobot(1.0f, 100.0f, 400.0f, 300.0f, 2.0f*360.0f*TO_RAD, 200.0f*360.0f*TO_RAD, 50.0f*TO_RAD, 40.0f*TO_RAD, 25.0f); // calibrated at roughly 2.5 and 80
    
    // Note: Radius curve parameters are initialized from RobotConfig defaults
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


    // raw follow points in every direction
    std::vector<bool> pointsCW = {true, true, false, false};
    std::vector<bool> pointsForward = {true, false, true, false};
    // std::vector<bool> pointsCW = {true, false};
    // std::vector<bool> pointsForward = {false, false};

    // generate every possible follow point
    std::vector<cv::Point2f> followPoints = {};
    for(int i = 0; i < pointsCW.size(); i++) {
        followPoints.emplace_back(followPointDirection(pointsCW[i], pointsForward[i]));
    }


    // choose which way to circle the opp/drive
    cv::Point2f followPoint = chooseBestPoint(followPoints, pointsCW, pointsForward, CW, currForward);
    followPoint = avoidBounds(followPoint); // make sure we don't hit any walls
    int turnDirection = enforceTurnDirection(followPoint, currForward); // force that we turn away from the opp if needed
    followPoint = commitToTarget(followPoint, deltaTime, 1.0f);






    // std::cout << "CW = " << CW << ", forward = " << currForward;
    // std::cout << "follow point x = " << followPoint.x << std::endl;


#ifdef DEBUG_DISPLAY
    float orbETA = orbFiltered.collideETA(oppFiltered, currForward, 65.0f);
    float angleMargin = 40.0f * TO_RAD;
    float humanLag = 0.05f; // 0.05
    float oppETA = oppFiltered.turnTimeMin(predictDriftStop(currForward), humanLag, angleMargin, currForward, true);

    std::cout << "    orbETA = " << orbETA << ", oppETA = " << oppETA << "       ";
    std::cout << std::endl;
#endif

    
   

    // display things we want
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), radiusEquation(currForward, CW), cv::Scalar(0, 255, 0), 2); // draw radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRad(), cv::Scalar(255, 0, 0), 2); // draw pp radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, cv::Scalar(0, 0, 255), 2); // draw dot on the opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppFiltered.getSizeRadius(), cv::Scalar(190, 190, 255), 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), cv::Scalar(255, 190, 190), 2); // draw op size

    displayPathTangency(oppFiltered, cv::Scalar{255, 255, 255}); // display opp path tangency
    displayLineList(fieldBoundLines, cv::Scalar(0, 0, 255)); // draw field bound lines
    displayPathPoints(convexPoints, cv::Scalar(0, 0, 255)); // draw field bound points
    displayPathLines(orbFiltered.getPath(), cv::Scalar(255, 200, 200)); // display orb path
    displayPathLines(oppFiltered.getPath(), cv::Scalar(255, 200, 200)); // display opp path
    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), predictDriftStop(currForward), cv::Scalar(255, 255, 0), 2);



    // calculate drive inputs based on curvature controller
    std::vector<float> driveInputs = curvatureController(followPoint, gamepad.GetRightStickY(), deltaTime, turnDirection);


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];
    
    
    processingTimeVisualizer.markEnd();
    return ret;
}






















// calculates raw follow point in a given direction
cv::Point2f AStarAttack::followPointDirection(bool CW, bool forward) {

    // determine attack radius
    float radius = radiusEquation(forward, CW);
    float ppRadius = ppRad();

    float collisionRadius = 65.0f; // radius at which we collide with opp
    float distanceToOpp = cv::norm(orbFiltered.position() - oppFiltered.position());
    bool outsideCircle = radius < distanceToOpp;
    
    // assume we're out of the circle by default
    cv::Point2f followPoint = tangentPoint(radius, CW); 

    // if we're outside the circle but the tangent point is too close then enforce the pure pursuit radius
    if(outsideCircle && cv::norm(followPoint - orbFiltered.position()) < ppRadius) {
        followPoint = ppPoint(radius, CW, ppRadius);
    }

    // if we're inside the circle
    if(!outsideCircle) {
        followPoint = followPointInsideCircle(radius, ppRadius, CW, forward, collisionRadius);
    }

    return followPoint;
}


// if we ever directly target the opp, enforce that we keep targetting them for a minimum time
cv::Point2f AStarAttack::commitToTarget(cv::Point2f followPoint, double deltaTime, float targetTime) {

    static bool enforceTarget = false; // if we're currently running the clock to force target the opp
    static float timeTargetting = 0.0f; // how long we've been force targetting the opp

    // if the follow point is basically right on the opp, turn on force targeting and reset the clock
    if(cv::norm(followPoint - oppFiltered.position()) < oppFiltered.getSizeRadius() * 0.7f) { 
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
        followPoint = oppFiltered.position(); 
        timeTargetting += deltaTime;
    }

    std::cout << "timeTargetting = " << timeTargetting << "     " << std::endl;

    return followPoint;
}


// calculates move and turn speeds to follow the followPoint
std::vector<float> AStarAttack::curvatureController(cv::Point2f followPoint, float moveSpeed, float deltaTime, int turnDirection) {

    float angleError = orbFiltered.angleTo(followPoint, currForward); // how far off we are from target
    if(turnDirection == 1) { angleError = angleWrapRad(angleError - M_PI) + M_PI; }
    if(turnDirection == -1) { angleError = angleWrapRad(angleError + M_PI) - M_PI; }

    float angleErrorChange = (angleError - prevAngleError) / deltaTime; // how the error is changing per time
    prevAngleError = angleError; // save for next time

    // determine desired path curvature/drive radius using pd controller and magic limits
    float currSpeed = cv::norm(cv::Point2f(orbFiltered.getVelFilteredSlow()[0], orbFiltered.getVelFilteredSlow()[1]));

    // 320
    float maxCurveGrip = pow(290.0f, 2.0f) / std::max(pow(currSpeed, 2), 0.1); // max curvature to avoid slipping, faster you go the less curvature you're allowed
    float maxCurveScrub = currSpeed * 0.02f; // max curvature to avoid turning in place when moving slow, faster you go the more curvature you're allowed cuz you're already moving
    float maxCurve = std::min(maxCurveGrip, maxCurveScrub); // use the lower value as the boundary

    // pd controller that increases path curvature with angle error
    float curvature = std::clamp(0.6f*angleError + 0.04f*angleErrorChange, -maxCurve, maxCurve); // 0.8f, 0.06f

    // reverese the input if we're going backwards
    if(!currForward) { moveSpeed *= -1.0f; }
    float turnSpeed = abs(moveSpeed) * curvature; // curvature = 1/radius so this is the same as w = v/r formula

    // don't demand more than 1.0 speed from any one motor side, scale everything down since that will maintain the same path curvature
    float totalSpeed = abs(moveSpeed) + abs(turnSpeed);
    if(totalSpeed > 1.0f) {
        moveSpeed /= totalSpeed;
        turnSpeed /= totalSpeed;
    }

    return std::vector<float> {moveSpeed, turnSpeed};
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
    RADIUS_CURVE_Y1 = fmax(RADIUS_CURVE_Y1, 30);

    prevRightBumper = currRightBumper;
    prevLeftBumper = currLeftBumper;
}



// equation for determining size of tangent circle
float AStarAttack::radiusEquation(bool forward, bool CW) {

    
    // float angleMargin = 50.0f * TO_RAD; // 50
    // float humanLag = 0.01f; // 0.05
    // float oppETA = oppFiltered.turnTimeMin(predictDriftStop(forward), humanLag, angleMargin, true, false);
    


    float orbETA = orbFiltered.collideETA(oppFiltered, forward);
    float oppETA = oppFiltered.turnTimeSimple(predictDriftStop(forward), oppFiltered.getWeaponAngleReach(), true, false);


    float fraction = 999999999.0f;
    if (oppETA != 0.0f) { fraction = std::max((orbETA - oppETA) / oppETA, 0.0f); }


    // let driver adjust aggressiveness with bumpers
    AdjustRadiusWithBumpers();

    // input fraction to output radius using RobotConfig variables
    std::vector<cv::Point2f> radiusCurve = {
        cv::Point2f(RADIUS_CURVE_X0, RADIUS_CURVE_Y0),
        cv::Point2f(RADIUS_CURVE_X1, RADIUS_CURVE_Y1),
        cv::Point2f(RADIUS_CURVE_X2, RADIUS_CURVE_Y2)
    };


    // radius is piecewise output
    return piecewise(radiusCurve, fraction);
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


// calculates move percent with a gain down based on angle error
float AStarAttack::calculateMovePercent(cv::Point2f followPoint, float angleThresh1, float angleThresh2, bool forward) {

    // how far we have to turn to face the follow point
    float angleError = orbFiltered.angleTo(followPoint, forward);

    // calculate move percent
    float movePercent = 1.0f;
    if(abs(angleError) > angleThresh1) {
        movePercent = 1.0f - (abs(angleError) - angleThresh1) / (angleThresh2 - angleThresh1);
    }
    if(abs(angleError) > angleThresh2) {
        movePercent = 0.0f;
    }
    // invert drive direction if needed
    if(!forward) { movePercent *= -1; }
    return movePercent;
}



// returns the tangent point of a circle with set radius around the opponent
cv::Point2f AStarAttack::tangentPoint(float radius, bool CW) {

    // circle is at the desired radius
    std::vector<cv::Point2f> circle = arcPointsFromCenter(radius, 2*M_PI, 5.0f);
    transformList(circle, oppFiltered.position(), 0.0f);

    // find the tangent point
    int tangentIndex = 0;
    
    // check first index to set initial values
    float mostTangentAngle = angle(orbFiltered.position(), circle[0]);

    for(int i = 1; i < circle.size(); i++) {
        float currentAngle = angle(orbFiltered.position(), circle[i]);
        float angleDiff = angleWrapRad(currentAngle - mostTangentAngle);
        if((angleDiff < 0 && CW) || (angleDiff > 0 && !CW)) {
            mostTangentAngle = currentAngle;
            tangentIndex = i;
        }
    }

    return circle[tangentIndex];
}


// returns intersection of pp radius with circle of set radius around opponent in the correction direction
cv::Point2f AStarAttack::ppPoint(float radius, bool CW, float ppRadius) {

    float distanceToOpp = orbFiltered.distanceTo(oppFiltered.position());

    std::vector<cv::Point2f> circle = arcPointsFromCenter(radius, 2*M_PI, 5.0f);
    transformList(circle, oppFiltered.position(), 0.0f);

    // find the greatest or least index point in the pp radius, thats the one to follow
    int defaultIndex = -1;
    int followIndex = defaultIndex;

    for(int i = 0; i < circle.size(); i++) {
        if(cv::norm(circle[i] - orbFiltered.position()) < ppRadius) {

            int indexDistance = i - followIndex;
            if(indexDistance > circle.size()/2.0f) { indexDistance -= circle.size(); }

            if((indexDistance > 0 && CW) || (indexDistance < 0 && !CW) || followIndex == defaultIndex) {
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


// display a path of points as points
void AStarAttack::displayPathPoints(std::vector<cv::Point2f>& path, cv::Scalar color) {
    for (int i = 0; i < path.size(); i++) {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        safe_circle(RobotController::GetInstance().GetDrawingImage(), centerInt, 3, color, 2);
    }
}


// displays a path of points as lines
void AStarAttack::displayPathLines(std::vector<cv::Point2f>& path, cv::Scalar color) {

    if(path.size() < 1) { return; } // no crashy

    for (int i = 0; i < path.size() - 1; i++) {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        cv::Point centerInt2(round(path[i + 1].x), round(path[i + 1].y));
        cv::line(RobotController::GetInstance().GetDrawingImage(), centerInt, centerInt2, color, 2);
    }
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



// displays a specific set of field bounds
void AStarAttack::displayFieldBoundIndices(std::vector<int> indices, cv::Scalar color) {
    for(int i = 0; i < indices.size(); i++) {
        cv::Point2f point1 = fieldBoundLines[indices[i]].getLinePoints().first;
        cv::Point2f point2 = fieldBoundLines[indices[i]].getLinePoints().second;
        cv::line(RobotController::GetInstance().GetDrawingImage(), point1, point2, color, 2);
    }
}


// displays the lines from a list
void AStarAttack::displayLineList(std::vector<Line>& lines, cv::Scalar color) {
    for(int i = 0; i < lines.size(); i++) {
        std::pair<cv::Point2f, cv::Point2f> linePoints = lines[i].getLinePoints();
        cv::line(RobotController::GetInstance().GetDrawingImage(), linePoints.first, linePoints.second, color, 2);
    }
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
    cv::line(RobotController::GetInstance().GetDrawingImage(), closestLinePoints.first, closestLinePoints.second, cv::Scalar(200, 200, 255), 2);
    
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




// finds closest wall distances to opponent by incrementing around them
float AStarAttack::wallScore(bool CW) {

    // angular range to scan for wall distances
    float sweepRange = 90.0f*TO_RAD;
    float sweepStart = 60.0f*TO_RAD;

    std::vector<cv::Point2f> scanPoints;
    std::vector<Line> scanLines = {};
    float sweepIncrement = 1.0f*TO_RAD; if(!CW) { sweepIncrement *= -1.0f; } // how much to increment sweep angle for each point

    // float startAngle = oppFiltered.angle(true) + 180.0f*TO_RAD - angleAround;
    float startAngle = angle(oppFiltered.position(), orbFiltered.position());
    float endAngle = startAngle + sweepRange;

    float closestDistance = 9999.0f;

    // sweep through the whole angle around by incrementing
    float angleOffset = sweepStart; if(!CW) { angleOffset *= -1.0f; }
    while(abs(angleOffset) < abs(sweepRange)) {
        float currAngle = startAngle + angleOffset;
        cv::Point2f testPoint = clipPointInBounds(oppFiltered.position());

        // increment the point outwards until it's out of bounds
        for(int i = 0; i < 400; i++) {
            float increment = 5.0f;
            testPoint.x += increment * cos(currAngle);
            testPoint.y += increment * sin(currAngle);

            if(!insideFieldBounds(testPoint)) { break; }
        }
        
        testPoint = closestBoundPoint(testPoint);
        scanPoints.emplace_back(testPoint);
        scanLines.emplace_back(Line(oppFiltered.position(), testPoint));
        

        float pointDistance = std::min(cv::norm(oppFiltered.position() - testPoint), 250.0);
        if(pointDistance < closestDistance) { closestDistance = pointDistance; }

        angleOffset += sweepIncrement;
    }

    cv::Scalar color = cv::Scalar(200, 255, 200);
    if(!CW) { color = cv::Scalar(200, 200, 255); }

    displayPathPoints(scanPoints, color);
    // displayLineList(scanLines, color);

    return closestDistance;
}



// // totals wall distances to opponent by incrementing around them
// float AStarAttack::wallScore(bool CW) {

//     // angle to sweep around the opp to total up the score
//     float angleAround = oppFiltered.angleTo(orbFiltered.position(), true);
//     if(CW) { angleAround = 180.0f*TO_RAD - angleAround; }
//     else { angleAround = 180.0f*TO_RAD + angleAround; }

//     std::vector<cv::Point2f> scanPoints;
//     std::vector<Line> scanLines = {};
//     float sweepIncrement = 1.0f*TO_RAD; if(!CW) { sweepIncrement *= -1.0f; } // how much to increment sweep angle for each point

//     // float startAngle = oppFiltered.angle(true) + 180.0f*TO_RAD - angleAround;
//     float startAngle = angle(oppFiltered.position(), orbFiltered.position());
//     float endAngle = startAngle + angleAround;

//     float closestDistance = 9999.0f;
//     float totalScore = 0.0f;

//     // sweep through the whole angle around by incrementing
//     float angleOffset = 0.0f;
//     while(abs(angleOffset) < abs(angleAround)) {
//         float currAngle = startAngle + angleOffset;
//         cv::Point2f testPoint = clipPointInBounds(oppFiltered.position());

//         // increment the point outwards until it's out of bounds
//         for(int i = 0; i < 400; i++) {
//             float increment = 5.0f;
//             testPoint.x += increment * cos(currAngle);
//             testPoint.y += increment * sin(currAngle);

//             if(!insideFieldBounds(testPoint)) { break; }
//         }
        
//         testPoint = closestBoundPoint(testPoint);
//         scanPoints.emplace_back(testPoint);
//         scanLines.emplace_back(Line(oppFiltered.position(), testPoint));
        

//         float pointDistance = std::min(cv::norm(oppFiltered.position() - testPoint), 250.0);
//         float pointScore = -pointDistance * std::max(M_PI - abs(angleOffset), 0.0);
//         if(pointDistance < closestDistance) { closestDistance = pointDistance; }
//         totalScore += pointScore;

//         angleOffset += sweepIncrement;
//     }

//     cv::Scalar color = cv::Scalar(200, 255, 200);
//     if(!CW) { color = cv::Scalar(200, 200, 255); }

//     displayPathPoints(scanPoints, color);
//     // displayLineList(scanLines, color);

//     // return closestDistance;
//     return totalScore;
// }



// returns sign of the input
int AStarAttack::sign(float num) {
    if(num < 0) { return -1; }
    return 1;
}


// enforces (or doesn't) a turn direction so we turn away from the op when desired
// 1 = always turn right, -1 = always turn left
int AStarAttack::enforceTurnDirection(cv::Point2f followPoint, bool forward) {

    float angleToOpp = orbFiltered.angleTo(oppFiltered.position(), forward);
    float angleToPoint = orbFiltered.angleTo(followPoint, forward);

    // if we naturally turn away from the opp, return the same follow point
    if(sign(angleToOpp) != sign(angleToPoint)) { return 0; }

    // if we turn by the opp in the first few degrees anyway just turn normally
    if(abs(angleToOpp) < 15.0f*TO_RAD) { return 0; } // 20.0f

    // if we don't have to turn past the opp anyway (or just barely turn past) just turn normally
    if(abs(angleToPoint) < abs(angleToOpp) + 40.0f*TO_RAD) { return 0; }

    // // if the point is basically the same time in both directions, enforce that we turn away from opp
    // float angleToPointABS = angle(orbFiltered.position(), followPoint);
    // float timePos = orbFiltered.pointETASim(followPoint, 0.0f, true, 0.0f, forward, false);
    // float timeNeg = orbFiltered.pointETASim(followPoint, 0.0f, false, 0.0f, forward, false);
    // float timeDif = signPoint*timePos - signPoint*timeNeg;

    // // if(timeDif > 0.15f) { // 0.15
    // if(true) {
    //     float followPointDist = cv::norm(orbFiltered.position() - followPoint);
    //     float followPointAngle = angle_wrap(orbFiltered.angle(forward) - signPoint * 175.0f*TO_RAD);
    //     return cv::Point2f(orbFiltered.position().x + followPointDist*cos(followPointAngle), orbFiltered.position().y + followPointDist*sin(followPointAngle));
    // }

    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 10, cv::Scalar(255, 255, 255), 5);


    return -sign(angleToPoint);

    // only return updated follow point if it's in the field bounds
    // if(insideFieldBounds(followPoint)) { followPointRaw = followPoint; }
    // followPointRaw = followPoint;
    // return followPointRaw;
}


// displays the path tangency data for inputted robot
void AStarAttack::displayPathTangency(FilteredRobot robot, cv::Scalar color) {

    float angle = robot.angle(true);
    cv::Point2f position = robot.position();

    float radius = 50.0f;
    cv::Point2f secondPoint = cv::Point2f(position.x + radius*cos(angle), position.y + radius*sin(angle));
    cv::line(RobotController::GetInstance().GetDrawingImage(), position, secondPoint, color, 2);
}


// calculates pure pursuit radius based on speeds
float AStarAttack::ppRad() {
    float radSlow = ASTAR_PP_RAD_SLOW;
    float radFast = ASTAR_PP_RAD_FAST;
    float speedFast = ASTAR_PP_SPEED_FAST;
    float currSpeed = cv::norm(cv::Point2f(orbFiltered.getVelFilteredSlow()[0], orbFiltered.getVelFilteredSlow()[1]));
    return radSlow + ((radFast - radSlow) / speedFast) * currSpeed;
}



// accounts for all bound stuff, including collisions with wall segments that jut into the field
cv::Point2f AStarAttack::avoidBounds(cv::Point2f rawFollowPoint) {

    // start with a check if the line to the follow point intersects boundary lines
    // clipped into field, just used as a comparison point for collisions
    cv::Point2f travelStart = clipPointInBounds(orbFiltered.position());
    cv::Point2f travelEnd = clipPointInBounds(rawFollowPoint);

    safe_circle(RobotController::GetInstance().GetDrawingImage(), travelStart, 5, cv::Scalar(255, 255, 255), 3); // display start point


    Line travelLine(travelStart, travelEnd); // line connecting orb pos and follow point
    std::vector<int> boundaryHits = {}; // indices of lines that intersect travel line

    // find all the convex points that are possibly in the way, the ones that belong to lines that are being collided with
    for(int i = 0; i < fieldBoundLines.size(); i++) {
        if(fieldBoundLines[i].doesIntersectLine(travelLine)) { 
            boundaryHits.emplace_back(i);
        }
    }

    displayFieldBoundIndices(boundaryHits, cv::Scalar(180, 180, 255)); // highlight lines that are intersecting with traversal path


    std::vector<int> convexCount(convexPoints.size(), 0); // tracks the number of each convex point found
    for(int i = 0; i < boundaryHits.size(); i++) {
        // if(boundaryHits[i] == 2 || boundaryHits[i] == 7 || boundaryHits[i] == 15) { continue; } // full field
        if(boundaryHits[i] == 1 || boundaryHits[i] == 9) { continue; } // no upper corners

        std::pair<cv::Point2f, cv::Point2f> linePoints = fieldBoundLines[boundaryHits[i]].getLinePoints();
        
        // find indices in convex list, if they exist
        int point1Index = vectorPointIndex(convexPoints, linePoints.first);
        int point2Index = vectorPointIndex(convexPoints, linePoints.second);

        if(point1Index != -1) { convexCount[point1Index]++; }
        if(point2Index != -1) { convexCount[point2Index]++; }
    }


    std::vector<cv::Point2f> ppWallPoints = PurePursuit::followPath(orbFiltered.position(), fieldBoundPoints, ppRad());


    bool usingConvexPoint = false; // if we're using a convex point as the follow point, don't check for clipping later if we are

    // search for closest convex points with any collision lines associated with it
    float closestConvex = 999999.9f;
    for(int i = 0; i < convexCount.size(); i++) {
        float convexDistance = cv::norm(convexPoints[i] - orbFiltered.position());
        if(convexCount[i] > 0 && convexDistance < closestConvex) {
            rawFollowPoint = convexPoints[i];
            closestConvex = convexDistance;
            usingConvexPoint = true;

            // if this convex point is too close and we have wall intersections to use
            if(cv::norm(rawFollowPoint - orbFiltered.position()) < ppRad() && ppWallPoints.size() > 0) {

                // use the pp point that's closest to opponent. this is cope but works most of the time
                float closestPP = cv::norm(ppWallPoints[0] - oppFiltered.position());
                rawFollowPoint = ppWallPoints[0];
                for(int i = 1; i < ppWallPoints.size(); i++) {
                    float newPPDistance = cv::norm(ppWallPoints[i] - oppFiltered.position());
                    if(newPPDistance < closestPP) {
                        closestPP = newPPDistance;
                        rawFollowPoint = ppWallPoints[i];
                    }
                }
            }
        }
    }

    // if the follow point is out of the field, clip it in
    if(!insideFieldBounds(rawFollowPoint) && !usingConvexPoint) {

        // display pp points
        // for(int i = 0; i < ppWallPoints.size(); i++) {
        //     safe_circle(RobotController::GetInstance().GetDrawingImage(),
        //         ppWallPoints[i],
        //         5.0f, cv::Scalar(255, 50, 50), 2);
        // }

        if(ppWallPoints.size() > 0) {
            int bestPointIndex = 0;
            float bestPointAngle = oppFiltered.angleTo(ppWallPoints[0], true);

            // find the most CW or CCW points relative to opp's angle, that's the one to follow
            for(int i = 1; i < ppWallPoints.size(); i++) {
                float testAngle = oppFiltered.angleTo(ppWallPoints[i], true);
                if((testAngle < bestPointAngle && !CW) || (testAngle > bestPointAngle && CW)) {
                    bestPointAngle = testAngle;
                    bestPointIndex = i;
                }
            }

            // set the follow point to the best index if it's more in the correct direction and if it's in the main radius
            float deltaAnglePP = angleWrapRad(angle(orbFiltered.position(), ppWallPoints[bestPointIndex]) - angle(orbFiltered.position(), rawFollowPoint));
            float distanceOppToPoint = cv::norm(oppFiltered.position() - ppWallPoints[bestPointIndex]);
            float radius = radiusEquation(true, CW);
            if(((deltaAnglePP < 0 && !CW) || (deltaAnglePP > 0 && CW)) && distanceOppToPoint < radius) {
                rawFollowPoint = ppWallPoints[bestPointIndex];
            }
        }
    } 

    return clipPointInBounds(rawFollowPoint); // make sure it's in bounds one more time
}


// score function for determining how good a follow point is, used for CW/CCW decision
float AStarAttack::directionScore(cv::Point2f followPoint, bool CW, bool forward) {
    static GraphWidget angleToPointGraph("wallWeight*wallGain", -100, 100, "deg");

    // rough scale to decide when we're reasonably far from the opponent that turning delays and stuff aren't a risk
    float farAway = 500.0f;
    float closeness = std::clamp(1.0f - ((orbFiltered.distanceTo(oppFiltered.position()) - 65.0f) / farAway), 0.0f, 1.0f);


    // how far the robot has to turn to this point
    float angleToPoint = orbFiltered.angleTo(followPoint, forward);    
    float turnGain = 20.0f * closeness; // 23


    // how far around the circle we have to go for each direction
    float directionSign = 1.0f; if(!CW) { directionSign = -1.0f; }
    float goAroundAngle = M_PI - directionSign*oppFiltered.angleTo(orbFiltered.position(), true);
    float goAroundGain = 10.0f; // 21


    // how close is the nearest wall in this direction
    float wallWeight = pow(wallScore(CW), 0.5f);
    float wallGain = -11.0f; // -0.38
       

    // how much velocity we already have built up in a given direction, ensures we don't switch to other direction randomly
    float tanVel = pow(orbFiltered.tangentVel(forward), 2.0f);
    float momentumWeight = 0.2f * closeness; // 0.5


    // penalty for using the back bc we want to use front more often
    float backWeight = 400.0f; // 40.0f

    angleToPointGraph.AddData(wallWeight*wallGain);

    
    // sum up components for score
    return radiusEquation(forward, CW) + goAroundAngle*goAroundGain + wallWeight*wallGain + abs(angleToPoint)*turnGain - tanVel*momentumWeight + backWeight*!forward;
}




// determines follow point when we're inside the main attack radius
cv::Point2f AStarAttack::followPointInsideCircle(float radius, float ppRadius, bool CW, bool forward, float collisionRadius) {

    float direction = 1.0f;
    if(CW) { direction = -1.0f; }

    float angleToOrb = oppFiltered.angleTo(orbFiltered.position(), true); 
    float distanceToOpp = cv::norm(orbFiltered.position() - oppFiltered.position());

    // first define parameters at collision radius
    float minAngleFront = -80.0f*TO_RAD; // -70
    float maxAngleFront = 10.0f*TO_RAD;

    float minAngleSide = -80.0f*TO_RAD; // -185
    float maxAngleSide = -10.0f*TO_RAD; // -60

    float sideAngle = -100.0f*TO_RAD;



    // linearly interpolate min and max angles at collision radius based on the current angle to orb
    float anglePercent = std::clamp((angleToOrb * direction) / sideAngle, 0.0f, 1.0f);
    float minAngleClose = anglePercent * (minAngleSide - minAngleFront) + minAngleFront;
    float maxAngleClose = anglePercent * (maxAngleSide - maxAngleFront) + maxAngleFront;



    
    // both angles are linearly interpolated to -90 degrees at the outer radius
    float angleAtRadius = -90.0f*TO_RAD;
    float radiusPercent = std::max((distanceToOpp - collisionRadius), 0.0f) / std::max((radius - collisionRadius), 0.01f);
    float minAngle = radiusPercent * (angleAtRadius - minAngleClose) + minAngleClose;
    minAngle = minAngleClose;
    float maxAngle = radiusPercent * (angleAtRadius - maxAngleClose) + maxAngleClose;



    // convert to world angle
    float minWorldAngle = angleWrapRad(minAngle*direction + angleToOrb + oppFiltered.angle(true));
    float maxWorldAngle = angleWrapRad(maxAngle*direction + angleToOrb + oppFiltered.angle(true));



    // if the pure pursuit point might be usable, check
    if(distanceToOpp > radius - ppRadius) {
        cv::Point2f ppFollow = ppPoint(radius, CW, ppRadius);
        float angleToPP = angle(orbFiltered.position(), ppFollow);
        float maxAngleDifference = angleWrapRad(maxWorldAngle - angleToPP);
        float minAngleDifference = angleWrapRad(minWorldAngle - angleToPP);


        // if(CW) { safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 5, cv::Scalar(255, 255, 255), 5); }


        
        if(CW && maxAngleDifference < 0) { maxWorldAngle = angleToPP; }
        if(CW && minAngleDifference < 0) { minWorldAngle = angleToPP; }
        if(!CW && maxAngleDifference > 0) { maxWorldAngle = angleToPP; }
        if(!CW && minAngleDifference > 0) { minWorldAngle = angleToPP; }
    }




    // angle to follow
    float angleRange = angleWrapRad(maxWorldAngle - minWorldAngle) * direction;
    float midAngle = minWorldAngle + (angleRange / 2) * direction;
    float angleError = angleWrapRad(orbFiltered.angle(forward) - midAngle) * direction;


    // default: assume we're in the angle range so just go forward
    float followAngle = orbFiltered.angle(forward);
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
    return cv::Point2f(orbFiltered.position().x + ppRadius*cos(followAngle), orbFiltered.position().y + ppRadius*sin(followAngle));
}


// returns the best follow point with the best score, also sets CW and forward by reference
cv::Point2f AStarAttack::chooseBestPoint(std::vector<cv::Point2f> followPoints,
    std::vector<bool> pointsCW, std::vector<bool> pointsForward, bool& CW, bool& forward) {

    if(followPoints.size() == 0) { return cv::Point2f(0, 0); } // no crashy

    // find follow point with best score
    int bestIndex = 0;
    float bestScore = directionScore(followPoints[0], pointsCW[0], pointsForward[0]);

    for(int i = 1; i < followPoints.size(); i++) {
        float newScore = directionScore(followPoints[i], pointsCW[i], pointsForward[i]);
        if(newScore < bestScore) {
            bestScore = newScore;
            bestIndex = i;
        }
    }

    // return best index
    CW = pointsCW[bestIndex];
    forward = pointsForward[bestIndex];
    return followPoints[bestIndex];
}


// predicts at what angle orb will drive into opp if we decide to kill rn
cv::Point2f AStarAttack::predictDriftStop(bool forward) {

    // // determine how far we'll drift while turning
    // float angleToOpp = orbFiltered.angleTo(oppFiltered.position(), forward);
    // float driftDistance = (orbFiltered.tangentVel(true) / (0.5f * orbFiltered.getMaxTurnSpeed())) * abs(sin(angleToOpp));
    // if((orbFiltered.tangentVel(true) < 0 && forward) || (orbFiltered.tangentVel(false) < 0 && !forward)) { driftDistance = 0.0f; } // if we're gonna reverse, assume no drift
    // driftDistance = std::min(orbFiltered.distanceTo(oppFiltered.position()) * 1.1f, driftDistance);

    // // determine point at which we'll stop drifting
    // cv::Point2f endDriftPoint = cv::Point2f(orbFiltered.position().x + driftDistance*cos(orbFiltered.angle(true)), orbFiltered.position().y + driftDistance*sin(orbFiltered.angle(true)));

    // return endDriftPoint;




    cv::Point2f relativeVel = orbFiltered.moveVelSlow() - oppFiltered.moveVelSlow(); // relative velocity between us and opp
    float relativeSpeed = cv::norm(relativeVel); // relative speed
    float relativeSpeedAngle = atan2(relativeVel.y, relativeVel.x); // direction of relative speed

    // whether the net speed is pointed CW or CCW around the opp
    float absAngleToOrb = angle(oppFiltered.position(), orbFiltered.position());
    int speedCW = 1;
    if(angleWrapRad(relativeSpeedAngle - absAngleToOrb) < 0.0f) { speedCW = -1; }

    float angleToOpp = orbFiltered.angleTo(oppFiltered.position(), forward);

    // what distance we'll drift around the opp as we complete the turn towards them
    float distanceToOrb = oppFiltered.distanceTo(orbFiltered.position());
    float driftDistance = (relativeSpeed / (0.5f * orbFiltered.getMaxTurnSpeed())) * sin(std::min((double) abs(angleToOpp), 90.0f*TO_RAD));
    driftDistance = std::min(distanceToOrb * 1.1f, driftDistance); // limit to be less than our distance away from the opp (covers edge case)




    // calculate the radians around the opponent we'll drift
    float angleToOrbAbs = abs(oppFiltered.angleTo(orbFiltered.position(), true));
    float driftScaleAngle = std::clamp((angleToOrbAbs - oppFiltered.getWeaponDriftScaleReach()) / (oppFiltered.getWeaponAngleReach() - oppFiltered.getWeaponDriftScaleReach()), 0.0f, 1.0f); // scale factor for the drift angle

    float collisionRadius = orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius();
    float driftScaleRadius = std::clamp((distanceToOrb - collisionRadius) / (85.0f - collisionRadius), 0.0f, 1.0f);

    float driftAngle = speedCW * driftScaleAngle * driftScaleRadius * (driftDistance / distanceToOrb); // how many radians we'll drift around opp

    

    // generate a point with the correct offset angle
    float collisionAngle = angleWrapRad(absAngleToOrb + driftAngle);
    cv::Point2f delta = cv::Point2f(distanceToOrb * cos(collisionAngle), distanceToOrb * sin(collisionAngle));
    return oppFiltered.position() + delta;
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