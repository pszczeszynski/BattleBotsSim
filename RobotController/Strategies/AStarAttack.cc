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



    // initialize filters
    orbFiltered = FilteredRobot(1.0f, 50.0f, 430.0f, 200.0f, 2.0f*360.0f*TO_RAD, 80.0f*360.0f*TO_RAD, 45.0f*TO_RAD, 40.0f*TO_RAD, 20.0f);
    oppFiltered = FilteredRobot(1.0f, 50.0f, 500.0f, 300.0f, 2.0f*360.0f*TO_RAD, 200.0f*360.0f*TO_RAD, 50.0f*TO_RAD, 40.0f*TO_RAD, 25.0f); // 25, 15, 25 last three
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


    // float distanceToCollision = std::max(orbFiltered.distanceTo(oppFiltered.position()) - orbFiltered.getSizeRadius() - oppFiltered.getSizeRadius(), 0.0f);
    
    // oppExtrap is the same as oppFiltered but moved into the future by some time 
    // oppExtrap = oppFiltered;
    // std::vector<cv::Point2f> dummyPath = {};
    // oppExtrap.constVelExtrapWrite(std::min(orbFiltered.ETASim(oppFiltered, dummyPath, false, false), 0.2f));
    // oppExtrap.constVelExtrapWrite(std::clamp(distanceToCollision * 0.006f, 0.0f, 0.2f));


    std::vector<cv::Point2f> oppSimPathForward = {};
    std::vector<cv::Point2f> oppSimPathBackward = {};

    FilteredRobot oppExtrapForward = orbFiltered.createVirtualOpp(oppFiltered, true, 0.25f, 0.000f, oppSimPathForward);
    FilteredRobot oppExtrapBackward = orbFiltered.createVirtualOpp(oppFiltered, false, 0.25f, 0.000f, oppSimPathBackward);



    // raw follow points in every direction
    std::vector<bool> pointsCW = {true, true, false, false};
    std::vector<bool> pointsForward = {true, false, true, false};
    // std::vector<bool> pointsCW = {true, false};
    // std::vector<bool> pointsForward = {false, false};
    // std::vector<bool> pointsCW = {true};
    // std::vector<bool> pointsForward = {true};

    // generate every possible follow point
    std::vector<cv::Point2f> followPoints = {};
    for(int i = 0; i < pointsCW.size(); i++) {
        FilteredRobot targetOpp = oppExtrapForward;
        if(pointsForward[i] == false) { targetOpp = oppExtrapBackward; }

        followPoints.emplace_back(followPointDirection(targetOpp, deltaTime, pointsCW[i], pointsForward[i]));
    }



    // choose which way to circle the opp/drive
    bool forwardInput = (gamepad.GetRightStickY() >= 0.0f);
    cv::Point2f followPoint = chooseBestPoint(oppExtrapForward, oppExtrapBackward, followPoints, pointsCW, pointsForward, CW, currForward, forwardInput, deltaTime);

    FilteredRobot targetOpp = oppExtrapForward;
    std::vector<cv::Point2f> oppSimPath = oppSimPathForward;
    if(!currForward) { 
        targetOpp = oppExtrapBackward; 
        oppSimPath = oppSimPathBackward;
    }

    followPoint = avoidBounds(targetOpp, deltaTime, followPoint); // make sure we don't hit any walls
    // followPoint = commitToTarget(followPoint, deltaTime, 0.5f);
    int turnDirection = enforceTurnDirection(targetOpp, followPoint, currForward); // force that we turn away from the opp if needed



    // float orbTime = orbETASim(oppExtrap);
    std::vector<cv::Point2f> orbSimPath = {};
    float orbTime = orbFiltered.ETASim(targetOpp, orbSimPath, false, false, currForward);
    std::cout << "orbTime = " << orbTime << std::endl;


    




    // std::cout << "CW = " << CW << ", forward = " << currForward;
    // std::cout << "follow point x = " << followPoint.x << std::endl;


    
    // cv::Point2f bound = closestBoundPoint(orbFiltered.position());
    // safe_circle(RobotController::GetInstance().GetDrawingImage(), bound, 5, cv::Scalar(255, 230, 230), 3);



    // float orbETA = orbFiltered.collideETA(oppFiltered, currForward);
    // float oppETA = oppFiltered.turnTimeSimple(predictDriftStop(currForward), oppFiltered.getWeaponAngleReach(), true, false);


    // float fraction = 999999999.0f;
    // if (oppETA != 0.0f) { fraction = std::max((orbETA - oppETA) / oppETA, 0.0f); }

    // std::cout << "fraction = " << fraction << std::endl;
    // prevFraction = fraction;




    // colors
    cv::Scalar colorOrb = cv::Scalar(255, 200, 0);
    cv::Scalar colorOrbLight = cv::Scalar(255, 230, 200);
    if(!currForward) { 
        colorOrb = cv::Scalar(0, 230, 255); 
        colorOrbLight = cv::Scalar(200, 230, 255);
    }

    cv::Scalar colorOpp = cv::Scalar(0, 0, 255);
    cv::Scalar colorOppLight = cv::Scalar(200, 200, 255);


    bool colliding = orbFiltered.distanceTo(oppFiltered.position()) < orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius() + 10.0f;
    bool facing = abs(orbFiltered.angleTo(oppFiltered.position(), currForward)) < orbFiltered.getWeaponAngleReach();
    bool oppFacing = abs(oppFiltered.angleTo(orbFiltered.position(), true)) < oppFiltered.getWeaponAngleReach();

    // turn opp green if we hit them
    if(colliding && facing && !oppFacing) { 
        colorOpp = cv::Scalar(200, 255, 200); 
        colorOppLight = cv::Scalar(200, 255, 200); 
        emote(); // DO NOT DELETE
    }

    // turn orb red if we get hit
    if(colliding && oppFacing) {
        colorOrb = cv::Scalar(0, 50, 255);
        colorOrbLight = cv::Scalar(0, 50, 255);
    }


    // display things we want
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3); // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), targetOpp.position(), radiusEquation(targetOpp, deltaTime, currForward, CW), cv::Scalar(200, 200, 255), 2); // draw radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRad(), colorOrbLight, 2); // draw pp radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), targetOpp.position(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), targetOpp.position(), targetOpp.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size

    displayPathTangency(targetOpp, cv::Scalar{255, 255, 255}); // display opp path tangency
    displayPathPoints(orbSimPath, colorOrb); // display orb's simulated path
    displayPathPoints(oppSimPath, colorOppLight); // display opp's simulated path
    displayLineList(fieldBoundLines, cv::Scalar(0, 0, 255)); // draw field bound lines
    displayPathPoints(convexPoints, cv::Scalar(0, 0, 255)); // draw field bound points
    displayPathLines(orbFiltered.getPath(), cv::Scalar(255, 200, 200)); // display orb path
    displayPathLines(targetOpp.getPath(), cv::Scalar(255, 200, 200)); // display opp path
    // cv::line(RobotController::GetInstance().GetDrawingImage(), targetOpp.position(), predictDriftStop(targetOpp, currForward), cv::Scalar(255, 255, 0), 2);

    std::string forwardStatus = "Forward"; 
    if(!currForward) { forwardStatus = "Backward"; }
    std::string CWStatus = "CW"; if(!CW) { CWStatus = "CCW"; }

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Orbiting", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), CWStatus, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);




    // calculate drive inputs based on curvature controller,           0.8, 0.06
    std::vector<float> driveInputs = orbFiltered.curvatureController(followPoint, 0.6f, 0.04f, gamepad.GetRightStickY(), deltaTime, turnDirection, currForward);


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];
    
    
    processingTimeVisualizer.markEnd();
    return ret;
}






















// calculates raw follow point in a given direction
cv::Point2f AStarAttack::followPointDirection(FilteredRobot opp, float deltaTime, bool CW, bool forward) {

    // determine attack radius
    float radius = radiusEquation(opp, deltaTime, forward, CW);
    float ppRadius = ppRad();


    float collisionRadius = orbFiltered.getSizeRadius() + opp.getSizeRadius(); // radius at which we collide with opp
    float distanceToOpp = cv::norm(orbFiltered.position() - opp.position());
    bool outsideCircle = radius < distanceToOpp;
    
    // assume we're out of the circle by default
    cv::Point2f followPoint = tangentPoint(opp, radius, CW); 

    // if we're outside the circle but the tangent point is too close then enforce the pure pursuit radius
    if(outsideCircle && orbFiltered.distanceTo(followPoint) < ppRadius) {
        followPoint = ppPoint(opp, radius, CW, ppRadius);
    }

    // if we're inside the circle
    if(!outsideCircle) {
        followPoint = followPointInsideCircle(opp, radius, ppRadius, CW, forward, collisionRadius);
    }

    return followPoint;
}


// if we ever directly target the opp, enforce that we keep targetting them for a minimum time
cv::Point2f AStarAttack::commitToTarget(FilteredRobot opp, cv::Point2f followPoint, double deltaTime, float targetTime) {

    static bool enforceTarget = false; // if we're currently running the clock to force target the opp
    static float timeTargetting = 0.0f; // how long we've been force targetting the opp

    // if the follow point is basically right on the opp, turn on force targeting and reset the clock
    if(opp.distanceTo(followPoint) < opp.getSizeRadius() * 0.2f) { 
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
        followPoint = opp.position(); 
        timeTargetting += deltaTime;
    }

    // std::cout << "timeTargetting = " << timeTargetting << "     " << std::endl;

    return followPoint;
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
float AStarAttack::radiusEquation(FilteredRobot opp, float deltaTime, bool forward, bool CW) {

    // float orbETA = orbFiltered.collideETA(oppFiltered, forward);
    // float orbETA = orbETASim(oppExtrap);
    std::vector<cv::Point2f> bruh = {};
    float orbETA = orbFiltered.ETASim(opp, bruh, true, false, forward);
    float oppETA = opp.turnTimeSimple(predictDriftStop(opp, forward), opp.getWeaponAngleReach(), true, true);

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
cv::Point2f AStarAttack::tangentPoint(FilteredRobot opp, float radius, bool CW) {

    // circle is at the desired radius
    std::vector<cv::Point2f> circle = arcPointsFromCenter(radius, 2*M_PI, 5.0f);
    transformList(circle, opp.position(), 0.0f);

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
cv::Point2f AStarAttack::ppPoint(FilteredRobot opp, float radius, bool CW, float ppRadius) {

    float distanceToOpp = orbFiltered.distanceTo(opp.position());

    std::vector<cv::Point2f> circle = arcPointsFromCenter(radius, 2*M_PI, 5.0f);
    transformList(circle, opp.position(), 0.0f);

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
float AStarAttack::wallScore(FilteredRobot opp, bool CW) {

    // angular range to scan for wall distances
    float sweepRange = 90.0f*TO_RAD;
    float sweepStart = 0.0f*TO_RAD; // 60

    std::vector<cv::Point2f> scanPoints;
    std::vector<Line> scanLines = {};
    float sweepIncrement = 1.0f*TO_RAD; if(!CW) { sweepIncrement *= -1.0f; } // how much to increment sweep angle for each point

    // float startAngle = oppFiltered.angle(true) + 180.0f*TO_RAD - angleAround;
    float startAngle = angle(opp.position(), orbFiltered.position());
    float endAngle = startAngle + sweepRange;

    float closestDistance = 9999.0f;

    // sweep through the whole angle around by incrementing
    float angleOffset = sweepStart; if(!CW) { angleOffset *= -1.0f; }
    while(abs(angleOffset) < abs(sweepRange)) {
        float currAngle = startAngle + angleOffset;
        cv::Point2f testPoint = clipPointInBounds(opp.position());

        // increment the point outwards until it's out of bounds
        for(int i = 0; i < 400; i++) {
            float increment = 5.0f;
            testPoint.x += increment * cos(currAngle);
            testPoint.y += increment * sin(currAngle);

            if(!insideFieldBounds(testPoint)) { break; }
        }
        
        testPoint = closestBoundPoint(testPoint);
        scanPoints.emplace_back(testPoint);
        scanLines.emplace_back(Line(opp.position(), testPoint));
        

        float pointDistance = cv::norm(opp.position() - testPoint);
        if(pointDistance < closestDistance) { closestDistance = pointDistance; }

        angleOffset += sweepIncrement;
    }

    cv::Scalar color = cv::Scalar(200, 255, 200);
    if(!CW) { color = cv::Scalar(200, 200, 255); }

    displayPathPoints(scanPoints, color);
    // displayLineList(scanLines, color);

    return closestDistance;
}




// returns sign of the input
int AStarAttack::sign(float num) {
    if(num < 0) { return -1; }
    return 1;
}


// enforces (or doesn't) a turn direction so we turn away from the op when desired
// 1 = always turn right, -1 = always turn left
int AStarAttack::enforceTurnDirection(FilteredRobot opp, cv::Point2f followPoint, bool forward) {

    float angleToOpp = orbFiltered.angleTo(opp.position(), forward);
    float angleToPoint = orbFiltered.angleTo(followPoint, forward);

    // if we naturally turn away from the opp, return the same follow point
    if(sign(angleToOpp) != sign(angleToPoint)) { return 0; }

    // if we turn by the opp in the first few degrees anyway just turn normally
    if(abs(angleToOpp) < 15.0f*TO_RAD) { return 0; } // 20.0f

    // if we don't have to turn past the opp anyway (or just barely turn past) just turn normally
    if(abs(angleToPoint) < abs(angleToOpp) + 30.0f*TO_RAD) { return 0; } // 40


    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 10, cv::Scalar(255, 255, 255), 5);


    return -sign(angleToPoint);
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
cv::Point2f AStarAttack::avoidBounds(FilteredRobot opp, float deltaTime, cv::Point2f rawFollowPoint) {

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
            if(orbFiltered.distanceTo(rawFollowPoint) < ppRad() && ppWallPoints.size() > 0) {

                // use the pp point that's closest to opponent. this is cope but works most of the time
                float closestPP = cv::norm(ppWallPoints[0] - opp.position());
                rawFollowPoint = ppWallPoints[0];
                for(int i = 1; i < ppWallPoints.size(); i++) {
                    float newPPDistance = cv::norm(ppWallPoints[i] - opp.position());
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
            float bestPointAngle = opp.angleTo(ppWallPoints[0], true);

            // find the most CW or CCW points relative to opp's angle, that's the one to follow
            for(int i = 1; i < ppWallPoints.size(); i++) {
                float testAngle = opp.angleTo(ppWallPoints[i], true);
                if((testAngle < bestPointAngle && !CW) || (testAngle > bestPointAngle && CW)) {
                    bestPointAngle = testAngle;
                    bestPointIndex = i;
                }
            }

            // set the follow point to the best index if it's more in the correct direction and if it's in the main radius
            float deltaAnglePP = angleWrapRad(angle(orbFiltered.position(), ppWallPoints[bestPointIndex]) - angle(orbFiltered.position(), rawFollowPoint));
            float distanceOppToPoint = cv::norm(opp.position() - ppWallPoints[bestPointIndex]);
            float radius = radiusEquation(opp, deltaTime, true, CW);
            if(((deltaAnglePP < 0 && !CW) || (deltaAnglePP > 0 && CW)) && distanceOppToPoint < radius) {
                rawFollowPoint = ppWallPoints[bestPointIndex];
            }
        }
    } 

    return clipPointInBounds(rawFollowPoint); // make sure it's in bounds one more time
}


// score function for determining how good a follow point is, used for CW/CCW decision
float AStarAttack::directionScore(FilteredRobot opp, cv::Point2f followPoint, float deltaTime, bool CW, bool forward, bool forwardInput) {
    static GraphWidget angleToPointGraph("wallWeight*wallGain", -100, 100, "deg");

    // rough scale to decide when we're reasonably far from the opponent that turning delays and stuff aren't a risk
    float farAway = 500.0f;
    float closeness = std::clamp(1.0f - ((orbFiltered.distanceTo(opp.position()) - 65.0f) / farAway), 0.0f, 1.0f);


    // how far the robot has to turn to this point
    float angleToPoint = orbFiltered.angleTo(followPoint, forward);    
    float turnGain = 25.0f; // 23


    // how far around the circle we have to go for each direction
    float directionSign = 1.0f; if(!CW) { directionSign = -1.0f; }
    float goAroundAngle = M_PI - directionSign*opp.angleTo(orbFiltered.position(), true);
    float goAroundGain = 10.0f; // 10


    // how close is the nearest wall in this direction
    float wallWeight = pow(wallScore(opp, CW), 0.3f); // 0.5
    float wallGain = -30.0f; // -50
       

    // how much velocity we already have built up in a given direction, ensures we don't switch to other direction randomly
    float raw = orbFiltered.tangentVel(forward);
    float tanVel = pow(raw, 2.0f) * sign(raw);
    float momentumWeight = 0.2f * closeness; // 0.2


    // penalty for using the back bc we want to use front more often
    int directionInput = 1; if(!forwardInput) { directionInput = -1; }
    float backWeight = 150.0f*directionInput; // 40.0f

    // angleToPointGraph.AddData(wallWeight*wallGain);

    
    // sum up components for score
    return radiusEquation(opp, deltaTime, forward, CW) + goAroundAngle*goAroundGain + wallWeight*wallGain + abs(angleToPoint)*turnGain - tanVel*momentumWeight + backWeight*!forward;
}




// determines follow point when we're inside the main attack radius
cv::Point2f AStarAttack::followPointInsideCircle(FilteredRobot opp, float radius, float ppRadius, bool CW, bool forward, float collisionRadius) {

    float direction = 1.0f;
    if(CW) { direction = -1.0f; }

    float angleToOrb = opp.angleTo(orbFiltered.position(), true); 
    float distanceToOpp = cv::norm(orbFiltered.position() - opp.position());

    // first define parameters at collision radius
    float minAngleFront = -60.0f*TO_RAD; // -80
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
    float minWorldAngle = angleWrapRad(minAngle*direction + angleToOrb + opp.angle(true));
    float maxWorldAngle = angleWrapRad(maxAngle*direction + angleToOrb + opp.angle(true));



    // if the pure pursuit point might be usable, check
    if(distanceToOpp > radius - ppRadius) {
        cv::Point2f ppFollow = ppPoint(opp, radius, CW, ppRadius);
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
cv::Point2f AStarAttack::chooseBestPoint(FilteredRobot oppForward, FilteredRobot oppBackward, std::vector<cv::Point2f> followPoints,
    std::vector<bool> pointsCW, std::vector<bool> pointsForward, bool& CW, bool& forward, bool forwardInput, float deltaTime) {

    if(followPoints.size() == 0) { return cv::Point2f(0, 0); } // no crashy

    // find follow point with best score
    int bestIndex = -1;
    float bestScore = 0.0f;

    for(int i = 0; i < followPoints.size(); i++) {

        FilteredRobot opp = oppForward;
        if(pointsForward[1] == false) { opp = oppBackward; }

        float newScore = directionScore(opp, followPoints[i], deltaTime, pointsCW[i], pointsForward[i], forwardInput);
        if(newScore < bestScore || i == 0) {
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
cv::Point2f AStarAttack::predictDriftStop(FilteredRobot opp, bool forward) {

    cv::Point2f relativeVel = orbFiltered.moveVelSlow() - opp.moveVelSlow(); // relative velocity between us and opp
    float relativeSpeed = cv::norm(relativeVel); // relative speed
    float relativeSpeedAngle = atan2(relativeVel.y, relativeVel.x); // direction of relative speed

    // whether the net speed is pointed CW or CCW around the opp
    float absAngleToOrb = angle(opp.position(), orbFiltered.position());
    int speedCW = 1;
    if(angleWrapRad(relativeSpeedAngle - absAngleToOrb) < 0.0f) { speedCW = -1; }

    float angleToOpp = orbFiltered.angleTo(opp.position(), forward);

    // what distance we'll drift around the opp as we complete the turn towards them
    float distanceToOrb = opp.distanceTo(orbFiltered.position());
    float driftDistance = (relativeSpeed / (0.5f * orbFiltered.getMaxTurnSpeed())) * sin(std::min((double) abs(angleToOpp), 90.0f*TO_RAD));
    driftDistance = std::min(distanceToOrb * 1.1f, driftDistance); // limit to be less than our distance away from the opp (covers edge case)




    // calculate the radians around the opponent we'll drift
    float angleToOrbAbs = abs(opp.angleTo(orbFiltered.position(), true));
    float driftScaleAngle = std::clamp((angleToOrbAbs - opp.getWeaponDriftScaleReach()) / (opp.getWeaponAngleReach() - opp.getWeaponDriftScaleReach()), 0.0f, 1.0f); // scale factor for the drift angle

    float collisionRadius = orbFiltered.getSizeRadius() + opp.getSizeRadius();
    float driftScaleRadius = std::clamp((distanceToOrb - collisionRadius) / (85.0f - collisionRadius), 0.0f, 1.0f);

    float driftAngle = speedCW * driftScaleAngle * driftScaleRadius * (driftDistance / distanceToOrb); // how many radians we'll drift around opp

    

    // generate a point with the correct offset angle
    float collisionAngle = angleWrapRad(absAngleToOrb + driftAngle);
    cv::Point2f delta = cv::Point2f(distanceToOrb * cos(collisionAngle), distanceToOrb * sin(collisionAngle));
    return opp.position() + delta;
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


// WEWEWEWEWEEE
void AStarAttack::emote() {

    cv::Mat drawing = RobotController::GetInstance().GetDrawingImage();

    cv::Mat logo = cv::imread("SkullEmote.png", cv::IMREAD_UNCHANGED); // keep alpha
    if (!logo.empty()) {
        // Pick top-left corner of where you want it
        cv::Point topLeft(100, 100);

        // Make sure it fits in the drawing surface
        cv::Rect roi(topLeft.x, topLeft.y, logo.cols, logo.rows);

        // If PNG has 4 channels (BGRA), split alpha and blend
        if (logo.channels() == 4) {
            std::vector<cv::Mat> layers;
            cv::split(logo, layers);
            cv::Mat bgr, alpha;
            cv::merge(std::vector<cv::Mat>{layers[0], layers[1], layers[2]}, bgr);
            alpha = layers[3];

            // blend PNG into drawing (simple alpha composite)
            cv::Mat region = drawing(roi);
            for (int y = 0; y < roi.height; ++y) {
                for (int x = 0; x < roi.width; ++x) {
                    float a = alpha.at<uchar>(y, x) / 255.0f;
                    for (int c = 0; c < 3; ++c) {
                        region.at<cv::Vec3b>(y, x)[c] =
                            static_cast<uchar>(a * bgr.at<cv::Vec3b>(y, x)[c] +
                                            (1.0f - a) * region.at<cv::Vec3b>(y, x)[c]);
                    }
                }
            }
        } else {
            // plain BGR copy if no alpha
            logo.copyTo(drawing(roi));
        }
    }

    // std::cout << "Working directory is: " << std::filesystem::current_path() << "\n";
}