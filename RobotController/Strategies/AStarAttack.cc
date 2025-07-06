#include "AStarAttack.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../UIWidgets/ClockWidget.h"
#include <cstdlib>

AStarAttack::AStarAttack()
{
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
    // fieldBoundPoints = {
    //     cv::Point(minX, minY),
    //     cv::Point2f(shelfStartX, minY),
    //     cv::Point2f(shelfStartX, shelfY),
    //     cv::Point2f(shelfEndX, shelfY),
    //     cv::Point2f(shelfEndX, minY),
    //     cv::Point2f(maxX, minY),
    //     cv::Point2f(maxX, screwY1),
    //     cv::Point2f(screwX2, screwY2),
    //     cv::Point2f(screwX2, screwY3),
    //     cv::Point2f(maxX, screwY4),
    //     cv::Point2f(maxX, gateY),
    //     cv::Point2f(gateX2, maxY),
    //     cv::Point2f(gateX1, maxY),
    //     cv::Point2f(minX, gateY),
    //     cv::Point2f(minX, screwY4),
    //     cv::Point2f(screwX1, screwY3),
    //     cv::Point2f(screwX1, screwY2),
    //     cv::Point2f(minX, screwY1),
    //     cv::Point2f(minX, minY), // include first point again to complete last line
    // };

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
    // convexPoints.emplace_back(cv::Point2f(shelfStartX, shelfY));
    // convexPoints.emplace_back(cv::Point2f(shelfEndX, shelfY));
    // convexPoints.emplace_back(cv::Point2f(screwX2, screwY2));
    // convexPoints.emplace_back(cv::Point2f(screwX1, screwY2));
    convexPoints.emplace_back(cv::Point2f(screwX2, screwY3));
    convexPoints.emplace_back(cv::Point2f(screwX1, screwY3));


    // default to leading with bar
    leadingWithBar = true;


    // initialize filters
    orbFiltered = FilteredRobot(1.0f, 100.0f, 2.0f*360.0f*TO_RAD, 40.0f*360.0f*TO_RAD);
    oppFiltered = FilteredRobot(1.0f, 100.0f, 3.0f*360.0f*TO_RAD, 120.0f*360.0f*TO_RAD); // calibrated at roughly 2.5 and 80

    oppAngleDrift = 0.0f;
    oppAnglePreviousPerfect = 0.0f;

}





DriverStationMessage AStarAttack::Execute(Gamepad &gamepad)
{
    static Clock updateClock; // for loop timing
    double deltaTime = updateClock.getElapsedTime(); // reset every loop so elapsed time is loop time
    if(deltaTime == 0) { deltaTime = 0.001; } // broski what
    updateClock.markStart(); // reset for next loop

    OdometryData orbData = RobotController::GetInstance().odometry.Robot(); // get our data
    OdometryData oppData = RobotController::GetInstance().odometry.Opponent(); // get opp data



    // simulate some drift on opp theta
    // float randomValue = static_cast<float>(rand()) / RAND_MAX;
    // float oppAngleIncrement = angleWrapRad(opponentData.robotAngle - oppAnglePreviousPerfect);
    // oppAngleDrift += (1.07f * oppAngleIncrement) + (0.005f * 2.0f * (randomValue - 0.5f));
    // oppAnglePreviousPerfect = opponentData.robotAngle;

    // update filtered positions/velocities
    orbFiltered.updateFilters(deltaTime, orbData.robotPosition, orbData.GetAngle());
    oppFiltered.updateFilters(deltaTime, oppData.robotPosition, oppData.GetAngle()); // oppFiltered.updateFiltersOpp(deltaTime, opponentData.robotPosition, oppAngleDrift);

    displayPathTangency(oppFiltered, cv::Scalar{255, 255, 255}); // display opp path tangency
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, cv::Scalar(0, 0, 255), 2); // draw a circle on the opponent
    displayLineList(fieldBoundLines, cv::Scalar(0, 0, 255)); // draw field bound lines
    displayPoints(convexPoints, cv::Scalar(0, 0, 255)); // draw field bound points


    // track paths and display
    orbFiltered.updatePath();
    oppFiltered.updatePath();
    displayLines(orbFiltered.getPath(), cv::Scalar(255, 200, 200));
    displayLines(oppFiltered.getPath(), cv::Scalar(255, 200, 200));


    

    float ppRadius = ppRad(60.0f, 100.0f, 400.0f, cv::norm(orbFiltered.moveVel())); // calculate pp radius based on move speed

    // raw follow points in each direction
    cv::Point2f followPointCW = followPointDirection(deltaTime, ppRadius, true, false);
    cv::Point2f followPointCCW = followPointDirection(deltaTime, ppRadius, false, false);
    



    // if we want automatic direction reversing
    bool autoSwitching = false;
    float autoSwitchThresh = 150.0f*TO_RAD;



    float angleToCW = orbFiltered.angleTo(followPointCW);
    float angleToCCW = orbFiltered.angleTo(followPointCCW);

    float CWWeight = angleToCW;
    float CCWWeight = angleToCCW;
    float turnGain = 40.0; // 60.0

    if(autoSwitching) {
        if(abs(CWWeight) > autoSwitchThresh) { CWWeight = angleWrapRad(2*CWWeight) / 2; }
        if(abs(CCWWeight) > autoSwitchThresh) { CCWWeight = angleWrapRad(2*CCWWeight) / 2; }
    }

    // how far around the circle we have to go for each direction
    float goAroundAngleCW = M_PI - oppFiltered.angleTo(orbFiltered.position());
    float goAroundAngleCCW = M_PI + oppFiltered.angleTo(orbFiltered.position());
    float goAroundGain = 5.0f;


    float wallGain = 0.8f; // 0.8
    float wallWeightCW = -closestWallDistance(true) * wallGain;
    float wallWeightCCW = -closestWallDistance(false) * wallGain;

    

    // which way around the opp to circle
    bool CW = radiusEquation(deltaTime, ppRadius, true) + abs(CWWeight)*turnGain + wallWeightCW + goAroundAngleCW*goAroundGain < radiusEquation(deltaTime, ppRadius, false) + abs(CCWWeight)*turnGain + wallWeightCCW + goAroundAngleCCW*goAroundGain;


    // draw the stuff that's actually used
    if(CW) {
        cv::Point2f dummy = followPointDirection(deltaTime, ppRadius, true, true);
        // std::cout << "Going CW";;
    }
    else {
        cv::Point2f dummy = followPointDirection(deltaTime, ppRadius, false, true);
        // std::cout << "Going CCW";
    }

    // std::cout << ", Orb speed = " << cv::norm(orbFiltered.moveVel()) << std::endl;
    

    // set the follow point to the follow point of the better direction
    cv::Point2f followPoint = followPointCW;
    float radius = radiusEquation(deltaTime, ppRadius, true);
    if(!CW) { 
        followPoint = followPointCCW; 
        radius = radiusEquation(deltaTime, ppRadius, false);
    }


    followPoint = avoidBounds(followPoint, ppRadius, radius, CW); // make sure we don't hit any walls


    cv::Point2f followPointCorrectWay = turnCorrectWay(followPoint); // force that we turn away from the opp if needed
    if(insideFieldBounds(followPointCorrectWay)) { followPoint = followPointCorrectWay; } // don't switch to a follow point thats out of bounds
    

    // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3);



    RobotMovement::DriveDirection direction = RobotMovement::DriveDirection::Forward;
    if(!leadingWithBar) { direction = RobotMovement::DriveDirection::Backward; }



    // hold angle to the follow point
    DriverStationMessage ret = RobotMovement::HoldAngle(orbFiltered.position(),
                                                        followPoint,
                                                        KILL_KD_PERCENT,
                                                        TURN_THRESH_1_DEG_KILL,
                                                        TURN_THRESH_2_DEG_KILL,
                                                        MAX_TURN_POWER_PERCENT_KILL,
                                                        MIN_TURN_POWER_PERCENT_KILL,
                                                        SCALE_DOWN_MOVEMENT_PERCENT_KILL,
                                                        direction);



    // scale movement based on angle error
    float movement = gamepad.GetRightStickY() * calculateMovePercent(orbFiltered.position(), orbFiltered.angle(), followPoint, 40.0f*TO_RAD, 70.0F*TO_RAD, leadingWithBar);
    ret.autoDrive.movement = movement;
    

    return ret;
}

















// calculates raw follow point in a given direction
cv::Point2f AStarAttack::followPointDirection(float deltaTime, float ppRadius, bool CW, bool display) {

    float angleToOpp = orbFiltered.angleTo(oppFiltered.position());
    float angleToOrb = oppFiltered.angleTo(orbFiltered.position());


    // determine radius
    float radius = radiusEquation(deltaTime, ppRadius, CW);
    float distanceToOpp = cv::norm(orbFiltered.position() - oppFiltered.position());
    float collisionRadius = 65.0f; // radius at which we collide with opp
    bool outsideCircle = radius < distanceToOpp;

    
    // assume we're out of the circle by default
    cv::Point2f followPoint = tangentPoint(radius, oppFiltered.position(), orbFiltered.position(), CW);


    
    // if we're outside the circle but the tangent point is too close
    if(outsideCircle && cv::norm(followPoint - orbFiltered.position()) < ppRadius) {
        followPoint = ppPoint(radius, oppFiltered.position(), orbFiltered.position(), CW, ppRadius, 80.0f*TO_RAD);
    }

    // if we're inside the circle
    if(!outsideCircle) {

        float direction = 1.0f;
        if(CW) { direction = -1.0f; }
        // float velAdvantage = cv::norm(orbFiltered.moveVel()) - cv::norm(oppFiltered.moveVel());


        // first define parameters at collision radius
        float minAngleFront = -40.0f*TO_RAD; // -70
        float maxAngleFront = 10.0f*TO_RAD;

        float minAngleSide = -185.0f*TO_RAD; // -185
        float maxAngleSide = -60.0f*TO_RAD; // -60

        // float sideAngle = -std::clamp(130.0f*TO_RAD - velAdvantage*0.15*TO_RAD, 80.0f*TO_RAD, 180.0f*TO_RAD);
        float sideAngle = -100.0f;



        // linearly interpolate min and max angles at collision radius based on the current angle to orb
        float anglePercent = std::clamp((angleToOrb * direction) / sideAngle, -0.3f, 1.0f);
        float minAngleClose = anglePercent * (minAngleSide - minAngleFront) + minAngleFront;
        float maxAngleClose = anglePercent * (maxAngleSide - maxAngleFront) + maxAngleFront;


        
        // both angles are linearly interpolated to -90 degrees at the outer radius
        float angleAtRadius = -90.0f*TO_RAD;
        float radiusPercent = std::max((distanceToOpp - collisionRadius), 0.0f) / std::max((radius - collisionRadius), 0.01f);
        float minAngle = radiusPercent * (angleAtRadius - minAngleClose) + minAngleClose;
        minAngle = minAngleClose;
        float maxAngle = radiusPercent * (angleAtRadius - maxAngleClose) + maxAngleClose;



        // convert to world angle
        float minWorldAngle = angleWrapRad(minAngle*direction + angleToOrb + oppFiltered.angle());
        float maxWorldAngle = angleWrapRad(maxAngle*direction + angleToOrb + oppFiltered.angle());


        // if the pure pursuit point might be usable, check
        if(distanceToOpp > radius - ppRadius) {
            cv::Point2f ppFollow = ppPoint(radius, oppFiltered.position(), orbFiltered.position(), CW, ppRadius, 80.0f*TO_RAD);
            float angleToPP = angle(orbFiltered.position(), ppFollow);
            float maxAngleDifference = angleWrapRad(maxWorldAngle - angleToPP);
            float minAngleDifference = angleWrapRad(minWorldAngle - angleToPP);

            
            if(CW && maxAngleDifference < 0) { maxWorldAngle = angleToPP; }
            if(CW && minAngleDifference < 0) { minWorldAngle = angleToPP; }
            if(!CW && maxAngleDifference > 0) { maxWorldAngle = angleToPP; }
            if(!CW && minAngleDifference > 0) { minWorldAngle = angleToPP; }
        }


        // angle to follow
        float angleRange = angleWrapRad(maxWorldAngle - minWorldAngle) * direction;
        float midAngle = minWorldAngle + (angleRange / 2) * direction;
        float angleError = angleWrapRad(orbFiltered.angle() - midAngle) * direction;


        // default: assume we're in the angle range so just go forward
        float followAngle = orbFiltered.angle();
        // if(abs(angleError) > angleRange/2) { followAngle = midAngle; }
        if(angleError < -angleRange/2) { followAngle = minWorldAngle; }
        if(angleError > angleRange/2) { followAngle = maxWorldAngle; }


        if(display) {
            cv::Point2f minAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(minWorldAngle), orbFiltered.position().y + ppRadius*sin(minWorldAngle));
            cv::Point2f maxAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(maxWorldAngle), orbFiltered.position().y + ppRadius*sin(maxWorldAngle));
            cv::Point2f midAnglePoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(midAngle), orbFiltered.position().y + ppRadius*sin(midAngle));
            cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), minAnglePoint, cv::Scalar(255, 50, 50), 2);
            cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), maxAnglePoint, cv::Scalar(255, 50, 50), 2);
            cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), midAnglePoint, cv::Scalar(255, 80, 80), 2);
        }
        


        // follow point is pp radius away at the calculated angle
        followPoint = cv::Point2f(orbFiltered.position().x + ppRadius*cos(followAngle), orbFiltered.position().y + ppRadius*sin(followAngle));
    }



    if(display) {
        // draw main radius
        safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), radius, cv::Scalar(0, 255, 0), 2);

        // draw pure pursuit radius
        safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRadius, cv::Scalar(255, 0, 0), 2);
    }
    

    return followPoint;
}



// equation for determining size of tangent circle
float AStarAttack::radiusEquation(float deltaTime, float ppRadius, bool CW) {

    float orbETA = orbFiltered.collideETA(oppFiltered);

    float angleMargin = 30.0f*TO_RAD;
    float humanLag = 0.13f; // 0.150
    float oppETA = std::min(oppFiltered.pointETASim(orbFiltered.position(), humanLag, true, angleMargin), oppFiltered.pointETASim(orbFiltered.position(), humanLag, false, angleMargin));


    return std::clamp(21.0f * ((orbETA - oppETA) / oppETA), 0.0f, 160.0f);
}


// calculates move percent with a gain down based on angle error
float AStarAttack::calculateMovePercent(cv::Point2f orbPos, float orbAngle, cv::Point2f followPoint, float angleThresh1, float angleThresh2, bool leadingWithBar) {

    // how far we have to turn to face the follow point
    float angleError = angleWrapRad(angle(orbPos, followPoint) - orbAngle);

    // invert angle error based on what side we're leading with
    if(!leadingWithBar) { angleError = angleWrapRad(angleError + M_PI); }

    // calculate move percent
    float movePercent = 1.0f;
    if(abs(angleError) > angleThresh1) {
        movePercent = 1.0f - (abs(angleError) - angleThresh1) / (angleThresh2 - angleThresh1);
    }
    if(abs(angleError) > angleThresh2) {
        movePercent = 0.0f;
    }
    // invert drive direction if needed
    if(!leadingWithBar) { movePercent *= -1; }
    return movePercent;
}



// returns the tangent point of a circle with set radius around the opponent
cv::Point2f AStarAttack::tangentPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW) {

    // circle is at the desired radius
    std::vector<cv::Point2f> circle = arcPointsFromCenter(radius, 2*M_PI, 5.0f);
    transformList(circle, oppPos, 0.0f);

    // find the tangent point
    int tangentIndex = 0;
    
    // check first index to set initial values
    float mostTangentAngle = angle(orbPos, circle[0]);

    for(int i = 1; i < circle.size(); i++) {
        float currentAngle = angle(orbPos, circle[i]);
        float angleDiff = angleWrapRad(currentAngle - mostTangentAngle);
        if((angleDiff < 0 && CW) || (angleDiff > 0 && !CW)) {
            mostTangentAngle = currentAngle;
            tangentIndex = i;
        }
    }

    return circle[tangentIndex];
}


// returns intersection of pp radius with circle of set radius around opponent in the correction direction
cv::Point2f AStarAttack::ppPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW, float ppRadius, float rejoinAngle) {

    float distanceToOpp = cv::norm(orbPos - oppPos);
    float radiusOffset = ppRadius * sin(rejoinAngle); // controls rejoin angle when we're inside the desired radius
    float circleRadius = std::max(std::min(distanceToOpp + radiusOffset, radius), 1.0f);

    std::vector<cv::Point2f> circle = arcPointsFromCenter(circleRadius, 2*M_PI, 5.0f);
    transformList(circle, oppPos, 0.0f);

    // find the greatest or least index point in the pp radius, thats the one to follow
    int followIndex = -1;

    for(int i = 0; i < circle.size(); i++) {
        if(cv::norm(circle[i] - orbPos) < ppRadius) {

            int indexDistance = i - followIndex;
            if(indexDistance > circle.size()/2.0f) { indexDistance -= circle.size(); }

            if((indexDistance > 0 && CW) || (indexDistance < 0 && !CW) || followIndex == -1) {
                followIndex = i;
            }
        }
    }

    // draw reduced circle
    if(distanceToOpp + radiusOffset < radius) {
        safe_circle(RobotController::GetInstance().GetDrawingImage(),
                    oppPos,
                    distanceToOpp + radiusOffset, cv::Scalar(0, 255, 0), 1.5);
    }
    
    if( followIndex < 0) {
        return oppPos;
    }

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



void AStarAttack::displayPoints(std::vector<cv::Point2f>& path, cv::Scalar color) {
    for (int i = 0; i < path.size(); i++) {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        safe_circle(RobotController::GetInstance().GetDrawingImage(), centerInt, 3, color, 2);
    }
}
void AStarAttack::displayLines(std::vector<cv::Point2f>& path, cv::Scalar color) {

    if(path.size() < 1) { return; } // no crashy

    for (size_t i = 0; i < path.size() - 1; i++)
    {
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
        std::vector<cv::Point2f> increments = {cv::Point2f(increment, increment), cv::Point2f(increment, 0), cv::Point2f(increment, -increment), cv::Point2f(0, -increment), cv::Point2f(-increment, -increment), cv::Point2f(-increment, 0.0), cv::Point2f(-increment, increment), cv::Point2f(0.0, increment)};
        for(int i = 0; i < increments.size(); i++) {
            cv::Point2f inBound = cv::Point2f(onBoundStart.x + increments[i].x, onBoundStart.y + increments[i].y);
            if(insideFieldBounds(inBound)) { returnPoint = inBound; }
        }
    }
    return returnPoint;
}




// finds closest
float AStarAttack::closestWallDistance(bool CW) {

    float angleAround = 180.0f*TO_RAD - angleWrapRad(angle(oppFiltered.position(), orbFiltered.position()) - oppFiltered.angle());
    // if(CW) { angleAround = 180.0f*TO_RAD - angleAround; }
    // else { angleAround = 180.0f*TO_RAD + angleAround; }

    std::vector<cv::Point2f> scanPoints;
    std::vector<Line> scanLines = {};
    int numPoints = 60.0f;

    float startAngle = oppFiltered.angle() + 180.0f*TO_RAD - angleAround;
    float endAngle = startAngle + 2*(CW - 0.5f)*60.0f*TO_RAD;

    float closestDistance = 9999.0f;

    for(int i = 0; i < numPoints; i++) {
        float currAngle = startAngle + ((endAngle - startAngle) / numPoints) * i;
        cv::Point2f testPoint = oppFiltered.position();

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
        
        float pointDistance = cv::norm(oppFiltered.position() - testPoint);
        if(pointDistance < closestDistance) { closestDistance = pointDistance; }
    }

    cv::Scalar color = cv::Scalar(200, 255, 200);
    if(!CW) { color = cv::Scalar(200, 200, 255); }

    displayPoints(scanPoints, color);
    // displayLineList(scanLines, color);

    return closestDistance;
}



// makes us preferentially turn away from the opponent if turning around
cv::Point2f AStarAttack::turnCorrectWay(cv::Point2f followPoint) {

    float angleToOpp = orbFiltered.angleTo(oppFiltered.position());
    float angleToPoint = orbFiltered.angleTo(followPoint);

    float signOpp = angleToOpp / abs(angleToOpp);
    float signPoint = angleToPoint / abs(angleToPoint);

    // if we naturally turn away from the opp, return the same follow point
    if(signOpp != signPoint) { return followPoint; }

    // if we don't have to turn past the opp anyway, return the same follow point
    if(abs(angleToPoint) < abs(angleToOpp) + 20.0f*TO_RAD) { return followPoint; }

    // if the point is basically the same time in both directions, enforce that we turn away from opp
    float angleToPointABS = angle(orbFiltered.position(), followPoint);
    float timePos = orbFiltered.timeToTurnToAngle(orbFiltered.getPosFiltered(), orbFiltered.getVelFiltered(), angleToPointABS, true);
    float timeNeg = orbFiltered.timeToTurnToAngle(oppFiltered.getPosFiltered(), oppFiltered.getVelFiltered(), angleToPointABS, false);
    float timeDif = signPoint*timePos - signPoint*timeNeg;

    if(timeDif > -0.15f) { 
        float followPointDist = cv::norm(orbFiltered.position() - followPoint);
        float followPointAngle = angleWrap(orbFiltered.angle() - signPoint * 170.0f*TO_RAD);
        return cv::Point2f(orbFiltered.position().x + followPointDist*cos(followPointAngle), orbFiltered.position().y + followPointDist*sin(followPointAngle));
    }

    return followPoint;
}


// displays the path tangency data for inputted robot
void AStarAttack::displayPathTangency(FilteredRobot robot, cv::Scalar color) {

    float angle = robot.angle();
    cv::Point2f position = robot.position();

    float radius = 50.0f;
    cv::Point2f secondPoint = cv::Point2f(position.x + radius*cos(angle), position.y + radius*sin(angle));
    cv::line(RobotController::GetInstance().GetDrawingImage(), position, secondPoint, color, 2);
}


// calculates pure pursuit radius based on speeds
float AStarAttack::ppRad(float radSlow, float radFast, float speedFast, float currSpeed) {
    return radSlow + ((radFast - radSlow) / speedFast) * currSpeed;
}



// accounts for all bound stuff, including collisions with wall segments that jut into the field
cv::Point2f AStarAttack::avoidBounds(cv::Point2f rawFollowPoint, float ppRadius, float radius, bool CW) {

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


    std::vector<cv::Point2f> ppWallPoints = PurePursuit::followPath(orbFiltered.position(), fieldBoundPoints, ppRadius);


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
            if(cv::norm(rawFollowPoint - orbFiltered.position()) < ppRadius && ppWallPoints.size() > 0) {

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
            float bestPointAngle = oppFiltered.angleTo(ppWallPoints[0]);

            // find the most CW or CCW points relative to opp's angle, that's the one to follow
            for(int i = 1; i < ppWallPoints.size(); i++) {
                float testAngle = oppFiltered.angleTo(ppWallPoints[i]);
                if((testAngle < bestPointAngle && !CW) || (testAngle > bestPointAngle && CW)) {
                    bestPointAngle = testAngle;
                    bestPointIndex = i;
                }
            }

            // set the follow point to the best index if it's more in the correct direction and if it's in the main radius
            float deltaAnglePP = angleWrapRad(angle(orbFiltered.position(), ppWallPoints[bestPointIndex]) - angle(orbFiltered.position(), rawFollowPoint));
            float distanceOppToPoint = cv::norm(oppFiltered.position() - ppWallPoints[bestPointIndex]);
            if(((deltaAnglePP < 0 && !CW) || (deltaAnglePP > 0 && CW)) && distanceOppToPoint < radius) {
                rawFollowPoint = ppWallPoints[bestPointIndex];
            }
        }
    } 

    return clipPointInBounds(rawFollowPoint); // make sure it's in bounds one more time
}