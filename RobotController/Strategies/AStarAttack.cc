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

AStarAttack::AStarAttack()
{
    // define the field
    float minX = 65.0f;
    float maxX = 675.0f;
    float minY = 55.0f;
    float maxY = 660.0f;
    float shelfStartX = 210.0f;
    float shelfEndX = 525.0f;
    float shelfY = 200.0f;
    float screwX1 = 110.0f;
    float screwX2 = 620.0f;
    float screwY1 = 140.0f;
    float screwY2 = 220.0f;
    float screwY3 = 470.0f;
    float screwY4 = 560.0f;
    float gateY = 615.0f;
    float gateX1 = 115.0f;
    float gateX2 = 620.0f;
    std::vector<cv::Point2f> pointSequence = {
        cv::Point(minX, minY),
        cv::Point2f(shelfStartX, minY),
        cv::Point2f(shelfStartX, shelfY),
        cv::Point2f(shelfEndX, shelfY),
        cv::Point2f(shelfEndX, minY),
        cv::Point2f(maxX, minY),
        cv::Point2f(maxX, screwY1),
        cv::Point2f(screwX2, screwY2),
        cv::Point2f(screwX2, screwY3),
        cv::Point2f(maxX, screwY4),
        cv::Point2f(maxX, gateY),
        cv::Point2f(gateX2, maxY),
        cv::Point2f(gateX1, maxY),
        cv::Point2f(minX, gateY),
        cv::Point2f(minX, screwY4),
        cv::Point2f(screwX1, screwY3),
        cv::Point2f(screwX1, screwY2),
        cv::Point2f(minX, screwY1),
        cv::Point2f(minX, minY), // include first point again to complete last line
    };

    // lines are defined by connected adjacent points
    for(int i = 0; i < pointSequence.size() - 1; i++) {
        Line addLine = Line(pointSequence[i], pointSequence[i + 1]);
        fieldBounds.emplace_back(addLine);
    }


    // points that stick out
    convexPoints.emplace_back(cv::Point2f(shelfStartX, shelfY));
    convexPoints.emplace_back(cv::Point2f(shelfEndX, shelfY));
    convexPoints.emplace_back(cv::Point2f(screwX2, screwY2));
    convexPoints.emplace_back(cv::Point2f(screwX2, screwY3));
    convexPoints.emplace_back(cv::Point2f(screwX1, screwY2));
    convexPoints.emplace_back(cv::Point2f(screwX1, screwY3));

}


DriverStationMessage AStarAttack::Execute(Gamepad &gamepad)
{
    static Clock updateClock; // used for loop timing
    double deltaTime = updateClock.getElapsedTime();
    if(deltaTime == 0) { deltaTime = 0.01; } // broski what
    updateClock.markStart();

    RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    // extrapolate our position into the future
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();
    //std::cout << "our vel x = " << ourData.robotVelocity.x << std::endl;
    ourData.Extrapolate(Clock::programClock.getElapsedTime() + (POSITION_EXTRAPOLATE_MS / 1000.0));

    // extrapolate the opponent's position into the future
    // OdometryData opponentData = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS_KILL / 1000.0, MAX_OPP_EXTRAP_MS_KILL);
    OdometryData opponentData = ExtrapolateOpponentPos(500.0 / 1000.0, 9999999999.0f);
    opponentData = RobotController::GetInstance().odometry.Opponent();

    // draw a crosshair on the opponent
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                opponentData.robotPosition,
                10, cv::Scalar(0, 0, 255), 2);

    // draw field bounds and convex points
    displayFieldBounds();
    displayPoints(convexPoints, cv::Scalar(0, 0, 255));


    

    // std::cout << "orb pos = " << ourData.robotPosition << std::endl;


    oppMoveVelFilter += std::min(30.0f * deltaTime, 1.0) * (cv::norm(opponentData.robotVelocity) - oppMoveVelFilter);
    oppTurnVelFilter += std::min(30.0f * deltaTime, 1.0)* (opponentData.robotAngleVelocity - oppTurnVelFilter);
    orbMoveVelFilter += std::min(30.0f * deltaTime, 1.0) * (cv::norm(ourData.robotVelocity) - orbMoveVelFilter);



    float distanceToOpp = cv::norm(ourData.robotPosition - opponentData.robotPosition);
    float speedToOpp = (distanceToOpp - previousDistanceToOpp) / deltaTime;
    speedToOppFilter += std::min(30.0f * deltaTime, 1.0) * (speedToOpp - speedToOppFilter);
    previousDistanceToOpp = distanceToOpp;



    // track orbs path and display lines
    orbPathSpacing = 30.0f;
    orbPathLength = 300.0f;
    int orbPathSize = round(orbPathLength / orbPathSpacing);

    if(orbPath.size() == 0) { orbPath.emplace_back(ourData.robotPosition); }

    if(cv::norm(ourData.robotPosition - orbPath[orbPath.size() - 1]) > orbPathSpacing) {
        if(orbPath.size() < orbPathSize) { 
            orbPath.emplace_back(ourData.robotPosition);
        }

        else {
            for(int i = 0; i < orbPath.size() - 1; i++) {
                orbPath[i] = orbPath[i + 1];
            }
            orbPath[orbPath.size() - 1] = ourData.robotPosition;
        }
    }

    //displayPathPoints(orbPath, cv::Scalar(255, 0, 0));
    displayLines(orbPath, cv::Scalar(255, 0, 0));


    
    
    float angleToOrb = angleWrapRad(angle(opponentData.robotPosition, ourData.robotPosition) - opponentData.robotAngle);
    float angleToOpp = angleWrapRad(angle(ourData.robotPosition, opponentData.robotPosition) - ourData.robotAngle);


    float turnSpeed = 2.0f*M_PI; // rad/sec, 0.4
    float collisionRadius = 70.0f;
    float minRadiusBaseAngle = 80.0f*TO_RAD;
    float maxRadius = 200.0f;
    float minRadius = 5.0f;
    float travelDistance = distanceToOpp - collisionRadius + std::max((minRadiusBaseAngle - abs(angleToOrb)), 0.0f)*(1.0f * maxRadius);

    float assumedSpeed = 230.0f; // pixels/sec, 100
    // float ETA = travelDistance / std::max(orbMoveVelFilter, assumedSpeed) + abs(angleToOpp) / turnSpeed;
    float turnTime = abs(angleToOpp) / turnSpeed;
    float ETA = travelDistance / assumedSpeed + turnTime;
    
    

    
    float minRadiusAngle = minRadiusBaseAngle + pow(ETA, 0.8f)*140.0f*TO_RAD; // 0.6, 110
    // minRadiusAngle = 80.0f*TO_RAD + travelDistance*0.012f;
    minRadiusAngle = 150.0f*TO_RAD - 0.06f*(orbMoveVelFilter)*TO_RAD;
    //minRadiusAngle = 140.0f*TO_RAD;

    
    // float radius = (1.0f - std::min((abs(angleToOrb)/minRadiusAngle), 1.0f)) * (maxRadius - minRadius) + minRadius;
    float radius = (0.5f*cos(M_PI * std::min((abs(angleToOrb)/minRadiusAngle), 1.0f)) + 0.5f) * (maxRadius - minRadius) + minRadius;

    // commit to a hit
    if(distanceToOpp < collisionRadius && abs(angleToOrb) > 50.0f*TO_RAD) { radius = 0.0f; }




    // collision radius circle
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                opponentData.robotPosition,
                collisionRadius, cv::Scalar(255, 100, 100), 1.5);


     



    cv::Point2f followPointCW;
    cv::Point2f followPointCCW;
    float ppRadius = std::min(90.0f + 0.0f*abs(orbMoveVelFilter), distanceToOpp - 1.0f); // minimum distance a follow point must be from the robot
    bool outsideCircle = radius < distanceToOpp;
    
    // if we're outside the desired radius
    if(outsideCircle) {

        // follow point is tangent point of circle
        followPointCW = tangentPoint(radius, opponentData.robotPosition, ourData.robotPosition, true);
        followPointCCW = tangentPoint(radius, opponentData.robotPosition, ourData.robotPosition, false);
    }

    // if we're inside the desired radius or the tangent point wasn't far enough
    if(!outsideCircle || (outsideCircle && cv::norm(followPointCW - ourData.robotPosition) < ppRadius)) {

        float rejoinAngle = std::max(87.0f*TO_RAD - 80.0f*TO_RAD*pow(abs(angleToOrb)/(90.0f*TO_RAD), 2.8f), -999.0f*TO_RAD);

        followPointCW = ppPoint(radius, opponentData.robotPosition, ourData.robotPosition, true, ppRadius, rejoinAngle, distanceToOpp);
        followPointCCW = ppPoint(radius, opponentData.robotPosition, ourData.robotPosition, false, ppRadius, rejoinAngle, distanceToOpp);

        // draw pure pursuit radius
        safe_circle(RobotController::GetInstance().GetDrawingImage(),
                    ourData.robotPosition,
                    ppRadius, cv::Scalar(255, 0, 0), 2);
    }


    float angleToCW = angleWrapRad(angle(ourData.robotPosition, followPointCW) - ourData.robotAngle);
    float angleToCCW = angleWrapRad(angle(ourData.robotPosition, followPointCCW) - ourData.robotAngle);

    // how far around the circle we have to go for each direction
    float goAroundAngleCW = M_PI - angleToOrb;
    float goAroundAngleCCW = M_PI + angleToOrb;


    float angleGain = 0.01f; // 0.2
    bool CW = goAroundAngleCW + angleGain*abs(angleToCW) < goAroundAngleCCW + angleGain*abs(angleToCCW);

    cv::Point2f followPoint = followPointCW;
    if(!CW) { followPoint = followPointCCW; }






    // if the follow point is out of the field, clip it in
    if(!insideFieldBounds(followPoint)) {
        followPoint = closestBoundPoint(followPoint);
    } 







    // check if the line to the follow point intersects boundary lines

    // start point is clipped into field, just used as a comparison point for collisions
    cv::Point2f travelStart = ourData.robotPosition;
    if(!insideFieldBounds(travelStart)) { travelStart = closestBoundPoint(travelStart); }

    Line travelLine(travelStart, followPoint);
    std::vector<int> boundaryHits = {}; // indices of lines that intersect travel line

    for(int i = 0; i < fieldBounds.size(); i++) {
        float howCloseFirstPoint = fieldBounds[i].howClosePoint(travelLine.getLinePoints().first);
        float howCloseSecondPoint = fieldBounds[i].howClosePoint(travelLine.getLinePoints().second);
        if(fieldBounds[i].doesIntersectLine(travelLine) || howCloseFirstPoint < 0.1f || howCloseSecondPoint < 0.1f) { // add tolerance in case point is exactly on line (clipped points often are)
            boundaryHits.emplace_back(i);
        }
    }

    // checks for lines that are less than a certain amount apart
    for(int i = 0; i < boundaryHits.size(); i++) {
        if(boundaryHits.size() - 1 - i > 0) {
            if(boundaryHits[i + 1] - boundaryHits[i] <= 3) {
                int insertLineIndex = boundaryHits[i + 1] - 1;
                while(insertLineIndex > boundaryHits[i]) {
                    boundaryHits.insert(boundaryHits.begin() + i + 1, insertLineIndex);
                    insertLineIndex--;
                }
            }
        }
    }

    //displayLineList(boundaryHits, cv::Scalar(255, 255, 255));
    displayFieldBoundIndices(boundaryHits, cv::Scalar(180, 180, 255));


    std::vector<int> convexCount(convexPoints.size(), 0); // tracks the number of each convex point found

    for(int i = 0; i < boundaryHits.size(); i++) {
        std::pair<cv::Point2f, cv::Point2f> linePoints = fieldBounds[boundaryHits[i]].getLinePoints();
        float point1Distance = cv::norm(linePoints.first - ourData.robotPosition);
        float point2Distance = cv::norm(linePoints.second - ourData.robotPosition);
        
        // find indices in convex list, if they exist
        int point1Index = vectorPointIndex(convexPoints, linePoints.first);
        int point2Index = vectorPointIndex(convexPoints, linePoints.second);

        if(point1Index != -1) { convexCount[point1Index]++; }
        if(point2Index != -1) { convexCount[point2Index]++; }
    }

    // search for closest convex points with 2 collision lines associated with it
    float closestConvex = 999999.9f;
    for(int i = 0; i < convexCount.size(); i++) {
        float convexDistance = cv::norm(convexPoints[i] - ourData.robotPosition);
        if(convexCount[i] == 2 && convexDistance < closestConvex) {
            followPoint = convexPoints[i];
            closestConvex = convexDistance;
        }
    }





    

    
    

    // draw circle
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                    opponentData.robotPosition,
                    radius, cv::Scalar(0, 255, 0), 2);



    

    // draw pure pursuit follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 0, 0), 2);



    

    // hold angle to the follow point
    DriverStationMessage ret = RobotMovement::HoldAngle(ourData.robotPosition,
                                                        followPoint,
                                                        KILL_KD_PERCENT,
                                                        TURN_THRESH_1_DEG_KILL,
                                                        TURN_THRESH_2_DEG_KILL,
                                                        MAX_TURN_POWER_PERCENT_KILL,
                                                        MIN_TURN_POWER_PERCENT_KILL,
                                                        SCALE_DOWN_MOVEMENT_PERCENT_KILL,
                                                        direction);


    float angleToFollowPoint = atan2(followPoint.y - ourData.robotPosition.y, followPoint.x - ourData.robotPosition.x);
    float angleError = angleWrapRad(angleToFollowPoint - ourData.robotAngle);


    float movePercent = std::max(1 - abs(angleError)/(20000.0f*TO_RAD), 0.0);
    if (abs(angleError) > (50.0f*TO_RAD)) {
        movePercent = 0.0f;
    }

    // clip the move percent if we're really far out
    // float maxMove = 1.0f;
    // if(distanceToOpp > maxRadius * 1.8f) {
    //     maxMove = 0.7f;
    // }
    // if(movePercent > maxMove) { movePercent = maxMove; }

    float movement = gamepad.GetRightStickY() * movePercent;


    ret.autoDrive.movement = movement;

    return ret;
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
cv::Point2f AStarAttack::ppPoint(float radius, cv::Point2f oppPos, cv::Point2f orbPos, bool CW, float ppRadius, float rejoinAngle, float distanceToOpp) {

    float radiusOffset = ppRadius * sin(rejoinAngle); // controls rejoin angle when we're inside the desired radius
    std::vector<cv::Point2f> circle = arcPointsFromCenter(std::min(distanceToOpp + radiusOffset, radius), 2*M_PI, 5.0f);
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
    

    return circle[followIndex];
}



// does a test line intersect any boundary line
bool AStarAttack::intersectsAnyBound(Line testLine) {
    for(int i = 0; i < fieldBounds.size(); i++) {
        float howCloseFirstPoint = fieldBounds[i].howClosePoint(testLine.getLinePoints().first);
        float howCloseSecondPoint = fieldBounds[i].howClosePoint(testLine.getLinePoints().second);
        if(fieldBounds[i].doesIntersectLine(testLine) || howCloseFirstPoint < 0.1f || howCloseSecondPoint < 0.1f) { // add tolerance in case point is exactly on line (clipped points often are)
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
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        cv::Point centerInt2(round(path[i + 1].x), round(path[i + 1].y));
        double progress = (double)i / path.size();
        cv::line(RobotController::GetInstance().GetDrawingImage(), centerInt, centerInt2, color, 2);
    }
}


// generates a list of points in order from the start point
std::vector<cv::Point2f> AStarAttack::arcPointsFromOrigin(float radius, float angle, float pointSpacing) {

    float arcLength = radius * angle;
    float thetaIncrement = pointSpacing / radius;
    int increments = (int) (angle / thetaIncrement);

    std::vector<cv::Point2f> points;

    for (int i = 0; i < increments; i++) {
        float currentAngle = i * thetaIncrement;
        float currentX = radius*cos(currentAngle) - radius;
        float currentY = radius*sin(currentAngle);
        points.emplace_back(cv::Point2f(currentX, currentY));
    }

    // make sure the exact final point is added
    points.emplace_back(cv::Point2f(radius*cos(angle) - radius, radius*sin(angle)));

    return points;
}


// clips the inputted point to the inputted bounds
cv::Point2f AStarAttack::clipInBounds(cv::Point2f point, float xMin, float xMax, float yMin, float yMax) {

    float resultX = point.x;
    if(resultX < xMin) { resultX = xMin; }
    else if(resultX > xMax) { resultX = xMax; }

    float resultY = point.y;
    if(resultY < yMin) { resultY = yMin; }
    else if(resultY > yMax) { resultY = yMax; }

    return cv::Point2f(resultX, resultY);
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


// gives back points for a line from a start point at an angle
std::vector<cv::Point2f> AStarAttack::angleLineFromPoint(cv::Point2f start, float length, float angle) {

    float xOffset = length * cos(angle);
    float yOffset = length * sin(angle);
    cv::Point2f secondPoint = cv::Point2f(start.x + xOffset, start.y + yOffset);

    return std::vector<cv::Point2f> {start, secondPoint};
}


// adds 2 path vectors to make a bigger one
std::vector<cv::Point2f> AStarAttack::addPaths(std::vector<cv::Point2f> path1, std::vector<cv::Point2f> path2) {

    std::vector<cv::Point2f> newPath = {};

    for(int i = 0; i < path1.size(); i++) {
        newPath.emplace_back(path1[i]);
    }

    for(int i = 0; i < path2.size(); i++) {
        newPath.emplace_back(path2[i]);
    }

    return newPath;
}


// reverses a list of points
std::vector<cv::Point2f> AStarAttack::reversePath(std::vector<cv::Point2f> path) {

    // create a list of the same size
    std::vector<cv::Point2f> reversePath(path.size(), cv::Point2f(0, 0));

    for(int i = 0; i < path.size(); i++) {
        reversePath[reversePath.size() - 1 - i] = path[i];
    }

    return reversePath;
}


// absolute angle made by 2 points
float AStarAttack::angle(cv::Point2f point1, cv::Point2f point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}


// generates the path in a specified direction
std::pair<std::vector<cv::Point2f>, float> AStarAttack::generatePath(bool clockwise, cv::Point2f oppPos, float oppAngle, cv::Point2f orbPos, float closingSpeed, float ppRadius, float vMax, float wMax) {

    int direction = 1;
    if(!clockwise) { direction = -1; }

    float angleToOrb = angleWrapRad(angle(oppPos, orbPos) - oppAngle);

    // opponent approach line
    float fastLineAngleOffset = 80.0f*TO_RAD;
    float slowLineAngleOffset = 120.0f*TO_RAD;
    float maxClosingSpeed = 600.0f;
    float absoluteOffset = std::max(std::min((-closingSpeed / maxClosingSpeed), 1.0f), 0.0f) * (fastLineAngleOffset - slowLineAngleOffset) + slowLineAngleOffset;   

    float lineLength = 1.0f;
    float lineAngleOffset = absoluteOffset*direction;
    float lineAbsoluteAngle = oppAngle + lineAngleOffset;
    std::vector<cv::Point2f> line = angleLineFromPoint(oppPos, lineLength, lineAbsoluteAngle);

    // time for a straight line is just distance over speed
    float lineTime = lineLength / vMax;


    // if we're behind the opp beyond the line angle, just go right for them
    if(abs(angleToOrb) < 180.0f*TO_RAD && abs(angleToOrb) > absoluteOffset) {
        std::vector<cv::Point2f> path = {orbPos, oppPos};
        float pathTime = cv::norm(oppPos - orbPos) / vMax;
        return std::pair<std::vector<cv::Point2f>, float>(path, pathTime);
    }
    

    // small cornering arc
    float approachAngle = 55.0f*TO_RAD*direction; // from vertical, greater is more aggressive
    float firstArcAngle = (lineAngleOffset - 90.0f*TO_RAD*direction) + approachAngle;
    float firstArcRadius = 85.0f;
    std::vector<cv::Point2f> firstArc = arcPointsFromOrigin(-firstArcRadius*direction, -firstArcAngle, 10.0f);
    transformList(firstArc, line[1], lineAbsoluteAngle - 90.0f*TO_RAD);

    // use arc time function to find traversal time
    float firstArcTime = arcTime(firstArcRadius, abs(firstArcAngle), vMax, wMax);


    // find the tangent point, if that's not the last point of the arc then we can draw a tangent line
    // also save the first point that's in the pure pursuit radius in case we want to clip to that
    int tangentIndex = 0;
    int firstRadiusIndex = -1;
    
    // check first index to set initial values
    float mostTangentAngle = angle(orbPos, firstArc[0]);
    if(cv::norm(firstArc[0] - orbPos) < ppRadius) { firstRadiusIndex = 0; }

    for(int i = 1; i < firstArc.size(); i++) {
        float currentAngle = angle(orbPos, firstArc[i]);
        float angleDiff = angleWrapRad(currentAngle - mostTangentAngle);
        if((angleDiff < 0 && clockwise) || (angleDiff > 0 && !clockwise)) {
            mostTangentAngle = currentAngle;
            tangentIndex = i;
        }

        // if this is the first point that's closer than the pp radius
        if(cv::norm(firstArc[i] - orbPos) < ppRadius && firstRadiusIndex == -1) {
            firstRadiusIndex = i;
        }
    }

    // if a tangent line can be drawn
    if(tangentIndex != firstArc.size() - 1 && angleToOrb*direction > 0) {

        // clip the arc up to the tangent point
        std::vector<cv::Point2f> clippedArc(tangentIndex + 1, cv::Point2f(0, 0));
        for(int i = 0; i <= tangentIndex; i++) { clippedArc[i] = firstArc[i]; }

        std::vector<cv::Point2f> path = addPaths(line, clippedArc);
        path.emplace_back(orbPos);

        float clippedArcTime = (firstArcTime * tangentIndex) / firstArc.size(); // proportional to full arc
        float tangentLineTime = cv::norm(orbPos - clippedArc[clippedArc.size() - 1]) / vMax;
        float pathTime = lineTime + clippedArcTime + tangentLineTime;

        return std::pair<std::vector<cv::Point2f>, float>(reversePath(path), pathTime);
    }

    // if no tangent line can be drawn but a point is closer than the pp radius, then clip to that
    if(firstRadiusIndex != -1) {

        // clip the arc up to the tangent point
        std::vector<cv::Point2f> clippedArc(firstRadiusIndex + 1, cv::Point2f(0, 0));
        for(int i = 0; i <= firstRadiusIndex; i++) {
            clippedArc[i] = firstArc[i];
        }
        std::vector<cv::Point2f> path = addPaths(line, clippedArc);
        path.emplace_back(orbPos);

        float clippedArcTime = (firstArcTime * tangentIndex) / firstArc.size(); // proportional to full arc
        float tangentLineTime = cv::norm(orbPos - clippedArc[clippedArc.size() - 1]) / vMax;
        float pathTime = lineTime + clippedArcTime + tangentLineTime;
        return std::pair<std::vector<cv::Point2f>, float>(reversePath(path), pathTime);
    }


    // big approach arc
    float slope = tan(90.0f*TO_RAD*direction - approachAngle + oppAngle);
    float slopePerp = -1.0f / slope;
    cv::Point2f arcStart = firstArc[firstArc.size() - 1];
    cv::Point2f arcEnd = orbPos;
    float xIntersect = (slope*arcStart.x - arcStart.y - slopePerp*arcEnd.x + arcEnd.y) / (slope - slopePerp);
    float yIntersect = slope*(xIntersect - arcStart.x) + arcStart.y;
    cv::Point2f intersect(xIntersect, yIntersect);

    float xOffset = cv::norm(arcEnd - intersect);
    float yOffset = cv::norm(arcStart - intersect);


    float secondArcRadius = (pow(xOffset, 2) + pow(yOffset, 2)) / (2 * xOffset);
    float secondArcAngle = acos(1 - xOffset/secondArcRadius);


    float angleOffset = 0.0f;
    if(direction == -1) { angleOffset = 180.0f*TO_RAD; }

    if (angleWrapRad(angleToOrb + approachAngle - angleOffset) < 0.0f) {
        secondArcAngle = 360.0f*TO_RAD - secondArcAngle;
    }
    
    std::vector<cv::Point2f> secondArc = arcPointsFromOrigin(-secondArcRadius*direction, -secondArcAngle*direction, 10.0f);
    transformList(secondArc, firstArc[firstArc.size() - 1], oppAngle - approachAngle + angleOffset);


    // complete path is the sum of all the individual ones
    std::vector<cv::Point2f> path = addPaths(line, firstArc);
    path = addPaths(path, secondArc);
    path = reversePath(path);

    float secondArcTime = arcTime(secondArcRadius, secondArcAngle, vMax, wMax);
    float pathTime = lineTime + firstArcTime + secondArcTime;

    return std::pair<std::vector<cv::Point2f>, float>(path, pathTime);
}


// returns an estimated time to traverse an arc path
float AStarAttack::arcTime(float radius, float theta, float vMax, float wMax) {

    // epic math time
    float travelVel = vMax / (1.0f + vMax / (radius * wMax));
    return (radius * theta) / travelVel;
}


// displays the field bounds list
void AStarAttack::displayFieldBounds() {
    for(int i = 0; i < fieldBounds.size(); i++) {
        cv::Point2f point1 = fieldBounds[i].getLinePoints().first;
        cv::Point2f point2 = fieldBounds[i].getLinePoints().second;
        cv::line(RobotController::GetInstance().GetDrawingImage(), point1, point2, cv::Scalar(0, 0, 255), 2);
    }
}


// displays a specific set of field bounds
void AStarAttack::displayFieldBoundIndices(std::vector<int> indices, cv::Scalar color) {
    for(int i = 0; i < indices.size(); i++) {
        cv::Point2f point1 = fieldBounds[indices[i]].getLinePoints().first;
        cv::Point2f point2 = fieldBounds[indices[i]].getLinePoints().second;
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

    float closest = 1500.0f; // will always be further away than anything else
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

    std::pair<float, int> closestFieldBound = closestFromLineList(fieldBounds, point);
    std::pair<cv::Point2f, cv::Point2f> closestLinePoints = fieldBounds[closestFieldBound.second].getLinePoints();

    // highlight closest line
    cv::line(RobotController::GetInstance().GetDrawingImage(), closestLinePoints.first, closestLinePoints.second, cv::Scalar(200, 200, 255), 2);
    
    cv::Point2f closestWallPoint = fieldBounds[closestFieldBound.second].closestLinePoint(point);
    return closestWallPoint;
}


// if the point is inside the shape of the field
bool AStarAttack::insideFieldBounds(cv::Point2f point) {

    // define horizontal line from point
    Line testLine(point, cv::Point2f(point.x + 999999.0f, point.y));

    // count the number of intersections with the bounds
    int intersections = 0;
    for(int i = 0; i < fieldBounds.size(); i++) {
        if(testLine.doesIntersectLine(fieldBounds[i])) { intersections++; }
    }

    //std::cout << "intersections = " << intersections << std::endl;

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