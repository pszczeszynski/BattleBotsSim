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
    //OdometryData opponentData = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS_KILL / 1000.0, MAX_OPP_EXTRAP_MS_KILL);
    OdometryData opponentData = ExtrapolateOpponentPos(500.0 / 1000.0, 9999999999.0f);
    opponentData = RobotController::GetInstance().odometry.Opponent();

    // draw a crosshair on the opponent
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                opponentData.robotPosition,
                10, cv::Scalar(0, 0, 255), 2);


    oppMoveVelFilter += std::min(20.0f * deltaTime, 1.0) * (cv::norm(opponentData.robotVelocity) - oppMoveVelFilter);
    oppTurnVelFilter += std::min(20.0f * deltaTime, 1.0)* (opponentData.robotAngleVelocity - oppTurnVelFilter);
    orbMoveVelFilter += std::min(20.0f * deltaTime, 1.0) * (cv::norm(ourData.robotVelocity) - orbMoveVelFilter);



    float distanceToOpp = cv::norm(ourData.robotPosition - opponentData.robotPosition);
    float speedToOpp = (distanceToOpp - previousDistanceToOpp) / deltaTime;
    speedToOppFilter += std::min(10.0f * deltaTime, 1.0) * (speedToOpp - speedToOppFilter);
    previousDistanceToOpp = distanceToOpp;


    orbPathSpacing = 5.0f;
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

    displayPathPoints(orbPath, cv::Scalar(255, 0, 0));
    displayPathLines(orbPath, cv::Scalar(255, 0, 0));



    
    // generate the path
    float angleToOrb = angleWrapRad(angle(opponentData.robotPosition, ourData.robotPosition) - opponentData.robotAngle);

    float vMax = 500.0f; // pixels/sec
    float wMax = 8.5*M_PI; // rad/sec


    float ETA = distanceToOpp/std::max(orbMoveVelFilter, 200.0f);
    float maxMoveVel = 650.0f;
    float maxETA = 0.8f; // seconds
    float moveVelPercent = std::min(orbMoveVelFilter / maxMoveVel, 1.0f); // what percent of total speed we're at
    float ETAPercent = std::min(ETA / maxETA, 1.0f); // what percent of ETA is left
    float maxRadius = 200.0f;
    float minRadius = ETAPercent * 55.0f;
    float minStartAngle = 80.0f*TO_RAD;
    float maxStartAngle = 110.0f*TO_RAD;
    float minRadAngle = (minStartAngle - maxStartAngle) * moveVelPercent + maxStartAngle;
    

    float radius;
    if(abs(angleToOrb) < minRadAngle) {
        radius = (1.0f - std::min(abs(angleToOrb)/minRadAngle, 1.0f)) * (maxRadius - minRadius) + minRadius;
    }
    else {
        radius = (1.0f - (abs(angleToOrb) - minRadAngle) / (M_PI - minRadAngle)) * minRadius;
    }
     
    std::vector<cv::Point2f> circle;
    transformList(circle, opponentData.robotPosition, 0.0f);



    // std::cout << "angle = " << angleToOrb << std::endl;
    






    bool CW = angleToOrb > 0;
    cv::Point2f followPoint;
    float ppRadius = std::min(90.0f + 0.0f*abs(orbMoveVelFilter), distanceToOpp - 5.0f); // minimum distance a follow point must be from the robot
    bool outsideCircle = radius < distanceToOpp;
    
    // if we're outside the desired radius
    if(outsideCircle) {

        // circle is at the desired radius
        circle = arcPointsFromCenter(radius, 2*M_PI, 5.0f);
        transformList(circle, opponentData.robotPosition, 0.0f);

        // find the tangent point
        int tangentIndex = 0;
        
        // check first index to set initial values
        float mostTangentAngle = angle(ourData.robotPosition, circle[0]);

        for(int i = 1; i < circle.size(); i++) {
            float currentAngle = angle(ourData.robotPosition, circle[i]);
            float angleDiff = angleWrapRad(currentAngle - mostTangentAngle);
            if((angleDiff < 0 && CW) || (angleDiff > 0 && !CW)) {
                mostTangentAngle = currentAngle;
                tangentIndex = i;
            }
        }

        followPoint = circle[tangentIndex];
    }

    // if we're inside the desired radius or the tangent point wasn't far enough
    if(!outsideCircle || (outsideCircle && cv::norm(followPoint - ourData.robotPosition) < ppRadius)) {

        float radiusOffset = ppRadius * sin(70.0f*TO_RAD); // controls rejoin angle when we're inside the desired radius
        circle = arcPointsFromCenter(std::min(distanceToOpp + radiusOffset, radius), 2*M_PI, 5.0f);
        transformList(circle, opponentData.robotPosition, 0.0f);

        // find the greatest or least index point in the pp radius, thats the one to follow
        int followIndex = -1;

        for(int i = 0; i < circle.size(); i++) {
            if(cv::norm(circle[i] - ourData.robotPosition) < ppRadius) {

                int indexDistance = i - followIndex;
                if(indexDistance > circle.size()/2.0f) { indexDistance -= circle.size(); }

                if((indexDistance > 0 && CW) || (indexDistance < 0 && !CW) || followIndex == -1) {
                    followIndex = i;
                }
            }
        }

        followPoint = circle[followIndex];

        // draw pure pursuit radius
        safe_circle(RobotController::GetInstance().GetDrawingImage(),
                    ourData.robotPosition,
                    ppRadius, cv::Scalar(255, 0, 0), 2);
    }




    
    
    displayPathPoints(circle, cv::Scalar(0, 255, 0));
    displayPathLines(circle, cv::Scalar(0, 255, 0));


    


    

    // draw pure pursuit follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 0, 0), 2);

    // // draw pure pursuit radius
    // safe_circle(RobotController::GetInstance().GetDrawingImage(),
    //             ourData.robotPosition,
    //             radius, cv::Scalar(255, 0, 0), 2);

    

    

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
    if (abs(angleError) > (70.0f*TO_RAD)) {
        movePercent = 0.0f;
    }
    float movement = gamepad.GetRightStickY() * movePercent;


    ret.autoDrive.movement = movement;

    return ret;
}





void AStarAttack::displayPathPoints(std::vector<cv::Point2f>& path, cv::Scalar color) {
    for (int i = 0; i < path.size(); i++) {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        safe_circle(RobotController::GetInstance().GetDrawingImage(), centerInt, 3, color, 2);
    }
}
void AStarAttack::displayPathLines(std::vector<cv::Point2f>& path, cv::Scalar color) {
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