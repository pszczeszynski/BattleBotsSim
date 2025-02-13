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
    for(int i = 0; i < fieldBoundPoints.size() - 1; i++) {
        Line addLine = Line(fieldBoundPoints[i], fieldBoundPoints[i + 1]);
        fieldBoundLines.emplace_back(addLine);
    }


    // points that stick out
    convexPoints.emplace_back(cv::Point2f(shelfStartX, shelfY));
    convexPoints.emplace_back(cv::Point2f(shelfEndX, shelfY));
    convexPoints.emplace_back(cv::Point2f(screwX2, screwY2));
    convexPoints.emplace_back(cv::Point2f(screwX2, screwY3));
    convexPoints.emplace_back(cv::Point2f(screwX1, screwY2));
    convexPoints.emplace_back(cv::Point2f(screwX1, screwY3));


    // default to leading with bar
    leadingWithBar = true;


    // initialize filters
    orbFiltered = FilteredRobot(1.0f, 100.0f);
    oppFiltered = FilteredRobot(1.0f, 100.0f);
}


DriverStationMessage AStarAttack::Execute(Gamepad &gamepad)
{
    static Clock updateClock; // for loop timing
    double deltaTime = updateClock.getElapsedTime(); // reset every loop so elapsed time is loop time
    if(deltaTime == 0) { deltaTime = 0.01; } // broski what
    updateClock.markStart(); // reset for next loop

    // extrapolate our position into the future
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();
    //ourData.Extrapolate(Clock::programClock.getElapsedTime() + (50.0 / 1000.0));

    // extrapolate the opponent's position into the future
    OdometryData opponentData = ExtrapolateOpponentPos(0.0 / 1000.0, 9999999999.0f);
    opponentData = RobotController::GetInstance().odometry.Opponent();

    // update filtered positions/velocities
    orbFiltered.updateFilters(deltaTime, ourData.robotPosition, ourData.robotAngle);
    oppFiltered.updateFilters(deltaTime, opponentData.robotPosition, opponentData.robotAngle);


    // std::cout << "orb vel x = " << orbFiltered.moveVel().x << ", y = " << orbFiltered.moveVel().y << std::endl;



    // draw a crosshair on the opponent
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                oppFiltered.pos(),
                10, cv::Scalar(0, 0, 255), 2);

    // draw field bounds and convex points
    displayLineList(fieldBoundLines, cv::Scalar(0, 0, 255));
    displayPoints(convexPoints, cv::Scalar(0, 0, 255));


    //std::cout << "orb x vel = " << orbFiltered.getMoveVel().x << ", y vel = " << orbFiltered.getMoveVel().y << ", theta vel = " << orbFiltered.getTurnVel() << std::endl;


    // update distance/speed to opponent
    float distanceToOpp = cv::norm(orbFiltered.pos() - oppFiltered.pos());
    float speedToOpp = (distanceToOpp - previousDistanceToOpp) / deltaTime;
    previousDistanceToOpp = distanceToOpp;

    

    // track paths and display
    orbFiltered.updatePath();
    oppFiltered.updatePath();
    displayLines(orbFiltered.getPath(), cv::Scalar(255, 200, 200));
    displayLines(oppFiltered.getPath(), cv::Scalar(255, 200, 200));


    
    // angle each robot has to turn to face the other
    float angleToOrb = angleWrapRad(angle(oppFiltered.pos(), orbFiltered.pos()) - oppFiltered.angle());
    float angleToOpp = angleWrapRad(angle(orbFiltered.pos(), oppFiltered.pos()) - orbFiltered.angle());




    float ppRadius = std::min(90.0f, distanceToOpp - 1.0f); // minimum distance a follow point must be from the robot


    // raw follow points in each direction
    cv::Point2f followPointCW = followPointDirection(orbFiltered.pos(), orbFiltered.angle(), oppFiltered.pos(), oppFiltered.angle(), ppRadius, true);
    cv::Point2f followPointCCW = followPointDirection(orbFiltered.pos(), orbFiltered.angle(), oppFiltered.pos(), oppFiltered.angle(), ppRadius, false);
    




    float angleToCW = angleWrapRad(angle(orbFiltered.pos(), followPointCW) - orbFiltered.angle());
    float angleToCCW = angleWrapRad(angle(orbFiltered.pos(), followPointCCW) - orbFiltered.angle());

    // how far around the circle we have to go for each direction
    float goAroundAngleCW = M_PI - angleToOrb;
    float goAroundAngleCCW = M_PI + angleToOrb;


    // how much velocity is perpendicular to the center of the opp
    float orbMoveSpeed = cv::norm(orbFiltered.moveVel());
    float orbMoveDirectionRelative = atan2(orbFiltered.moveVel().y, orbFiltered.moveVel().x) - oppFiltered.angle();
    float perpMoveVel = pow(orbMoveSpeed, 2) * sin(orbMoveDirectionRelative);

    // std::cout << "perp move vel = " << perpMoveVel << std::endl;



    // float angleGain = 0.02f; // 0.2
    // bool CW = goAroundAngleCW + angleGain*abs(angleToCW) < goAroundAngleCCW + angleGain*abs(angleToCCW);

    float velGain = 0.000008f;
    bool CW = goAroundAngleCW - velGain*perpMoveVel < goAroundAngleCCW + velGain*perpMoveVel;

    // set the follow point to the follow point of the better direction
    cv::Point2f followPoint = followPointCW;
    float angleToFollowPoint = angleToCW;
    float radius = radiusEquation(orbFiltered.pos(), oppFiltered.pos(), oppFiltered.angle(), true);
    if(!CW) { 
        followPoint = followPointCCW; 
        angleToFollowPoint = angleToCCW;
        radius = radiusEquation(orbFiltered.pos(), oppFiltered.pos(), oppFiltered.angle(), false);
    }


    





    // check if the line to the follow point intersects boundary lines

    // start point is clipped into field, just used as a comparison point for collisions
    cv::Point2f travelStart = orbFiltered.pos();
    if(!insideFieldBounds(travelStart)) { travelStart = closestBoundPoint(travelStart); }

    Line travelLine(travelStart, followPoint);
    std::vector<int> boundaryHits = {}; // indices of lines that intersect travel line

    // find all the convex points that are possibly in the way, the ones that belong to lines that are being collided with
    for(int i = 0; i < fieldBoundLines.size(); i++) {
        float howCloseFirstPoint = fieldBoundLines[i].howClosePoint(travelLine.getLinePoints().first);
        float howCloseSecondPoint = fieldBoundLines[i].howClosePoint(travelLine.getLinePoints().second);
        if(fieldBoundLines[i].doesIntersectLine(travelLine) || howCloseFirstPoint < 0.1f || howCloseSecondPoint < 0.1f) { // add tolerance in case point is exactly on line (clipped points often are)
            boundaryHits.emplace_back(i);
        }
    }

    // checks for lines that are less than a certain amount apart (fills in gaps)
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

    displayFieldBoundIndices(boundaryHits, cv::Scalar(180, 180, 255)); // highlight lines that are intersecting with traversal path



    std::vector<int> convexCount(convexPoints.size(), 0); // tracks the number of each convex point found
    for(int i = 0; i < boundaryHits.size(); i++) {
        std::pair<cv::Point2f, cv::Point2f> linePoints = fieldBoundLines[boundaryHits[i]].getLinePoints();
        float point1Distance = cv::norm(linePoints.first - orbFiltered.pos());
        float point2Distance = cv::norm(linePoints.second - orbFiltered.pos());
        
        // find indices in convex list, if they exist
        int point1Index = vectorPointIndex(convexPoints, linePoints.first);
        int point2Index = vectorPointIndex(convexPoints, linePoints.second);

        if(point1Index != -1) { convexCount[point1Index]++; }
        if(point2Index != -1) { convexCount[point2Index]++; }
    }



    // use a different radius for walls, has to be smaller to not clip walls
    float ppRadiusForWalls = 70.0f;
    std::vector<cv::Point2f> ppWallPointsForWalls = PurePursuit::followPath(orbFiltered.pos(), fieldBoundPoints, ppRadiusForWalls);

    // draw wall pure pursuit radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                orbFiltered.pos(),
                ppRadiusForWalls, cv::Scalar(255, 0, 0), 1.5);


    // if we're using a convex point as the follow point, don't check for clipping later if we are
    bool usingConvexPoint = false;

    // search for closest convex points with 2 collision lines associated with it
    float closestConvex = 999999.9f;
    for(int i = 0; i < convexCount.size(); i++) {
        float convexDistance = cv::norm(convexPoints[i] - orbFiltered.pos());
        if(convexCount[i] == 2 && convexDistance < closestConvex) {
            followPoint = convexPoints[i];
            closestConvex = convexDistance;
            usingConvexPoint = true;

            // if this convex point is too close and we have wall intersections to use
            if(cv::norm(followPoint - orbFiltered.pos()) < ppRadiusForWalls && ppWallPointsForWalls.size() > 0) {

                // use the pp point that's closest to opponent
                float closestPP = cv::norm(ppWallPointsForWalls[0] - oppFiltered.pos());
                followPoint = ppWallPointsForWalls[0];
                for(int i = 1; i < ppWallPointsForWalls.size(); i++) {
                    float newPPDistance = cv::norm(ppWallPointsForWalls[i] - oppFiltered.pos());
                    if(newPPDistance < closestPP) {
                        closestPP = newPPDistance;
                        followPoint = ppWallPointsForWalls[i];
                    }
                }
            }
        }
    }


    // find pp circle intersections with the wall
    std::vector<cv::Point2f> ppWallPoints = PurePursuit::followPath(orbFiltered.pos(), fieldBoundPoints, ppRadius);

    // if the follow point is out of the field, clip it in
    if(!insideFieldBounds(followPoint) && !usingConvexPoint) {

        // display pp points
        for(int i = 0; i < ppWallPoints.size(); i++) {
            safe_circle(RobotController::GetInstance().GetDrawingImage(),
                ppWallPoints[i],
                5.0f, cv::Scalar(255, 50, 50), 2);
        }

        if(ppWallPoints.size() > 0) {
            int bestPointIndex = 0;
            float bestPointAngle = angleWrapRad(angle(oppFiltered.pos(), ppWallPoints[0]) - oppFiltered.angle());

            // find the most CW or CCW points, that's the one to follow
            for(int i = 1; i < ppWallPoints.size(); i++) {
                float testAngle = angleWrapRad(angle(oppFiltered.pos(), ppWallPoints[i]) - oppFiltered.angle());
                if((testAngle < bestPointAngle && !CW) || (testAngle > bestPointAngle && CW)) {
                    bestPointAngle = testAngle;
                    bestPointIndex = i;
                }
            }

            // set the follow point to the best index if it's more in the correct direction and if it's in the main radius
            float deltaAnglePP = angleWrapRad(angle(orbFiltered.pos(), ppWallPoints[bestPointIndex]) - angle(orbFiltered.pos(), followPoint));
            float distanceOppToPoint = cv::norm(oppFiltered.pos() - ppWallPoints[bestPointIndex]);
            if(((deltaAnglePP < 0 && !CW) || (deltaAnglePP > 0 && CW)) && distanceOppToPoint < radius) {
                followPoint = ppWallPoints[bestPointIndex];
            }
            
        }
    } 

    


    // draw main radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                    oppFiltered.pos(), radius, cv::Scalar(0, 255, 0), 2);

    // draw follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), followPoint, 5, cv::Scalar(255, 230, 230), 3);





    // if we want automatic direction reversing
    bool autoSwitching = false;

    if(!leadingWithBar) { angleToFollowPoint = angleWrapRad(angleToFollowPoint + M_PI); }
    if(abs(angleToFollowPoint) > 90.0f*TO_RAD && autoSwitching) {
        leadingWithBar = !leadingWithBar;
    }

    RobotMovement::DriveDirection direction = RobotMovement::DriveDirection::Forward;
    if(!leadingWithBar) { direction = RobotMovement::DriveDirection::Backward; }




    // hold angle to the follow point
    DriverStationMessage ret = RobotMovement::HoldAngle(orbFiltered.pos(),
                                                        followPoint,
                                                        KILL_KD_PERCENT,
                                                        TURN_THRESH_1_DEG_KILL,
                                                        TURN_THRESH_2_DEG_KILL,
                                                        MAX_TURN_POWER_PERCENT_KILL,
                                                        MIN_TURN_POWER_PERCENT_KILL,
                                                        SCALE_DOWN_MOVEMENT_PERCENT_KILL,
                                                        direction);



    // scale movement based on angle error
    float movement = gamepad.GetRightStickY() * calculateMovePercent(orbFiltered.pos(), orbFiltered.angle(), followPoint, 40.0f*TO_RAD, 80.0F*TO_RAD, leadingWithBar);
    // float movement = 1.0f * calculateMovePercent(orbFiltered.pos(), orbFiltered.angle(), followPoint, 40.0f*TO_RAD, 80.0F*TO_RAD, leadingWithBar);
    ret.autoDrive.movement = movement;
    

    return ret;
}

















// calculates raw follow point in a given direction
cv::Point2f AStarAttack::followPointDirection(cv::Point2f orbPos, float orbAngle, cv::Point2f oppPos, float oppAngle, float ppRadius, bool CW) {

    // CW = false;

    // determine radius
    float radius = radiusEquation(orbPos, oppPos, oppAngle, CW);
    float distanceToOpp = cv::norm(orbPos - oppPos);
    float collisionRadius = 70.0f; // radius at which we collide with opp
    bool outsideCircle = radius < distanceToOpp;

    float angleToOpp = angleWrapRad(angle(orbPos, oppPos) - orbAngle);
    float angleToOrb = angleWrapRad(angle(oppPos, orbPos) - oppAngle);


    // std::cout << "opp angle = " << oppAngle << std::endl;
    // std::cout << "distance to opp = " << distanceToOpp << ", radius = " << radius << std::endl;


    
    // assume we're out of the circle by default
    cv::Point2f followPoint = tangentPoint(radius, oppPos, orbPos, CW);


    
    // if we're outside the circle but the tangent point is too close
    if(outsideCircle && cv::norm(followPoint - orbPos) < ppRadius) {
        followPoint = ppPoint(radius, oppPos, orbPos, CW, ppRadius, 80.0f*TO_RAD);
    }

    // decides if we win or not
    if(!outsideCircle) {

        float direction = 1.0f;
        if(CW) { direction = -1.0f; }


        // first define parameters at collision radius
        float minAngleFront = -50.0f*TO_RAD;
        float maxAngleFront = 50.0f*TO_RAD;

        float minAngleSide = -180.0f*TO_RAD;
        float maxAngleSide = -10.0f*TO_RAD;

        float sideAngle = -120.0f*TO_RAD;




        // linearly interpolate min and max angles at collision radius based on the current angle to orb
        float anglePercent = std::clamp((angleToOrb * direction) / sideAngle, 0.0f, 1.0f);

        // std::cout << "angle percent = " << anglePercent << std::endl;

        float minAngleClose = anglePercent * (minAngleSide - minAngleFront) + minAngleFront;
        float maxAngleClose = anglePercent * (maxAngleSide - maxAngleFront) + maxAngleFront;


        
        // both angles are linearly interpolated to -90 degrees at the outer radius
        float angleAtRadius = -90.0f*TO_RAD;
        float radiusPercent = std::max((distanceToOpp - collisionRadius), 0.0f) / std::max((radius - collisionRadius), 0.01f);
        float minAngle = radiusPercent * (angleAtRadius - minAngleClose) + minAngleClose;
        float maxAngle = radiusPercent * (angleAtRadius - maxAngleClose) + maxAngleClose;


        // std::cout << "min angle = " << minAngle*TO_DEG << ", max angle = " << maxAngle*TO_DEG << std::endl;

        // std::cout << "angle to orb = " << angleToOrb << std::endl;

        // convert to world angle
        float minWorldAngle = angleWrapRad(minAngle*direction + angleToOrb + oppAngle);
        float maxWorldAngle = angleWrapRad(maxAngle*direction + angleToOrb + oppAngle);


        // std::cout << "minWorldAngle = " << minWorldAngle*TO_DEG << ", maxWorldAngle = " << maxWorldAngle*TO_DEG << std::endl;


        // angle to follow
        float angleRange = angleWrapRad(maxWorldAngle - minWorldAngle) * direction;
        float midAngle = minWorldAngle + (angleRange / 2) * direction;
        float angleError = angleWrapRad(orbAngle - midAngle) * direction;

        std::cout << "angle range = " << angleRange*TO_DEG << std::endl;

        // default: assume we're in the angle range so just go forward
        float followAngle = orbAngle;
        if(angleError < -angleRange/2) { followAngle = minWorldAngle; }
        if(angleError > angleRange/2) { followAngle = maxWorldAngle; }


        // std::cout << "midAngle = " << midAngle*TO_DEG << ", angleError = " << angleError*TO_DEG << std::endl;


        cv::Point2f minAnglePoint = cv::Point2f(orbPos.x + ppRadius*cos(minWorldAngle), orbPos.y + ppRadius*sin(minWorldAngle));
        cv::Point2f maxAnglePoint = cv::Point2f(orbPos.x + ppRadius*cos(maxWorldAngle), orbPos.y + ppRadius*sin(maxWorldAngle));
        cv::Point2f midAnglePoint = cv::Point2f(orbPos.x + ppRadius*cos(midAngle), orbPos.y + ppRadius*sin(midAngle));
        cv::line(RobotController::GetInstance().GetDrawingImage(), orbPos, minAnglePoint, cv::Scalar(255, 50, 50), 2);
        cv::line(RobotController::GetInstance().GetDrawingImage(), orbPos, maxAnglePoint, cv::Scalar(255, 50, 50), 2);
        cv::line(RobotController::GetInstance().GetDrawingImage(), orbPos, midAnglePoint, cv::Scalar(255, 80, 80), 2);


        // follow point is pp radius away at the calculated angle
        followPoint = cv::Point2f(orbPos.x + ppRadius*cos(followAngle), orbPos.y + ppRadius*sin(followAngle));
    }

    // commit to the shot if you're there and lined up
    float angleToOppThresh = 30.0f*TO_RAD;
    bool facingOpp = abs(angleToOpp) < angleToOppThresh || abs(angleToOpp) > M_PI - angleToOppThresh;
    if(distanceToOpp < collisionRadius && abs(angleToOrb) > 40.0f * TO_RAD &&  facingOpp) { followPoint = oppPos; }

    // draw pure pursuit radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                orbPos,
                ppRadius, cv::Scalar(255, 0, 0), 2);

    return followPoint;
}



// equation for determining size of tangent circle
float AStarAttack::radiusEquation(cv::Point2f orbPos, cv::Point2f oppPos, float oppAngle, bool CW) {

    float distanceToOpp = cv::norm(orbPos - oppPos);
    float angleToOrb = angleWrapRad(angle(oppPos, orbPos) - oppAngle);
    if(!CW) { angleToOrb *= -1; }

    // float collisionRadius = 70.0f;
    float maxRadius = 200.0f;
    float minRadius = 0.0f;
    float minRadiusAngle = 120.0f*TO_RAD - 0.03f*(cv::norm(orbFiltered.moveVel()))*TO_RAD; // 0.18


    // // collision radius circle
    // safe_circle(RobotController::GetInstance().GetDrawingImage(),
    //             oppPos,
    //             collisionRadius, cv::Scalar(255, 100, 100), 1.5);

    
    
    float radius = (1.0f - std::min(abs(angleToOrb)/minRadiusAngle, 1.0f)) * (maxRadius - minRadius) + minRadius;
    // float radius = (0.5f*cos(M_PI * std::min(angleToOrb/minRadiusAngle, 1.0f)) + 0.5f) * (maxRadius - minRadius) + minRadius;
    if(angleToOrb < 0.0f) { radius = maxRadius; }
    //if(distanceToOpp < collisionRadius && angleToOrb > 50.0f*TO_RAD) { radius = 0.0f; }
    return radius;
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

