
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
#include "FollowPoint.h"
#include <limits>

AStarAttack* AStarAttack::_instance = nullptr;

AStarAttack::AStarAttack()
{
    _instance = this;
    
    // init filters
    orbFiltered = FilteredRobot(1.0f, 50.0f, 400.0f, 200.0f, 
        22.0f, 60.0f, 50.0f*TO_RAD, 
        40.0f*TO_RAD, 20.0f, 20.0f*TO_RAD, 10.0f*TO_RAD);
    oppFiltered = FilteredRobot(1.0f, 50.0f, 400.0f, 200.0f, 
        14.0f, 200.0f*360.0f*TO_RAD, 60.0f*TO_RAD, 
        40.0f*TO_RAD, 25.0f, 0.0f, 0.0f);

    // init field
    field = Field();
}





DriverStationMessage AStarAttack::Execute(Gamepad &gamepad, double rightStickY)
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
    // std::vector<bool> pointsCW = {true, true, false, false};
    // std::vector<bool> pointsForward = {true, false, true, false};
    // std::vector<bool> pointsCW = {true, false};
    // std::vector<bool> pointsForward = {false, false};



    std::vector<bool> pointsCW = {true, true, false, false};
    std::vector<bool> pointsTurnAway = {true, false, true, false};
    std::vector<bool> pointsForward = {true, true, true, true};

    // pointsCW = {true};
    // pointsTurnAway = {false};
    // pointsForward = {true};

    // pointsCW = {true, false};
    // pointsTurnAway = {false, false};
    // pointsForward = {true, true};

    // pointsCW = {true, true};
    // pointsTurnAway = {true, false};
    // pointsForward = {true, true};
    





    
    // bumpers can enforce a single follow point
    if(gamepad.GetRightBumper()) {
        pointsCW = {true};
        pointsTurnAway = {false};
        pointsForward = {true};
    }
    if(gamepad.GetLeftBumper()) {
        pointsCW = {true, false};
        pointsTurnAway = {false, false};
        pointsForward = {true, true};

        pointsCW = {true};
        pointsTurnAway = {true};
        pointsForward = {true};
    }








    // generate every possible follow point
    std::vector<FollowPoint> follows = {};
    
    for(int i = 0; i < pointsCW.size(); i++) {

        
        FollowPoint follow = createFollowPoint(pointsCW[i], pointsForward[i], pointsTurnAway[i], deltaTime);
        driveAngle(follow);
        // avoidBoundsVector(follow);

        // if(follow.badTurn) {
        //     follow = createFollowPoint(pointsCW[i], pointsForward[i], true, deltaTime);
        //     driveAngle(follow);
        // }

        follows.emplace_back(follow);         
    }


    bool forwardInput = (rightStickY >= 0.0f); // if driver is pushing forward or backward
    FollowPoint follow = chooseBestPoint(follows, forwardInput); // pick which point to use
    
    // store the followPoints for debugging/display (after scores have been computed)
    _lastFollowPoints = follows;


    // std::cout << "orb time = " << follow.orbETA << ", opp time = " << follow.oppETA<< "   ";

    
    display(follow); // display data



    // calculate drive inputs based on curvature controller
    bool turningCorrect = (orbFiltered.angleTo(oppFiltered.position(), follow.forward) > 0 && follow.CW) || (orbFiltered.angleTo(oppFiltered.position(), follow.forward) < 0 && !follow.CW);
    int enforceTurn = 0;
    if(follow.turnAway && follow.CW && !turningCorrect) { enforceTurn = 1; }
    if(follow.turnAway && !follow.CW && !turningCorrect) { enforceTurn = -1; }

    std::vector<float> driveInputs = orbFiltered.curvatureController(follow.driveAngle, 
        rightStickY, deltaTime, follow.forward, enforceTurn);


    // create and send drive command
    DriverStationMessage ret;
    ret.type = DriverStationMessageType::DRIVE_COMMAND;
    ret.driveCommand.movement = driveInputs[0];
    ret.driveCommand.turn = driveInputs[1];


    std::cout << std::endl;
    
    
    processingTimeVisualizer.markEnd();
    return ret;
}























// displays all the data
void AStarAttack::display(FollowPoint follow) {

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
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), follow.radius, cv::Scalar(200, 200, 255), 2); // draw radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRad(), colorOrbLight, 2); // draw pp radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), ppRadWall(), colorOrbLight, 1); // draw pp wall radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.opp.position(), 10, colorOpp, 2); // draw dot on the extrap opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), 10, colorOpp, 2); // draw dot on the actual opp
    safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppFiltered.getSizeRadius(), colorOpp, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbFiltered.getSizeRadius(), colorOrb, 2); // draw op size
    safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.orbSimPath.back(), 10, cv::Scalar(255, 255, 255), 5);
    // if(follow.enforceTurnDirection != 0) { safe_circle(RobotController::GetInstance().GetDrawingImage(), follow.point, 10, cv::Scalar(255, 255, 255), 5); } // bold the follow point if we're enforcing a turn direction


    // display paths
    DisplayUtils::displayPath(follow.orbSimPath, colorOrb, cv::Scalar(255, 255, 255), 6); // display orb's simulated path
    DisplayUtils::displayPath(follow.oppSimPath, colorOpp, cv::Scalar(255, 255, 255), 6); // display opp's simulated path
    DisplayUtils::displayLines(field.getBoundLines(), cv::Scalar(0, 0, 255)); // draw field bound lines
    DisplayUtils::displayPath(orbFiltered.getPath(), cv::Scalar(100, 100, 100), colorOrbLight, 3); // display orb path
    DisplayUtils::displayPath(follow.opp.getPath(), cv::Scalar(200, 200, 200), colorOppLight, 3); // display opp path


    // display lines
    cv::Point2f oppOrientationEnd = cv::Point2f(oppFiltered.position().x + oppFiltered.getSizeRadius()*cos(oppFiltered.angle(true)), oppFiltered.position().y + oppFiltered.getSizeRadius()*sin(oppFiltered.angle(true)));
    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppOrientationEnd, colorOpp, 2);


    // display text
    std::string forwardStatus = follow.forward ? "Forward" : "Backward"; 
    std::string CWStatus = follow.CW ? "CW" : "CCW";

    cv::putText(RobotController::GetInstance().GetDrawingImage(), "Orbiting", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), forwardStatus, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);
    cv::putText(RobotController::GetInstance().GetDrawingImage(), CWStatus, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, colorOrb, 2);




    // add lines to show weapon regions
    cv::Point2f oppWeapon1 = oppFiltered.position() + oppFiltered.getSizeRadius()*cv::Point2f(cos(oppFiltered.angle(true) + oppFiltered.getWeaponAngleReach()), sin(oppFiltered.angle(true) + oppFiltered.getWeaponAngleReach()));
    cv::Point2f oppWeapon2 = oppFiltered.position() + oppFiltered.getSizeRadius()*cv::Point2f(cos(oppFiltered.angle(true) - oppFiltered.getWeaponAngleReach()), sin(oppFiltered.angle(true) - oppFiltered.getWeaponAngleReach()));
    
    cv::Point2f orbWeapon1 = orbFiltered.position() + orbFiltered.getSizeRadius()*cv::Point2f(cos(orbFiltered.angle(follow.forward) + orbFiltered.getWeaponAngleReach()), sin(orbFiltered.angle(follow.forward) + orbFiltered.getWeaponAngleReach()));
    cv::Point2f orbWeapon2 = orbFiltered.position() + orbFiltered.getSizeRadius()*cv::Point2f(cos(orbFiltered.angle(follow.forward) - orbFiltered.getWeaponAngleReach()), sin(orbFiltered.angle(follow.forward) - orbFiltered.getWeaponAngleReach()));

    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppWeapon1, colorOppLight, 2);
    cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), oppWeapon2, colorOppLight, 2);

    cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbWeapon1, colorOrbLight, 2);
    cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), orbWeapon2, colorOrbLight, 2);

}




// makes a valid point
FollowPoint AStarAttack::createFollowPoint(bool CW, bool forward, bool turnAway, float deltaTime) {

    // extrapolate the opp based on this point's parameters
    std::vector<cv::Point2f> oppSimPath = {};
    FilteredRobot targetOpp = orbFiltered.createVirtualOpp(oppFiltered, forward, 
        CW, turnAway, 0.25f, oppSimPath);


    // create follow point based on the extrapolated opp's and the given parameters
    FollowPoint follow = FollowPoint(forward, CW, turnAway, targetOpp, oppSimPath);
    
    radiusEquation(follow);




    tangentPoint(follow); // main case is drive towards the tangent point on the attack radius

    bool outsideCircle = follow.radius < orbFiltered.distanceTo(oppFiltered.position()); // if orb is outside of the attack radius

    // if we're outside the circle but the tangent point is too close then enforce the pure pursuit radius
    if(outsideCircle && orbFiltered.distanceTo(follow.point) < ppRad()) { follow.point = ppPoint(follow); }

    // if we're inside the circle then use this algorithm
    if(!outsideCircle) { followPointInsideCircle(follow); }

    return follow;

}




// // check's if a follow point is valid (the robot has to turn in the intended direction to reach it)
// bool AStarAttack::followValid(FollowPoint follow) {

//     // if it predicted a turn around that we didn't actually need, recalculate without that
//     float angleToFollow = orbFiltered.angleTo(follow.point, follow.forward);
//     float angleToOpp = orbFiltered.angleTo(follow.opp.position(), follow.forward);

//     // // wrap to 0 to 360 in the correct direction
//     // if(follow.turnRight) {
//     //     angleToFollow = angleWrapRad(angleToFollow - M_PI) + M_PI; 
//     //     angleToOpp = angleWrapRad(angleToOpp - M_PI) + M_PI; 
//     // }
//     // else {
//     //     angleToFollow = angleWrapRad(angleToFollow + M_PI) - M_PI; 
//     //     angleToOpp = angleWrapRad(angleToOpp + M_PI) - M_PI; 
//     // }

    

//     // this means we predicted a turn around we shouldn't have, so this follow is not valid
//     if(abs(angleToFollow - angleToOpp) > M_PI && !follow.exception) { 
//         // if((follow.CW && !follow.turnRight) || (!follow.CW && follow.turnRight)) {
//         //     // std::cout << "exception";
//         //     return false;
//         // }
//         return false;
//     }

//     // the exception case is also not valid because the follow point ended up on the wrong side of us
//     if(abs(angleToFollow) > M_PI && follow.exception) {
//         return false;
//     }

//     return true;
// }



// returns angle to drive along by canceling opponents velocity
void AStarAttack::driveAngle(FollowPoint &follow) {

    cv::Point2f oppVel = oppFiltered.moveVelSlow(); // opp's move velocity vector
    float oppSpeed = oppFiltered.moveSpeedSlow(); // how fast the opp is moving in absolute
    float maxCancel = orbFiltered.getMaxMoveSpeed() * 0.9f; // reserve some speed margin
    if(oppSpeed > maxCancel) { oppVel *= abs(maxCancel / oppSpeed); }

    float pointAngle = angle(cv::Point2f(orbFiltered.getPosFiltered()[0], orbFiltered.getPosFiltered()[1]), follow.point); // absolute angle to the point

    float p = oppVel.x*cos(pointAngle) + oppVel.y*sin(pointAngle);
    float s = sqrt(pow(orbFiltered.getMaxMoveSpeed(), 2) - pow(oppVel.x*sin(pointAngle) - oppVel.y*cos(pointAngle), 2));

    float pointSpeed = 0.0f;
    if(s > p) { pointSpeed = -p + s; } // thanks chat, ensures vector sum is always to maxMoveSpeed

    cv::Point2f pointVel = cv::Point2f(pointSpeed*cos(pointAngle), pointSpeed*sin(pointAngle)); // vector in the direction of the followPoint
    cv::Point2f totalVel = oppVel + pointVel; // sum the opp's vel with our desired vel in their frame

    follow.driveAngle = atan2(totalVel.y, totalVel.x); // target angle is in the direction of the total vector
}



// if we ever directly target the opp, enforce that we keep targetting them for a minimum time
void AStarAttack::commitToTarget(FollowPoint &follow, double deltaTime, float targetTime) {

    static bool enforceTarget = false; // if we're currently running the clock to force target the opp
    static float timeTargetting = 0.0f; // how long we've been force targetting the opp

    // if the follow point is basically right on the opp, turn on force targeting and reset the clock
    if(oppFiltered.distanceTo(follow.point) < oppFiltered.getSizeRadius() * 0.2f) { 
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
        follow.point = oppFiltered.position(); 
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
    float orbETA = 999999.0f;
    int bestIndex = 0;
    float bestFraction = 99999.0f;
    std::vector<float> radGains = { 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
    // radGains = { 0.7f};

    // what in the world
    for(int i = 0; i < radGains.size(); i++) {
        float testETA = orbFiltered.ETASim(follow.opp, follow.orbSimPath, false, true, 
            follow.forward, follow.CW, follow.turnAway, follow.inflectDistance, radGains[i]);

        DisplayUtils::displayPath(follow.orbSimPath, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), 1);


        float oppETA = follow.opp.turnTimeSimple(follow.orbSimPath.back(), follow.opp.getWeaponAngleReach(), true, true);

        float fraction2 = 0.0f;
        if(testETA > 0.01f) { fraction2 = std::max((testETA - oppETA) / testETA, 0.0f); }

        // std::cout << "fra = " << fraction2 << "    ";
        if(fraction2 < bestFraction || i == 0) { 
            bestIndex = i;
            bestFraction = fraction2;
        }
    }

    // do it again to write everything properly bruh
    orbETA = orbFiltered.ETASim(follow.opp, follow.orbSimPath, false, true, 
        follow.forward, follow.CW, follow.turnAway, follow.inflectDistance, radGains[bestIndex]);

    

    
    float oppETA = follow.opp.turnTimeSimple(follow.orbSimPath.back(), follow.opp.getWeaponAngleReach(), true, true);

    // save these with the follow point
    follow.orbETA = orbETA;
    follow.oppETA = oppETA;

    // AdjustRadiusWithBumpers(); // let driver adjust aggressiveness with bumpers

    // input fraction to output radius using RobotConfig variables
    std::vector<cv::Point2f> radiusCurve = {
        cv::Point2f(RADIUS_CURVE_X0, RADIUS_CURVE_Y0),
        cv::Point2f(RADIUS_CURVE_X1, RADIUS_CURVE_Y1),
        cv::Point2f(RADIUS_CURVE_X2, RADIUS_CURVE_Y2),
        cv::Point2f(RADIUS_CURVE_X3, RADIUS_CURVE_Y3)
    };


    // magic fraction, trust me bro this one works better
    float fraction2 = 0.0f;
    if(orbETA > 0.01f) { fraction2 = std::max((orbETA - oppETA) / orbETA, 0.0f); }

    // float fraction2 = 0.0f;
    // if(orbETA + oppETA > 0.01f) { fraction2 = std::max((orbETA - oppETA) / (orbETA + oppETA), 0.0f); }

    // std::cout << "best = " << fraction2 << "    ";

    follow.radius = piecewise(radiusCurve, fraction2);
    // follow.radius = std::max(follow.radius, radGains[bestIndex] * 0.9f * std::max(oppFiltered.distanceTo(orbFiltered.position()) - oppFiltered.getSizeRadius() - orbFiltered.getSizeRadius(), 0.0f));
    follow.radius = std::min(follow.radius, RADIUS_CURVE_Y3);

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

    // calculate tangent point to that circle
    float d = oppFiltered.distanceTo(orbFiltered.position());

    // tangent point doesn't exist if we're inside the circle
    if(d < follow.radius) { 
        follow.point = oppFiltered.position(); 
        return;
    }

    // epic math
    float theta = acos(follow.radius / d);
    float alpha = atan2(orbFiltered.position().y - oppFiltered.position().y, orbFiltered.position().x - oppFiltered.position().x);
    float theta2 = angle_wrap(alpha + (follow.CW? 1 : -1)*theta);

    follow.point = oppFiltered.position() + follow.radius*(cv::Point2f(cos(theta2), sin(theta2)));
}


// returns intersection of pp radius with circle of set radius around opponent in the correction direction
cv::Point2f AStarAttack::ppPoint(FollowPoint follow) {

    std::vector<cv::Point2f> circle = arcPointsFromCenter(follow.radius, 2*M_PI, 5.0f);
    transformList(circle, oppFiltered.position(), 0.0f);

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



// does a point list include this point, returns -1 if not found
int AStarAttack::vectorPointIndex(std::vector<cv::Point2f> pointList, cv::Point2f testPoint) {

    for(int i = 0; i < pointList.size(); i++) {
        if(pointList[i] == testPoint) { return i; }
    }
    return -1;
}



// spirals out from our current radius from opp to see how walls will affect us
float AStarAttack::wallScore(FollowPoint follow) {

    const float maxPathLength = 500.0f; // length of predicted path we'll sweep

    std::vector<cv::Point2f> scanPoints;
    std::vector<Line> scanLines = {};
    const float sweepIncrement = follow.CW ? 1.0f*TO_RAD : -1.0f*TO_RAD; // how much to increment sweep angle for each point

    float startAngle = angle(oppFiltered.position(), orbFiltered.position());


    float score = 0.0f; // integrated score of all the points
    float pathLength = 0.0f; // accumlated predicted path length

    // scan up to a full circle
    for (float sweptAngle = 0; abs(sweptAngle) < 2*M_PI; sweptAngle += sweepIncrement) {

        float currAngle = angle_wrap(startAngle + sweptAngle); // current angle we're scanning at

        cv::Point2f testPoint = field.clipPointInBounds(oppFiltered.position()); // point to ray cast
        constexpr int kMaxRaycastSteps = 400;


        // increment the point outwards until it's out of bounds or further than our spiral
        for(int i = 0; i < kMaxRaycastSteps; i++) {

            testPoint += 5.0f * cv::Point2f(cos(currAngle), sin(currAngle));

            // if the ray cast has exited the field
            if(!field.insideFieldBounds(testPoint)) {
                testPoint = field.closestBoundPoint(testPoint); // raycasted point might lie slightly outside, so make sure to re-clip after

                float gapSize = std::max(oppFiltered.distanceTo(testPoint) - oppFiltered.getSizeRadius(), 0.0f); // how big is the gap to drive in
                float margin = std::max(gapSize - 2*orbFiltered.getSizeRadius(), 0.01f); // how much margin would we have if we had to drive past this point
                float sweptPercent = pathLength / maxPathLength; // roughly what percentage has been swept so far

                float rangeWeight = cos(-sweptPercent * 0.5f * M_PI);

                score += pow(margin, -0.6f) + rangeWeight; // score function is integrated based on how much gap we have

                break;
            }

            float spiralRadius = abs(sweptAngle)*110.0f + oppFiltered.distanceTo(orbFiltered.position()); // slope of predicted spiral
            
            // if the point has gotten to the spiral, don't add to score
            if(oppFiltered.distanceTo(testPoint) > spiralRadius) { break; }
        }

        // total up the path length
        if(!scanPoints.empty()) {
            pathLength += cv::norm(scanPoints.back() - testPoint);
        }
        
        
        scanPoints.emplace_back(testPoint); // add to the list to display later
        scanLines.emplace_back(Line(oppFiltered.position(), testPoint));

        

        // display scanned points
        cv::Scalar color = cv::Scalar(150, 255, 150);
        if(!follow.CW) { color = cv::Scalar(150, 150, 255); }

        safe_circle(RobotController::GetInstance().GetDrawingImage(), testPoint, 2, color, 2);


        // // if we have at least 2 points start drawing lines
        // if(scanPoints.size() > 1) {
        //     cv::line(RobotController::GetInstance().GetDrawingImage(), scanPoints[scanPoints.size() - 2], testPoint, color, 2);
        // }

        

        // if the points are so far away now then stop counting
        if(pathLength > maxPathLength) { break; }
    }

    score /= pathLength; // normalize to sweep range so we can change that without losing tune
    return score;
}


// returns sign of the input
int AStarAttack::sign(float num) { return (num < 0)? -1 : 1; }


// // true if turning towards this follow point will require it to turn past the opp
// bool AStarAttack::willTurnPastOpp(FollowPoint follow) {

//     float angleToOpp = orbFiltered.angleTo(oppFiltered.position(), follow.forward);
//     float targetAngle = orbFiltered.angleTo(follow.point, follow.forward);

//     // turn direction is either what's enforced or whatever default direction
//     float turnDirection = follow.turnRight ? 1 : -1;

//     if(turnDirection == 1) { targetAngle = angleWrapRad(targetAngle - M_PI) + M_PI; }
//     if(turnDirection == -1) { targetAngle = angleWrapRad(targetAngle + M_PI) - M_PI; }

//     // std::cout << "turn direction = " << turnDirection;

//     float startMargin = orbFiltered.getTurnPastStartMargin() * turnDirection; // if we turn by the opp in the first few degrees anyway then ignore
//     float endMargin = orbFiltered.getTurnPastEndMargin() * turnDirection; // if we don't have to turn past the opp anyway (or just barely turn past) then ignore


//     // std::cout << "target = " << targetAngle << ", start = " << startMargin << ", end = " << endMargin;


//     // if the sweep range is a full circle we don't turn past opp
//     if(turnDirection * (targetAngle - startMargin - endMargin) < 0) { // don't touch
//         // std::cout << "full circle";
//         return false; } 

//     float endAngle = angleWrapRad(targetAngle - endMargin); // end of the range we'll turn through


//     // std::cout << "turn direction = " << turnDirection;



//     // float globalStart = angleWrapRad(startMargin + orbFiltered.angle(follow.forward));
//     // float globalEnd = angleWrapRad(endAngle + orbFiltered.angle(follow.forward));
//     // cv::Point2f startPoint = ppRad() * cv::Point2f(cos(globalStart), sin(globalStart)) + orbFiltered.position();
//     // cv::Point2f endPoint = ppRad() * cv::Point2f(cos(globalEnd), sin(globalEnd)) + orbFiltered.position();

//     // cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), startPoint, cv::Scalar(255, 255, 255), 2);
//     // cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), endPoint, cv::Scalar(0, 255, 255), 2);


//     // rotate everything such the start of the range is on the wrap around line
//     float rotatedEnd = angleWrapRad(endAngle - startMargin + M_PI);
//     float rotatedOpp = angleWrapRad(angleToOpp - startMargin + M_PI);

//     // we turn past the opp if it's on the turned-through side of the range
//     if(rotatedOpp*turnDirection < rotatedEnd*turnDirection) { 
//         // std::cout << " turning past opp";
//         return true; }

//     // std::cout << "not turning past opp";
//     return false;
// }



// scores how bad a point based on the curve we need to execute to switch to it
float AStarAttack::switchPointScore(FollowPoint follow) {

    // create a fake opponent at the follow point so we can use ETASim
    FilteredRobot ghostOpp = FilteredRobot();
    ghostOpp.setPos({follow.point.x, follow.point.y, 0.0f});
    ghostOpp.setSizeRadius(0.1f);

    // generate a predicted path to the follow point
    std::vector<cv::Point2f> path = {};
    float garbage = 0.0f;
    orbFiltered.ETASim(ghostOpp, path, false, false, follow.forward, follow.CW, follow.turnAway, garbage, 0.6f);


    // see how long it'll take opp to get to each point
    float worstTimeMargin = -99999999999999.0f;
    int worstIndex = 0;
    for(int i = 0; i < path.size(); i++) {

        float oppTime = (abs(oppFiltered.angleTo(path[i], true)) - oppFiltered.getWeaponAngleReach())/oppFiltered.getMaxTurnSpeed()
        + std::max(oppFiltered.distanceTo(path[i]) - oppFiltered.getSizeRadius() - orbFiltered.getSizeRadius(), 0.0f)/oppFiltered.getMaxMoveSpeed();

        // increases as opp gets there faster, positive if opp will get there first
        // float timeMargin = pathTimes[i] - oppTime; 
        float timeMargin = 0.0f;
        
        // save the point that's closest to us getting hit
        if(timeMargin > worstTimeMargin || i == 0) { 
            worstTimeMargin = timeMargin; 
            worstIndex = i;
        }
    }


    // // check to see how much this path cuts into the radius
    // float maxSqueeze = 0.0f;
    // int maxIndex = 0;
    // for(int i = 0; i < path.size(); i++) {
    //     float squeeze = std::max(follow.radius - oppFiltered.distanceTo(path[i]), 0.0f);
    //     if(squeeze > maxSqueeze) { 
    //         maxSqueeze = squeeze; 
    //         maxIndex = i;
    //     }
    // }


    // DisplayUtils::displayPoints(path, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), 1);
    // safe_circle(RobotController::GetInstance().GetDrawingImage(), path[worstIndex], 3, cv::Scalar(255, 255, 255), 2);

    // std::cout << "  worst time = " << worstTimeMargin << "    ";

    return worstTimeMargin;






    // float distanceToPoint = orbFiltered.distanceTo(follow.point);
    // float testCurveRad = std::min(25.0f, 0.5f*distanceToPoint);

    // float orbSideAngle = 90.0f*TO_RAD; if(!follow.turnRight) { orbSideAngle *= -1; }
    // float centerOffsetAngle = angleWrapRad(orbFiltered.angle(follow.forward) + orbSideAngle);

    // cv::Point2f arcCenter = orbFiltered.position() + testCurveRad*cv::Point2f(cos(centerOffsetAngle), sin(centerOffsetAngle));

    // float distance = cv::norm(arcCenter - follow.point);
    // float theta = acos(testCurveRad / distance);
    // float alpha = atan2(follow.point.y - arcCenter.y, follow.point.x - arcCenter.x);

    // int invert = follow.turnRight? -1 : 1;
    // cv::Point2f arcEnd = arcCenter + testCurveRad*cv::Point2f(cos(theta*invert + alpha), sin(theta*invert + alpha));


    // float squeezeDistance = std::max(testCurveRad + follow.radius - cv::norm(oppFiltered.position() - arcCenter), 0.0);

    // float startToOpp = oppFiltered.distanceTo(orbFiltered.position());
    // float endToOpp = oppFiltered.distanceTo(arcEnd);

    // safe_circle(RobotController::GetInstance().GetDrawingImage(), arcCenter, testCurveRad, cv::Scalar(255, 255, 255), 1);
    // cv::line(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), arcCenter, cv::Scalar(255, 255, 255), 2);
    // cv::line(RobotController::GetInstance().GetDrawingImage(), orbFiltered.position(), arcCenter, cv::Scalar(255, 255, 255), 2);
    // cv::line(RobotController::GetInstance().GetDrawingImage(), arcEnd, arcCenter, cv::Scalar(255, 255, 255), 2);


    // // std::cout << "squeeze = " << squeezeDistance << "   ";

    // return squeezeDistance;
}


// // enforces (or doesn't) a turn direction so we turn away from the op when desired
// // 1 = always turn right, -1 = always turn left
// void AStarAttack::turnAwayFromOpp(FollowPoint &follow) {

//     // if we're not gonna turn past the opp then don't enforce anything
//     if(!willTurnPastOpp(follow)) { return; }

//     // otherwise enforce a turn direction opposite from what you'd normally do
//     follow.enforceTurnDirection = -sign(orbFiltered.angleTo(follow.point, follow.forward));
// }



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



// // adjusts drive angle if needed to obey pure pursuit radius on walls
// void AStarAttack::avoidBoundsVector(FollowPoint &follow) {

//     std::vector<cv::Point2f> ppWallPoints = PurePursuit::followPath(orbFiltered.position(), field.getBoundPoints(), ppRadWall());

//     // if we're not hitting a wall and in the field don't change the drive angle
//     if(ppWallPoints.size() == 0 && field.insideFieldBounds(orbFiltered.position())) { return; }

//     cv::Point2f boundPoint = field.closestBoundPoint(orbFiltered.position()); // get the closest point to us thats on a field bound

//     // if we're not hitting a wall and fully out of the field then drive back in to closest bound point
//     if(ppWallPoints.size() == 0 && !field.insideFieldBounds(orbFiltered.position())) { 
//         safe_circle(RobotController::GetInstance().GetDrawingImage(), boundPoint, 8, cv::Scalar(255, 255, 255), 3);
//         follow.enforceTurnDirection = 0; // don't worry about turning away from opp, need to return optimally
//         follow.driveAngle = angle(orbFiltered.position(), boundPoint);
//         return;
//     }

//     // from here we're guarenteed to have pure pursuit intersections with the wall







//     // now let's find which pp points actually define the region

//     float angleToBound = angle(orbFiltered.position(), field.closestBoundPoint(orbFiltered.position())); // angle to the closest bound point

//     // min and max offsets from the line to the bound point
//     float mostPosOffset = 0;
//     float mostNegOffset = 0;

//     // find the most extreme offsets, those will be the outer ones used for pure pursuit
//     for(int i = 0; i < ppWallPoints.size(); i++) {
//         float pointAngle = angle(orbFiltered.position(), ppWallPoints[i]);
//         float offset = angleWrapRad(pointAngle - angleToBound); // offset from the line to the center of the field

//         if(offset > mostPosOffset) { mostPosOffset = offset; }
//         if(offset < mostNegOffset) { mostNegOffset = offset; }
//     }

//     // angle region in which drive angle must lie
//     float minAngle = angleWrapRad(angleToBound + mostNegOffset);
//     float maxAngle = angleWrapRad(angleToBound + mostPosOffset);


//     // points at the min and max angles
//     cv::Point2f minPoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(minAngle), sin(minAngle));
//     cv::Point2f maxPoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(maxAngle), sin(maxAngle));

//     safe_circle(RobotController::GetInstance().GetDrawingImage(), minPoint, 8, cv::Scalar(255, 255, 255), 3);
//     safe_circle(RobotController::GetInstance().GetDrawingImage(), maxPoint, 8, cv::Scalar(255, 255, 255), 3);









//     // now change the drive angle if needed

//     // intersection of pp radius and drive angle
//     cv::Point2f drivePoint = orbFiltered.position() + ppRadWall()*cv::Point2f(cos(follow.driveAngle), sin(follow.driveAngle));

//     // if drive point isn't in the field we need to change it
//     if(!field.insideFieldBounds(drivePoint)) {

//         // set it to the closest bound of the region
//         float minAngleOffset = angleWrapRad(follow.driveAngle - minAngle);
//         float maxAngleOffset = angleWrapRad(follow.driveAngle - maxAngle);

//         // set drive angle to the correct bound
//         follow.driveAngle = maxAngle;
//         if(abs(minAngleOffset) < abs(maxAngleOffset)) { follow.driveAngle = minAngle; }
//     }









//     // make sure we turn the right direction to the drive angle

//     float orbOffset = angleWrapRad(angleToBound - orbFiltered.angle(follow.forward));
//     float driveOffset = angleWrapRad(angleToBound - follow.driveAngle);

//     // only change the direction enforcement if turning the wrong way would make us go way out of the field
//     if(abs(orbOffset) > 5.0f*TO_RAD) { 

//         // stop enforcing direction bc that might make it go out
//         follow.enforceTurnDirection = 0;

//         if(orbOffset > 0 && driveOffset < 0) { follow.enforceTurnDirection = -1; }
//         if(orbOffset < 0 && driveOffset > 0) { follow.enforceTurnDirection = 1; }

//         if(!field.insideFieldBounds(orbFiltered.position())) { follow.enforceTurnDirection *= -1; } // needs to invert if out of the field
//     }

// }



// score function for determining how good a follow point is, used for CW/CCW decision
void AStarAttack::directionScore(FollowPoint &follow, bool forwardInput) {
    static GraphWidget angleToPointGraph("wallWeight*wallGain", -100, 100, "deg");


    follow.directionScores.emplace_back(follow.radius);



    // how far around the circle we have to go for each direction
    float goAroundAngle = M_PI - (follow.CW? 1 : -1)*oppFiltered.angleTo(orbFiltered.position(), true);
    float goAroundGain = 50.0f + 0.06f * oppFiltered.tangentVel(true); // 50    0.06
    follow.directionScores.emplace_back(goAroundAngle*goAroundGain*0.0f);



    // how far the robot has to turn to this point
    float slowness = std::clamp(1 - orbFiltered.moveSpeedSlow()/350, 0.0f, 1.0f);
    float facingness = abs(oppFiltered.angleTo(orbFiltered.position(), false)) / M_PI;

    float slowFacingGain = 10.0f; // 15
    float fastAwayGain = 4.0f;

    float turnGain = fastAwayGain + (slowFacingGain - fastAwayGain)*(slowness*facingness);
    turnGain = 80.0f; // 4.0
    follow.directionScores.emplace_back(turnGain * turnScore(follow));

    


    // turn past opp score
    // follow.directionScores.emplace_back(200.0f*switchPointScore(follow)*0.0f);4

    // safe_circle(RobotController::GetInstance().GetDrawingImage(), oppFiltered.position(), follow.inflectDistance + orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius(), cv::Scalar(255, 255, 255), 2);
    // std::cout << "inflect distance = " << follow.inflectDistance;
    float collideScore = 60.0f;
    float zeroScoreDistance = 90.0f;
    float inflectScore = collideScore * pow((1.0f - std::clamp(follow.inflectDistance / zeroScoreDistance, 0.0f, 1.0f)), 3.0f);
    // follow.directionScores.emplace_back(500.0f/std::max(follow.inflectDistance, 0.001f));
    follow.directionScores.emplace_back(inflectScore);



    // how close is the nearest wall in this direction
    float wallGain = 120.0f; // 200
    follow.directionScores.emplace_back(wallScore(follow)*wallGain);
       

    // how much velocity we already have built up in a given direction, ensures we don't switch to other direction randomly
    float raw = orbFiltered.tangentVel(follow.forward);
    float tanVel = pow(abs(raw), 0.5f) * sign(raw);
    float momentumWeight = -0.08f; // 1.5
    follow.directionScores.emplace_back(tanVel*momentumWeight);


    // subtract score from directions that agree with input while adding to ones that don't
    int directionAgreement = (follow.forward == forwardInput)? -1 : 1;
    follow.directionScores.emplace_back(150.0f*directionAgreement); // 170


    // sum up the scores and add it as the last entry
    float totalScore = 0.0f;
    for(int i = 0; i < follow.directionScores.size(); i++) { totalScore += follow.directionScores[i]; }
    follow.directionScores.emplace_back(totalScore);

}



// returns score based on how much we have to turn to face the follow point
float AStarAttack::turnScore(FollowPoint follow) {

    // how far we need to rotate to match the drive angle, includes enforced turn direction
    float reverseOffset = follow.forward ? 0 : M_PI;
    float angleError = angle_wrap(follow.driveAngle - orbFiltered.getPosFiltered()[2] + reverseOffset);

    bool turningCorrect = (orbFiltered.angleTo(oppFiltered.position(), follow.forward) > 0 && follow.CW) || (orbFiltered.angleTo(oppFiltered.position(), follow.forward) < 0 && !follow.CW);
    if(follow.turnAway && follow.CW && !turningCorrect) { angleError = angle_wrap(angleError - M_PI) + M_PI; }
    if(follow.turnAway && !follow.CW && !turningCorrect) { angleError = angle_wrap(angleError + M_PI) - M_PI; }

    float accel = orbFiltered.getMaxTurnAccel() * sign(angleError); // assume we accel in the direction we're going to turn
    float dirac = pow(orbFiltered.turnVelSlow(), 2.0f) + (2.0f * accel * angleError);
    float t1 = (-orbFiltered.turnVelSlow() + sqrt(dirac)) / accel;
    float t2 = (-orbFiltered.turnVelSlow() - sqrt(dirac)) / accel;

    float time = std::min(t1, t2);
    if(t1 < 0.0f) { time = t2; }
    if(t2 < 0.0f) { time = t1; }

    // std::cout << "turn time = " << time;

    return time;
}



// determines follow point when we're inside the main attack radius
void AStarAttack::followPointInsideCircle(FollowPoint &follow) {

    float direction = follow.CW? -1.0 : 1.0f;

    float angleToOrb = oppFiltered.angleTo(orbFiltered.position(), true); 
    float distanceToOpp = orbFiltered.distanceTo(oppFiltered.position());
    float collisionRadius = orbFiltered.getSizeRadius() + oppFiltered.getSizeRadius();
    

    // first define parameters at collision radius
    float minAngleFront = -10.0f*TO_RAD; // -30
    float maxAngleFront = -10.0f*TO_RAD; // 0

    float minAngleSide = -40.0f*TO_RAD; // -90
    float maxAngleSide = -40.0f*TO_RAD; // -0

    float sideAngle = -90.0f*TO_RAD;



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




    // convert to world angle
    float minWorldAngle = angle_wrap(minAngle*direction + angleToOrb + oppFiltered.angle(true));
    float maxWorldAngle = angle_wrap(maxAngle*direction + angleToOrb + oppFiltered.angle(true));



    // if the pure pursuit point might be usable, check
    if(distanceToOpp > follow.radius - ppRad()) {
        cv::Point2f ppFollow = ppPoint(follow);
        float angleToPP = angle(orbFiltered.position(), ppFollow);
        float maxAngleDifference = angle_wrap(maxWorldAngle - angleToPP);
        float minAngleDifference = angle_wrap(minWorldAngle - angleToPP);


        // if(CW) { safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 5, cv::Scalar(255, 255, 255), 5); }


        
        if(follow.CW && maxAngleDifference < 0) { maxWorldAngle = angleToPP; }
        if(follow.CW && minAngleDifference < 0) { minWorldAngle = angleToPP; }
        if(!follow.CW && maxAngleDifference > 0) { maxWorldAngle = angleToPP; }
        if(!follow.CW && minAngleDifference > 0) { minWorldAngle = angleToPP; }
    }




    // angle to follow
    float angleRange = angle_wrap(maxWorldAngle - minWorldAngle) * direction;
    float midAngle = minWorldAngle + (angleRange / 2) * direction;
    float angleError = angle_wrap(orbFiltered.angle(follow.forward) - midAngle) * direction;


    // default: assume we're in the angle range so just go forward
    // add an offset to make sure we turn in the right direction (no rounding errors)
    // float offset = follow.turnRight? 1.0f*TO_RAD : -1.0f*TO_RAD;
    float offset = 0.0f;
    float followAngle = angle_wrap(orbFiltered.angle(follow.forward) + offset);
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


// returns the best follow point with the lowest score
FollowPoint AStarAttack::chooseBestPoint(std::vector<FollowPoint>& follows, bool forwardInput) {

    if(follows.size() == 0) { return FollowPoint(); } // no crashy

    // find follow point with best score
    int bestIndex = 0;
    directionScore(follows[0], forwardInput);
    float bestScore = follows[0].directionScores.back();

    for(int i = 1; i < follows.size(); i++) {

        directionScore(follows[i], forwardInput);
        float newScore = follows[i].directionScores.back(); // last entry is total score

        if(newScore < bestScore) {
            bestScore = newScore;
            bestIndex = i;
        }
    }

    return follows[bestIndex]; // return best index
}



// Field boundary editing interface implementation
AStarAttack* AStarAttack::GetInstance() { return _instance; }

const std::vector<FollowPoint>& AStarAttack::GetFollowPoints() const {
    return _lastFollowPoints;
}

// sets field lines to what the UI changed them to
void AStarAttack::SetFieldBoundaryPoints(const std::vector<cv::Point2f>& points) { field.setBoundPoints(points); }

// gets field points for UI
std::vector<cv::Point2f>& AStarAttack::GetFieldBoundaryPoints() { return field.getBoundPoints(); }



// Radius curve parameter interface implementation
void AStarAttack::GetRadiusCurvePoints(float radiusCurveX[4], float radiusCurveY[4]) {
    radiusCurveX[0] = RADIUS_CURVE_X0;
    radiusCurveX[1] = RADIUS_CURVE_X1;
    radiusCurveX[2] = RADIUS_CURVE_X2;
    radiusCurveX[3] = RADIUS_CURVE_X3;
    radiusCurveY[0] = RADIUS_CURVE_Y0;
    radiusCurveY[1] = RADIUS_CURVE_Y1;
    radiusCurveY[2] = RADIUS_CURVE_Y2;
    radiusCurveY[3] = RADIUS_CURVE_Y3;
}



void AStarAttack::SetRadiusCurvePoints(const float radiusCurveX[4], const float radiusCurveY[4]) {
    RADIUS_CURVE_X0 = radiusCurveX[0];
    RADIUS_CURVE_X1 = radiusCurveX[1];
    RADIUS_CURVE_X2 = radiusCurveX[2];
    RADIUS_CURVE_X3 = radiusCurveX[3];
    RADIUS_CURVE_Y0 = radiusCurveY[0];
    RADIUS_CURVE_Y1 = radiusCurveY[1];
    RADIUS_CURVE_Y2 = radiusCurveY[2];
    RADIUS_CURVE_Y3 = radiusCurveY[3];
}



void AStarAttack::ResetRadiusCurveToDefault() {
    RADIUS_CURVE_X0 = 0.0f;   RADIUS_CURVE_Y0 = 0.0f;
    RADIUS_CURVE_X1 = 15.0f;  RADIUS_CURVE_Y1 = 85.0f;
    RADIUS_CURVE_X2 = 100.0f; RADIUS_CURVE_Y2 = 150.0f;
    RADIUS_CURVE_X3 = 200.0f; RADIUS_CURVE_Y3 = 150.0f;
}



void AStarAttack::ResetFieldBoundariesToDefault() {
    field.resetBoundsToDefault();
}



// forces field to regenerate bound lines
void AStarAttack::RegenerateFieldBoundaryLines() { field.generateBoundLines(); }
