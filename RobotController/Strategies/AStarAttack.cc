#include "AStarAttack.h"
#include "../RobotOdometry.h"
#include "RobotMovement.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "Extrapolate.h"
#include "../SafeDrawing.h"
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

AStarAttack::AStarAttack() :
    tiles {21},
    fieldMax {720},
    fieldPad {20},
    pathSolver {tiles, tiles}
{
    // define shelf rectangle
    float shelfWidth = 300;
    float shelfHeight = 200;

    // find grid points for the shelf corners, y from top
    cv::Point shelfBottomLeft = toGrid(cv::Point2f(fieldMax/2 - shelfWidth/2, shelfHeight));
    cv::Point shelfBottomRight = toGrid(cv::Point2f(fieldMax/2 + shelfWidth/2, shelfHeight));

    // list of all boundary points
    std::vector<cv::Point> boundaryPoints;

    // generate outer rectangle
    for (int y = 0; y < tiles; y++) {
        for (int x = 0; x < tiles; x++) {
            
            // top row is filled in fully besides where the shelf is
            if (y == tiles - 1 && (x < shelfBottomLeft.x || x > shelfBottomRight.x)) {
                boundaryPoints.emplace_back(cv::Point(x, y));
            }
            // bottom row is fully filled in
            else if (y == 0) {
                boundaryPoints.emplace_back(cv::Point(x, y));
            }
            // otherwise just the first and last
            else if (x == 0 || x == tiles - 1) {
                boundaryPoints.emplace_back(cv::Point(x, y));
            }
        }
    }

    // generate shelf outline
    for (int y = shelfBottomLeft.y; y < tiles; y++) {
        for (int x = shelfBottomLeft.x; x <= shelfBottomRight.x; x++) {

            // bottom is filled in
            if(y == shelfBottomLeft.y) {
                boundaryPoints.emplace_back(cv::Point(x, y));
            }
            else if (x == shelfBottomLeft.x || x == shelfBottomRight.x) {
                boundaryPoints.emplace_back(cv::Point(x, y));
            }
        }
    }
    pathSolver.setBoundaryPoints(boundaryPoints);
    orbMoveVelFilter = 0;
    oppMoveVelFilter = 0;
    oppTurnVelFilter = 0;

}

DriverStationMessage AStarAttack::Execute(Gamepad &gamepad)
{
    RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    // extrapolate our position into the future
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();
    //std::cout << "our vel x = " << ourData.robotVelocity.x << std::endl;
    ourData.Extrapolate(Clock::programClock.getElapsedTime() + (POSITION_EXTRAPOLATE_MS / 1000.0));

    // extrapolate the opponent's position into the future
    //OdometryData opponentData = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS_KILL / 1000.0, MAX_OPP_EXTRAP_MS_KILL);
    OdometryData opponentData = ExtrapolateOpponentPos(0 / 1000.0, MAX_OPP_EXTRAP_MS_KILL);
    opponentData =RobotController::GetInstance().odometry.Opponent();

    // draw a crosshair on the opponent
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                opponentData.robotPosition,
                10, cv::Scalar(0, 0, 255), 2);


    oppMoveVelFilter += 0.3f * (cv::norm(opponentData.robotVelocity) - oppMoveVelFilter);
    oppTurnVelFilter += 0.1f * (opponentData.robotAngleVelocity - oppTurnVelFilter);
    orbMoveVelFilter += 0.3f * (cv::norm(ourData.robotVelocity) - orbMoveVelFilter);

    //std::cout << "Turn filter = " << oppTurnVelFilter << std::endl;
    
    // set start and goal and generate path
    pathSolver.setStartParams(toGrid(ourData.robotPosition),
                                std::max(orbMoveVelFilter * 1.0f * (tiles / (fieldMax - 2*fieldPad)), 0.2f*tiles),
                              toGrid(opponentData.robotPosition),
                              -opponentData.robotAngle,
                              std::min(oppMoveVelFilter * (tiles / (fieldMax - 2*fieldPad)), 99.0f),
                              std::min(std::max(-oppTurnVelFilter, -99.0f), 99.0f));

                        


    std::vector<cv::Point> path = pathSolver.generatePath(-ourData.robotAngle);

    // if the path is empty or just 1 point, just use our position and opponent
    if(path.size() < 2) {
        path = {};
        cv::Point2f ourPosition = toGrid(ourData.robotPosition);
        cv::Point2f oppPosition = toGrid(opponentData.robotPosition);
        path.emplace_back(cv::Point(round(ourPosition.x), round(ourPosition.y)));
        path.emplace_back(cv::Point(round(oppPosition.x), round(oppPosition.y)));
    }



    // convert to a float path
    std::vector<cv::Point2f> pathF = {};
    for (int i = 0; i < path.size(); i++) {
        cv::Point2f fieldPoint = toField(path[i]);

        // sub in exact start and end locations
        if (i == 0) {
            fieldPoint = ourData.robotPosition;
        }

        /**
        if (i == path.size() - 1) {
            fieldPoint = opponentData.robotPosition;
        }
        */

        pathF.emplace_back(fieldPoint);
    }


    cv::Point followPoint(0, 0);



    displayBoundaryPoints();
    displayPathPointsDirect(pathF);
    displayPathLinesDirect(pathF);
    displayOppWeapon();



    float radius = std::min(60 + 0.05*orbMoveVelFilter, cv::norm(ourData.robotPosition - opponentData.robotPosition) - 20.0f);
    std::vector<cv::Point2f> circleIntersections = PurePursuit::followPath(ourData.robotPosition, pathF, radius);

    if(circleIntersections.size() == 0) {
        circleIntersections.emplace_back(opponentData.robotPosition);
    }


    // draw pure pursuit follow point
    safe_circle(RobotController::GetInstance().GetDrawingImage(), circleIntersections[0], 5, cv::Scalar(255, 0, 0), 2);

    // draw pure pursuit radius
    safe_circle(RobotController::GetInstance().GetDrawingImage(),
                ourData.robotPosition,
                radius, cv::Scalar(255, 0, 0), 2);

    

    followPoint = circleIntersections[0];
    

    // hold angle to the opponent
    DriverStationMessage ret = RobotMovement::HoldAngle(ourData.robotPosition,
                                                        followPoint,
                                                        KILL_KD_PERCENT,
                                                        TURN_THRESH_1_DEG_KILL,
                                                        TURN_THRESH_2_DEG_KILL,
                                                        MAX_TURN_POWER_PERCENT_KILL,
                                                        MIN_TURN_POWER_PERCENT_KILL,
                                                        SCALE_DOWN_MOVEMENT_PERCENT_KILL,
                                                        direction);

    float movement = gamepad.GetRightStickY();
    ret.autoDrive.movement = movement;

    // reset field
    //field.resetRegion(oppWeapon.globalTiles(oppData));

    return ret;
}

// converts a field float point to a grid point, no rounding
cv::Point2f AStarAttack::toGrid(cv::Point2f position) {
    float xPercent = (position.x - fieldPad) / (fieldMax - 2*fieldPad);
    float yPercent = ((fieldMax - position.y) - fieldPad) / (fieldMax - 2*fieldPad); // inverted y scale
    xPercent = std::min(xPercent, 1.0f); xPercent = std::max(xPercent, 0.0f);
    yPercent = std::min(yPercent, 1.0f); yPercent = std::max(yPercent, 0.0f);

    // return point rounded to nearest tile
    return cv::Point2f(xPercent * (tiles-1), yPercent * (tiles-1));
}

// converts a grid point to field point
cv::Point2f AStarAttack::toField(cv::Point position) {
    float xPercent = (float) position.x / (tiles - 1);
    float yPercent = (float) position.y / (tiles - 1);
    float xPos = (xPercent * (fieldMax - 2*fieldPad)) + fieldPad;
    float yPos = ((1.0f - yPercent) * (fieldMax - 2*fieldPad)) + fieldPad;
    return cv::Point2f(xPos, yPos);
}

// displays the path
void AStarAttack::displayPathPoints(std::vector<cv::Point>& path) {
    //std::cout << "Path = ";
    for (int i = 0; i < path.size(); i++) {
        //stsd::cout << "(" << path[i].x << ", " << path[i].y << "), ";
        safe_circle(RobotController::GetInstance().GetDrawingImage(), toField(path[i]), 5, cv::Scalar(0, 255, 0), 2);

    }
    //std::cout << std::endl;
}

// displays the boundary points from AStar
void AStarAttack::displayBoundaryPoints() {
    std::vector<cv::Point> points = pathSolver.getBoundaryPoints();
    for (int i = 0; i < points.size(); i++) {
        safe_circle(RobotController::GetInstance().GetDrawingImage(), toField(points[i]), 10, cv::Scalar(0, 0, 255), 2);
    }
}


// displays the path with nice lines oooo
void AStarAttack::displayPathLines(std::vector<cv::Point>& path) {
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        double progress = (double)i / path.size();
        //cv::Scalar color = true ? cv::Scalar(0, progress * 255, (1.0 - progress) * 255) : cv::Scalar(progress * 255, (1.0 - progress) * 255, 0);
        cv::Scalar color = true ? cv::Scalar(0, 255, 0) : cv::Scalar(progress * 255, (1.0 - progress) * 255, 0);
        cv::line(RobotController::GetInstance().GetDrawingImage(), toField(path[i]), toField(path[i + 1]), color, 2);
    }
}


void AStarAttack::displayPathPointsDirect(std::vector<cv::Point2f>& path) {
    for (int i = 0; i < path.size(); i++) {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        safe_circle(RobotController::GetInstance().GetDrawingImage(), centerInt, 5, cv::Scalar(0, 255, 0), 2);
    }
}
void AStarAttack::displayPathLinesDirect(std::vector<cv::Point2f>& path) {
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        cv::Point centerInt(round(path[i].x), round(path[i].y));
        cv::Point centerInt2(round(path[i + 1].x), round(path[i + 1].y));
        double progress = (double)i / path.size();
        cv::line(RobotController::GetInstance().GetDrawingImage(), centerInt, centerInt2, cv::Scalar(0, 255, 0), 2);
    }
}

// displays the field points that are weapon
void AStarAttack::displayOppWeapon() {
    std::vector<cv::Point> points = pathSolver.getWeaponPoints();
    for (int i = 0; i < points.size(); i++) {
        safe_circle(RobotController::GetInstance().GetDrawingImage(), toField(points[i]), 10, cv::Scalar(0, 165, 255), 2);
    }
}


