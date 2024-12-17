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
    tiles {20},
    fieldMax {720},
    fieldPad {30},
    oppWeapon {oppWeaponRegion},
    pathSolver {tiles, tiles}
{
    // define shelf rectangle
    float shelfWidth = 280;
    float shelfHeight = 170;

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
}

DriverStationMessage AStarAttack::Execute(Gamepad &gamepad)
{
    RobotMovement::DriveDirection direction = LEAD_WITH_BAR ? RobotMovement::DriveDirection::Forward : RobotMovement::DriveDirection::Backward;

    // extrapolate our position into the future
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();
    ourData.Extrapolate(Clock::programClock.getElapsedTime() + (POSITION_EXTRAPOLATE_MS / 1000.0));

    // extrapolate the opponent's position into the future
    //OdometryData opponentData = ExtrapolateOpponentPos(OPPONENT_POSITION_EXTRAPOLATE_MS_KILL / 1000.0, MAX_OPP_EXTRAP_MS_KILL);
    OdometryData opponentData = ExtrapolateOpponentPos(0, MAX_OPP_EXTRAP_MS_KILL);

    // draw a crosshair on the opponent
    safe_circle(RobotController::GetInstance().GetDrawingImage(), opponentData.robotPosition, 10, cv::Scalar(0, 0, 255), 2);




    cv::Point oppGrid = toGrid(opponentData.robotPosition);
    cv::Point orbGrid = toGrid(ourData.robotPosition);


    //std::cout << "Opp x = " << oppNorm.x << ", Opp y = " << oppNorm.y << std::endl;
    //std::cout << "OppNorm x = " << oppGrid.x << ", OppNorm y = " << oppGrid.y << ", OppAngle = " << opponentData.robotAngle << std::endl;


    // set start and goal and generate path
    pathSolver.setStartGoal(orbGrid, oppGrid);


    
    // find all the obstacle points, just opponent and shelf
    std::vector<cv::Point> obstaclePoints = oppWeapon.globalPoints(oppGrid, (float) -opponentData.robotAngle);
    

    pathSolver.setWeaponPoints(obstaclePoints);
    std::vector<cv::Point> path = pathSolver.generatePath();

    displayPath(path);
    displayBoundaryPoints();
    //field.display(path);


    cv::Point followPoint(0, 0);

    if(path.size() > 1) {
        followPoint = toField(path[1]);
    }


    //followPoint = cv::Point(0,0);
    //std::cout << "follow point = " << followPoint << std::endl;

    

    

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

    // reset field
    //field.resetRegion(oppWeapon.globalTiles(oppData));

    return ret;
}

// converts a field float point to a grid point
cv::Point AStarAttack::toGrid(cv::Point2f position) {
    float xPercent = (position.x - fieldPad) / (fieldMax - 2*fieldPad);
    float yPercent = ((fieldMax - position.y) - fieldPad) / (fieldMax - 2*fieldPad); // inverted y scale
    xPercent = std::min(xPercent, 1.0f); xPercent = std::max(xPercent, 0.0f);
    yPercent = std::min(yPercent, 1.0f); yPercent = std::max(yPercent, 0.0f);

    // return point rounded to nearest tile
    return cv::Point(round(xPercent * (tiles-1)), round(yPercent * (tiles-1)));
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
void AStarAttack::displayPath(std::vector<cv::Point>& path) {
    //std::cout << "Path = ";
    for (int i = 0; i < path.size(); i++) {
        //std::cout << "(" << path[i].x << ", " << path[i].y << "), ";
        safe_circle(RobotController::GetInstance().GetDrawingImage(), toField(path[i]), 10, cv::Scalar(0, 255, 0), 2);

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

