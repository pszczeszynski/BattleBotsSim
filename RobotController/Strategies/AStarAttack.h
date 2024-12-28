#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
#include "ScoreRegion.h"
#include "AStar.h"
#include "../PurePursuit.h"
#include <algorithm>
#include <cmath>
#include "Line.h"

class AStarAttack : public Strategy
{
public:
    AStarAttack();

    virtual DriverStationMessage Execute(Gamepad& gamepad) override;


private:

    int tiles; // how many tiles in the AStar grid
    float fieldMax; // maximum coordinate of a robot on the field
    float fieldPad; // how far in the grid starts

    AStar pathSolver; // AStar solver

    float oppMoveVelFilter = 0.0f;
    float oppTurnVelFilter = 0.0f;
    float orbMoveVelFilter = 0.0f;

    cv::Point followPoint; // pure pursuit follow point




    cv::Point2f toGrid(cv::Point2f position);
    cv::Point2f toField(cv::Point2f position);
    void displayPathPoints(std::vector<cv::Point>& path);
    void displayPathLines(std::vector<cv::Point>& path);
    void displayPathPointsDirect(std::vector<cv::Point2f>& path);
    void displayPathLinesDirect(std::vector<cv::Point2f>& path);
    void displayBoundaryLines();
    void displayOppWeapon();

};