#pragma once

#include "Strategy.h"
#include "../Input/Gamepad.h"
#include "Field.h"
#include "ScoreRegion.h"
#include "AStar.h"
#include "../PurePursuit.h"
#include <algorithm>

class AStarAttack : public Strategy
{
public:
    AStarAttack();

    virtual DriverStationMessage Execute(Gamepad& gamepad) override;


private:

    // field grid
    int tiles;
    float fieldMax; // maximum coordinate of a robot on the field
    float fieldPad; // how far in the grid starts
    
    // opponent wepaon grid stuff
    std::vector<std::vector<float>> oppWeaponRegion = {
        {0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0},
        {0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0},
        {0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0}, 
        {0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };
    ScoreRegion oppWeapon;

    // AStar solver
    AStar pathSolver;



    cv::Point toGrid(cv::Point2f POSITION_EXTRAPOLATE_MS);
    cv::Point2f toField(cv::Point position);
    void displayPath(std::vector<cv::Point>& path);
    void displayBoundaryPoints();

};