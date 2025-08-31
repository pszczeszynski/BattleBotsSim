#pragma once

#include <opencv2/core.hpp>
#include "FilteredRobot.h"
#include <vector>


class FieldState
{
 public:

    FieldState();
    FieldState(FilteredRobot orb, FilteredRobot opp, float simTime, float timeStep, float maxSimTime, int numEvo);

    float metric();
    float evolve();
    float chooseInput();

    int sign(float value);




    float calculateChildTurn(int i);


 private:

    FilteredRobot orbFiltered;
    FilteredRobot oppFiltered;

    float simTime; // time state since the original simulation began
    float timeStep; // how long is a simulation time step
    float maxSimTime; // time horizon

    int numEvo; // how many child states to generate


    std::vector<FilteredRobot> extrapRobots(float orbTurn);
    float calculateMaxCurve();

    
};