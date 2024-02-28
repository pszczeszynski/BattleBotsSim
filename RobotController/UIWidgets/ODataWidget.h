#pragma once
#include "imgui.h"
#include <vector>
#include <string>
#include "../Odometry/OdometryBase.h"

class ODataWidget
{
public:

    void Draw();

private:
    OdometryData currRobot;
    OdometryData currOpponent;

};