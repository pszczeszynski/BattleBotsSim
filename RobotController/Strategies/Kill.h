#pragma once

#include "Strategy.h"
#include "../../Common/Communication.h"
#include "../Input/Gamepad.h"
#include <opencv2/core.hpp>
#include "FilteredRobot.h"

class Kill : public Strategy
{
public:
    Kill();

    virtual DriverStationMessage Execute(Gamepad& gamepad) override;


private:

    // filtered data for orb and opp
    FilteredRobot orbFiltered;
    FilteredRobot oppFiltered;




    std::vector<float> curvatureController(cv::Point2f followPoint, float moveSpeed, float deltaTime, int turnDirection, bool forward);
    cv::Point2f collidePoint(bool forward);
    int sign(float num);
};