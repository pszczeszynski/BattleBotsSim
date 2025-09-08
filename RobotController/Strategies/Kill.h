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

    FilteredRobot orbFiltered; // orb robot
    FilteredRobot oppFiltered; // opp robot
    FilteredRobot oppExtrap; // opp robot extrapolated into future


    void displayPathPoints(std::vector<cv::Point2f>& path, cv::Scalar color);
};