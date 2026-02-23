#pragma once

#include "Strategy.h"
#include "../../Common/Communication.h"
#include "../Input/Gamepad.h"
#include <opencv2/core.hpp>
#include "FilteredRobot.h"
#include "Field.h"

class Kill : public Strategy
{
public:
    Kill();

    virtual DriverStationMessage Execute(Gamepad& gamepad) override;


private:

    FilteredRobot orbFiltered; // orb robot
    FilteredRobot oppFiltered; // opp robot

    Field field; // field object


    float orbTimeToPoint(cv::Point2f point, std::vector<cv::Point2f> &orbSimPath, bool forward);
    float angle(cv::Point2f point1, cv::Point2f point2);
    cv::Point2f extrapOpp(std::vector<cv::Point2f> &oppSimPath, std::vector<cv::Point2f> &orbSimPath, bool forward);

};