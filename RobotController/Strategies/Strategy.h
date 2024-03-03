#pragma once
#include "../../Communication/Communication.h"
#include "../Input/Gamepad.h"

/*
 Base class for all strategies
*/

class Strategy
{
public:
    virtual DriverStationMessage Execute(Gamepad& gamepad) = 0;
};
