#pragma once

#include "Strategy.h"
#include "../../Common/Communication.h"
#include "../Input/Gamepad.h"
#include "../UIWidgets/KillWidget.h"

class Kill : public Strategy
{
public:
    Kill();

    virtual DriverStationMessage Execute(Gamepad& gamepad) override;
};