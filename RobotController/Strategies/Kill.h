#pragma once

#include "Strategy.h"
#include "../../Communication/Communication.h"
#include "../Input/Gamepad.h"
#include "../UIWidgets/KillWidget.h"

class Kill : public Strategy
{
public:
    Kill();

    virtual DriveCommand Execute(Gamepad& gamepad) override;

};