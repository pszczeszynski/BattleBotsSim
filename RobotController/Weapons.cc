#include "Weapons.h"
#include "Clock.h"
#include "Input/Gamepad.h"
#include "RobotController.h"
#include "imgui.h"
#include "Input/InputState.h"

Weapons& Weapons::GetInstance()
{
    static Weapons instance;
    return instance;
}

float Weapons::GetFrontWeaponTargetPower()
{
    return _frontWeaponPower;
}

float Weapons::GetBackWeaponTargetPower()
{
    return _backWeaponPower;
}

float Weapons::GetFrontWeaponRPM()
{
    CANData lastCANData = RobotController::GetInstance().GetCANData();
    return lastCANData.motorERPM[2] * ERPM_FIELD_SCALAR * ERPM_TO_RPM;
}

float Weapons::GetBackWeaponRPM()
{
    CANData lastCANData = RobotController::GetInstance().GetCANData();
    return lastCANData.motorERPM[3] * ERPM_FIELD_SCALAR * ERPM_TO_RPM;
}


#define SECONDS_UNTIL_FULL_POWER 15.6
void Weapons::UpdateSpinnerPowers()
{
    static Clock updateTimer;
    static bool rampUpF = false;
    static bool rampUpB = false;
    static bool overrideRampLimit = false;

    Gamepad& gamepad = RobotController::GetInstance().GetGamepad();
    CANData lastCANData = RobotController::GetInstance().GetCANData();

    // get the delta time
    double deltaTimeS = updateTimer.getElapsedTime();
    // mark the start of the update
    updateTimer.markStart();
    // reset the delta time if it is too large
    if (deltaTimeS > 10)
    {
        deltaTimeS = 0;
    }

    double scaleFront = _frontWeaponPower < 0.1 ? 1 : (_frontWeaponPower < 0.55 ? 2.2 : 1.2);
    double scaleBack = _backWeaponPower < 0.1 ? 1 : (_backWeaponPower < 0.55 ? 2.2 : 1.2);

    // if a pressed
    if (gamepad.GetButtonA() || InputState::GetInstance().IsKeyDown(ImGuiKey_W))
    {
        rampUpB = true;
    }

    // if b pressed
    if (gamepad.GetButtonB() || InputState::GetInstance().IsKeyDown(ImGuiKey_S))
    {
        rampUpB = false;
    }

    // if x pressed
    if (gamepad.GetButtonX() || InputState::GetInstance().IsKeyDown(ImGuiKey_I))
    {
        rampUpF = true;
    }

    // if y pressed
    if (gamepad.GetButtonY() || InputState::GetInstance().IsKeyDown(ImGuiKey_K))
    {
        rampUpF = false;
    }

    // if user toggles off the weapon, then set the power to 0
    if (InputState::GetInstance().IsKeyDown(ImGuiKey_9))
    {
        if (rampUpF)
        {
            _frontWeaponPower = std::min(_frontWeaponPower, 0.55f);
        }
        else
        {
            _frontWeaponPower = 0;
        }
    }

    // if user toggles off the weapon, then set the power to 0
    if (InputState::GetInstance().IsKeyDown(ImGuiKey_0))
    {
        if (rampUpF)
        {
            _backWeaponPower = std::min(_backWeaponPower, 0.55f);
        }
        else
        {
            _backWeaponPower = 0;
        }
    }

    if (InputState::GetInstance().IsKeyDown(ImGuiKey_O))
    {
        overrideRampLimit = true;
    }

    if (InputState::GetInstance().IsKeyDown(ImGuiKey_L))
    {
        overrideRampLimit = false;
    }

#ifndef SIMULATION
    // if we are ramping up and past 50% power with no current (because of a
    // fault), then keep it at 50%
    if (rampUpF)
    {
        bool hasNoCurrent = lastCANData.motorCurrent[2] == 0.0f;

        // if we have no current and we are ramping up, back off to the current rpm percent (mins a bit)
        if (hasNoCurrent && !overrideRampLimit && _frontWeaponPower > 0.5)
        {
            _frontWeaponPower = std::max(0.5, frontWeaponCurrRPMPercent - 0.02);
        }
    }

    // same thing for the other one
    if (rampUpB)
    {
        bool hasNoCurrent = lastCANData.motorCurrent[3] == 0.0f;

        // if we have no current and we are ramping up, back off to the current
        // rpm percent (minus a bit)
        if (hasNoCurrent && !overrideRampLimit && _backWeaponPower > 0.5)
        {
            _backWeaponPower = std::max(0.5, backWeaponCurrRPMPercent - 0.02);
        }
    }
#endif

    _frontWeaponPower += (rampUpF ? 1 : -1) * scaleFront * deltaTimeS / SECONDS_UNTIL_FULL_POWER;
    _backWeaponPower += (rampUpB ? 1 : -1) * scaleBack * deltaTimeS / SECONDS_UNTIL_FULL_POWER;

    // force weapon powers to be between 0 and 1
    _frontWeaponPower = std::max(0.0f, std::min(1.0f, _frontWeaponPower));
    _backWeaponPower = std::max(0.0f, std::min(1.0f, _backWeaponPower));
}
