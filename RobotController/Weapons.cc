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


#define SECONDS_UNTIL_FULL_POWER 0.01
void Weapons::UpdateSpinnerPowers()
{
    static bool weaponOnF = false;
    static bool weaponOnB = false;

    Gamepad& gamepad = RobotController::GetInstance().GetGamepad();

    // if a pressed
    if (gamepad.GetButtonA())
    {
        weaponOnB = true;
    }

    // if b pressed
    if (gamepad.GetButtonB())
    {
        weaponOnB = false;
    }

    // if x pressed
    if (gamepad.GetButtonX())
    {
        weaponOnF = true;
    }

    // if y pressed
    if (gamepad.GetButtonY())
    {
        weaponOnF = false;
    }


    _frontWeaponPower = (weaponOnF ? 1 : 0);
    _backWeaponPower = (weaponOnB ? 1 : 0);
}
