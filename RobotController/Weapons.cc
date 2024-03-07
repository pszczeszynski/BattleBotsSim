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


#define IDLE_RPM 6500
void Weapons::UpdateSpinnerPowers()
{
    static WeaponState frontWeaponState = WeaponState::OFF;
    static WeaponState backWeaponState = WeaponState::OFF;

    Gamepad& gamepad = RobotController::GetInstance().GetGamepad();
    Gamepad& gamepad2 = RobotController::GetInstance().GetGamepad2();

    // get current RPM of each weapon
    float frontRPM = GetFrontWeaponRPM();
    float backRPM = GetBackWeaponRPM();

    // if a pressed (OR dpad up pressed on second controller)
    if (gamepad.GetButtonA() || gamepad2.GetDpadUp())
    {
        backWeaponState = WeaponState::ON;
    }

    // if b pressed (OR dpad down pressed on second controller)
    if (gamepad.GetButtonB() || gamepad2.GetDpadDown())
    {
        backWeaponState = WeaponState::OFF;
    }

    // if x pressed (OR y pressed on second controller)
    if (gamepad.GetButtonX() || gamepad2.GetButtonY())
    {
        frontWeaponState = WeaponState::ON;
    }

    // if y pressed (OR a pressed on second controller)
    if (gamepad.GetButtonY() || gamepad2.GetButtonA())
    {
        frontWeaponState = WeaponState::OFF;
    }


    // A and B on the second controller idle the front weapon
    if (gamepad2.GetButtonB() || gamepad2.GetButtonX())
    {
        frontWeaponState = WeaponState::IDLE;
    }

    // Dpad left and right on the second controller idle the back weapon
    if (gamepad2.GetDpadLeft() || gamepad2.GetDpadRight())
    {
        backWeaponState = WeaponState::IDLE;
    }



    if (frontWeaponState == WeaponState::ON)
    {
        _frontWeaponPower = 1.0;
    }
    else if (frontWeaponState == WeaponState::OFF)
    {
        _frontWeaponPower = 0.0;
    }
    else if (frontWeaponState == WeaponState::IDLE)
    {
        // check if below idle RPM
        if (frontRPM < IDLE_RPM * 1.1)
        {
            _frontWeaponPower = 0.5;
        }
        else
        {
            _frontWeaponPower = 0.0;
        }
    }


    if (backWeaponState == WeaponState::ON)
    {
        _backWeaponPower = 1.0;
    }
    else if (backWeaponState == WeaponState::OFF)
    {
        _backWeaponPower = 0.0;
    }
    else if (backWeaponState == WeaponState::IDLE)
    {
        // check if below idle RPM
        if (backRPM < IDLE_RPM * 1.1)
        {
            _backWeaponPower = 0.5;
        }
        else
        {
            _backWeaponPower = 0.0;
        }
    }
}
