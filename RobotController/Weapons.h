#pragma once

#define MAX_WEAPON_RPM 13000


class Weapons
{
public:
    static Weapons& GetInstance();

    float GetFrontWeaponTargetPower();
    float GetBackWeaponTargetPower();
    float GetFrontWeaponRPM();
    float GetBackWeaponRPM();
    void UpdateSpinnerPowers();

private:
    enum class WeaponState
    {
        OFF = 0,
        IDLE,
        ON
    };


    float _frontWeaponPower = 0;
    float _backWeaponPower = 0;

    bool _isIdlingFront = false;
    bool _isIdlingBack = false;
};
