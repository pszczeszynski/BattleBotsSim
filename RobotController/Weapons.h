#pragma once

#define MAX_WEAPON_RPM 6200

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
    float _frontWeaponPower = 0;
    float _backWeaponPower = 0;
};
