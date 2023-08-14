#pragma once
#define CAN_RATE 500000 // must match vec set can rate
#include <FlexCAN_T4.h>

class VESC
{
private:
    enum MotorIndexes
    {
        l_drive = 0,
        r_drive,
        f_weapon,
        b_weapon
    };

    static void OnMessage(const CAN_message_t &msg);
    int ids[4];
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN1;
    // Voltage, Duty Cycle, RPM, AH, FET TEMP, MOTOR TEMP, CURRENT
    float volts[4];
    float duty_cycle[4];
    int rpm[4];
    float ah[4];
    float fet_temp[4];
    float motor_temp[4];
    float current[4];

    void _SetMotorPower(float power, int motorIndex);
public:
    VESC(int l_drive_id, int r_drive_id, int f_weapon_id, int b_weapon_id);

    void Drive(float leftPower, float rightPower);
    void DriveWeapons(float frontPower, float backPower);
    void Update();  
};
