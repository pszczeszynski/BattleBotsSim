#pragma once
#include <FlexCAN_T4.h>
#include "Communication.h"
#define CAN_RATE 500000 // must match vec set can rate

class VESC
{
private:
    float lastPowers[4] = { 0, 0, 0, 0 };
    static void OnMessage(const CAN_message_t &msg);
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN1;

    void _SetMotorPower(float power, int motorIndex);
public:
    VESC(int l_drive_id, int r_drive_id, int f_weapon_id, int b_weapon_id);

    void Drive(float leftPower, float rightPower);
    void DriveWeapons(float frontPower, float backPower);
    void Update();
    
    void GetCurrents(unsigned char* outCurrents);
    void GetVolts(unsigned char* outVolts);
    void GetRPMs(unsigned char* outRPMs);
    void GetFETTemps(unsigned char* outFetTemps);
};