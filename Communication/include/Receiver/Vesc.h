#pragma once
#include "Communication.h"
#include "Receiver/CANBUS.h"
#include <FlexCAN_T4.h>
#define CAN_RATE 500000 // must match vec set can rate

class VESC {
private:
  float lastPowers[5] = {0, 0, 0, 0, 0};

  void _SetMotorPower(float power, int motorIndex);
  void _SetMotorCurrent(float current_amps, int motorIndex);
  CANBUS *_can;

public:
  static void OnMessage(const CAN_message_t &msg);
  VESC(CANBUS *can, int l_drive_id, int r_drive_id, int f_weapon_id,
       int b_weapon_id, int self_righter_id);

  void Drive(float leftPower, float rightPower, float selfRighterPower,
             bool isDutyCycleControl);
  void DriveWeapons(float frontCurrent_amps, float backCurrent_amps);

  void GetCurrents(unsigned char *outCurrents);
  void GetVolts(unsigned char *outVolts);
  void GetRPMs(unsigned char *outRPMs);
  void GetFETTemps(unsigned char *outFetTemps);
  void GetMotorTemps(unsigned char *outMotorTemps);

  void GetFloatCurrents(float *outCurrents);
  void GetFloatVolts(float *outVolts);
  void GetIntRPMs(int *outRPMs);
  void GetFloatFETTemps(float *outFetTemps);
  void GetFloatMotorTemps(float *outMotorTemps);
  void GetFloatDutyCycle(float *outDutyCycle);
};
