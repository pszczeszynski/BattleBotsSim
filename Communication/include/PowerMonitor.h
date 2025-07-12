#pragma once
#include "Hardware.h"
#include <Arduino.h>

class PowerMonitor {
public:
  PowerMonitor() {}
  void readSensors();
  float getBattVoltage();
  float get5vVoltage();
  float get3v3Voltage();
  float get5vCurrent();

private:
  float _batt_voltage = 0;
  float _5v_voltage = 0;
  float _3v3_voltage = 0;
  float _5v_current = 0;

  const float _batt_vsns_divider_upper = 100000;
  const float _batt_vsns_divider_lower = 34000;

  const float _5v_vsns_divider_upper = 8250;
  const float _5v_vsns_divider_lower = 9760;

  const float _3v3_vsns_divider_upper = 750;
  const float _3v3_vsns_divider_lower = 8250;

  const float _3v3_isns_shunt_R = 0.01;
  const float _3v3_isns_gain = 100;
};