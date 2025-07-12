#include "PowerMonitor.h"

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) +
         out_min;
}

void PowerMonitor::readSensors() {
  // voltage divider: V_output (sense) = V_input (actual) * R2 / (R1 + R2)
  // Input = Output / R2 * (R1 + R2)
  _batt_voltage = mapf(analogRead(VSNS_BATT_PIN), 0, 1023, 0, 3.3) /
                  _batt_vsns_divider_lower *
                  (_batt_vsns_divider_lower + _batt_vsns_divider_upper);

  _5v_voltage = mapf(analogRead(VSNS_5V_PIN), 0, 1023, 0, 3.3) /
                _5v_vsns_divider_lower *
                (_5v_vsns_divider_lower + _5v_vsns_divider_upper);

  _3v3_voltage = mapf(analogRead(VSNS_3V3_PIN), 0, 1023, 0, 3.3) /
                 _3v3_vsns_divider_lower *
                 (_3v3_vsns_divider_lower + _3v3_vsns_divider_upper);

  // shunt resistor: V_input = I * R, V_output = gain * V_input
  // I = V_output / (gain * R)
  _5v_current = mapf(analogRead(ISNS_5V_PIN), 0, 1023, 0, 3.3) /
                (_3v3_isns_gain * _3v3_isns_shunt_R);
}

float PowerMonitor::getBattVoltage() { return _batt_voltage; }

float PowerMonitor::get5vVoltage() { return _5v_voltage; }

float PowerMonitor::get3v3Voltage() { return _3v3_voltage; }

float PowerMonitor::get5vCurrent() { return _5v_current; }