#include <sys/select.h>
#include "Vesc.h"
#include <cmath>
// numeric limits
#include <limits>

// Voltage, Duty Cycle, RPM, AH, FET TEMP, MOTOR TEMP, CURRENT
int _ids[4];
float _volts[4];
float _duty_cycles[4];
int _rpms[4];
float _ahs[4];
float _fet_temps[4];
float _motor_temps[4];
float _currents[4];

VESC::VESC(int l_drive_id, int r_drive_id, int f_weapon_id, int b_weapon_id)
{
    _ids[l_drive] = l_drive_id;
    _ids[r_drive] = r_drive_id;
    _ids[f_weapon] = f_weapon_id;
    _ids[b_weapon] = b_weapon_id;
    // set volts, duty_cycles, rpms, ahs, fet_temps, motor_temps, currents to 0
    for (int i = 0; i < 4; i++)
    {
        _volts[i] = 0;
        _duty_cycles[i] = 0;
        _rpms[i] = 0;
        _ahs[i] = 0;
        _fet_temps[i] = 0;
        _motor_temps[i] = 0;
        _currents[i] = 0;
    }

    Can0.begin();
    Can0.setBaudRate(500000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(OnMessage);
}


#define MSG_ID_MASK 0xffffff00
#define VESC_ID_MASK 0xff
#define MSG_ID_INPUT_VOLTAGE 0x1B
#define MSG_ID_DUTY_CYCLE 0x9
#define MSG_ID_AH 0xE
#define MSG_ID_TEMPS_AND_CURRENT 0x10
#define VOLTAGE_SCALE 10.0
#define DUTY_CYCLE_SCALE 1000.0
#define AH_SCALE 10000
#define TEMP_AND_CURRENT_SCALE 10

/**
 * @brief OnMessage is called when a CAN message is received
 * @param msg the CAN message to process
*/
static void VESC::OnMessage(const CAN_message_t &msg)
{
    uint32_t msg_id = (msg.id & MSG_ID_MASK) >> 8;
    int vesc_id = msg.id & VESC_ID_MASK;

    int enum_index = 0;
    if (_ids[l_drive] == vesc_id)
    {
        enum_index = l_drive;
    }
    else if (_ids[r_drive] == vesc_id)
    {
        enum_index = r_drive;
    }
    else if (_ids[f_weapon] == vesc_id)
    {
        enum_index = f_weapon;
    }
    else if (_ids[b_weapon] == vesc_id)
    {
        enum_index = b_weapon;
    }
    else
    {
        Serial.print("Unknown VESC ID: ");
        Serial.println(vesc_id);
        return;
    }

  Serial.print("Message ID: ");
  Serial.print(msg_id, HEX);
  Serial.print(", VESC ID: ");
  Serial.println(vesc_id);

  Serial.print("Data: ");
  for (uint8_t i = 0; i < msg.len; i++)
  {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
  }
  Serial.println();

    if (msg_id == MSG_ID_INPUT_VOLTAGE)
    {
        float input_voltage = ((msg.buf[4] << 8) | msg.buf[5]) / VOLTAGE_SCALE;
        _volts[enum_index] = input_voltage;
        // Serial.print("Input Voltage: ");
        // Serial.println(input_voltage);

    }
    else if (msg_id == MSG_ID_DUTY_CYCLE)
    {
        // duty cycle is a float from -1 to 1
        float duty_cycle = int16_t((msg.buf[6] << 8) | msg.buf[7]) / DUTY_CYCLE_SCALE;
        float rpm = ((msg.buf[0] << 24) | msg.buf[1] << 16 | msg.buf[2] << 8 | msg.buf[3]);
        _duty_cycles[enum_index] = duty_cycle;
        _rpms[enum_index] = rpm;
        // Serial.print("Duty Cycle: ");
        // Serial.println(duty_cycle);
        // Serial.print("RPM: ");
        // Serial.println(rpm);
    }
    else if (msg_id == MSG_ID_AH)
    {
        // ah means amp hours
        float ah = ((msg.buf[0] << 24) | msg.buf[1] << 16 | msg.buf[2] << 8 | msg.buf[3]) / AH_SCALE;
        _ahs[enum_index] = ah;
        // Serial.print("Amp Hours: ");
        // Serial.println(ah);
    }
    else if (msg_id == MSG_ID_TEMPS_AND_CURRENT)
    {
        float fet_temp = (msg.buf[0] << 8 | msg.buf[1]) / TEMP_AND_CURRENT_SCALE;
        float motor_temp = int16_t((msg.buf[2] << 8 | msg.buf[3])) / TEMP_AND_CURRENT_SCALE;
        float current = int16_t(msg.buf[4] << 8 | msg.buf[5]) / TEMP_AND_CURRENT_SCALE;

        _fet_temps[enum_index] = fet_temp;
        _motor_temps[enum_index] = motor_temp;
        _currents[enum_index] = current;
        // Serial.print("FET Temperature: ");
        // Serial.println(fet_temp);
        // Serial.print("Motor Temperature: ");
        // Serial.println(motor_temp);
        // Serial.print("Current: ");
        // Serial.println(current);
    }
}

float absolute(float f)
{
    return f > 0 ? f : -f;
}

#define DEAD_BAND 0.01f

void VESC::_SetMotorPower(float power, int motorIndex)
{
    // if the power is within the dead band, set it to 0
    if (absolute(power) <= DEAD_BAND)
    {
        power = 0;
    }

    // if the power is 0 and we already sent a 0, don't send another 0
    if (power == 0 && lastPowers[motorIndex] == 0)
    {
        return;
    }

    // save the last power
    lastPowers[motorIndex] = power;

    long frame_id = 0x00000000;

    CAN_message_t message;
    int dutyCycle = 100000 * power;
    message.id = frame_id | _ids[motorIndex];
    message.len = 4;
    message.flags.extended = 1;

    // for each byte in the duty cycle, add it to the message
    // we need to do this in reverse order because the VESC expects the bytes in reverse order
    for (int i = 0; i < 4; i++)
    {
        message.buf[3 - i] = ((dutyCycle >> (8 * i)) & 0xff);
    }

    Can0.write(message);
}
void VESC::Drive(float leftPower, float rightPower)
{
    _SetMotorPower(leftPower, l_drive);
    _SetMotorPower(rightPower, r_drive);
}

void VESC::DriveWeapons(float frontPower, float backPower)
{
    _SetMotorPower(frontPower, f_weapon);
    _SetMotorPower(backPower, b_weapon);
}

void VESC::Update()
{
    Can0.events();
}

unsigned char FloatToUnsignedChar(float f)
{
    // round f to nearest int
    int val = (std::round)(f);

    // get limit of unsigned char
    int limit = std::numeric_limits<unsigned char>::max();

    // enforce 0 -> limit
    val = std::max(0, val);
    val = std::min(limit, val);

    // return val as unsigned char
    return (unsigned char) val;
}

void VESC::GetCurrents(unsigned char *outCurrents)
{
    for (int i = 0; i < 4; i++)
    {
        outCurrents[i] = FloatToUnsignedChar(_currents[i]);
    }
}

void VESC::GetVolts(unsigned char* outVolts)
{
    for (int i = 0; i < 4; i++)
    {
        outVolts[i] = FloatToUnsignedChar(_volts[i]);
    }
}

void VESC::GetRPMs(unsigned char* outRPMs)
{
    for (int i = 0; i < 4; i++)
    {
        outRPMs[i] = FloatToUnsignedChar(abs(_rpms[i]) / 1000.0);
    }
}

void VESC::GetFETTemps(unsigned char* outFetTemps)
{
    for (int i = 0; i < 4; i++)
    {
        outFetTemps[i] = FloatToUnsignedChar(_fet_temps[i]);
    }
}


void VESC::GetMotorTemps(unsigned char* outMotorTemps){
  for (int i = 0; i < 4; i++)
    {
        outMotorTemps[i] = FloatToUnsignedChar(_motor_temps[i]);
    }
}

void VESC::GetFloatCurrents(float* outCurrents){
  outCurrents = _currents;
}
void VESC::GetFloatVolts(float* outVolts){
  outVolts = _volts;

}
void VESC::GetIntRPMs(int* outRPMs){
  outRPMs = _rpms;

}
void VESC::GetFloatFETTemps(float* outFetTemps){
  outFetTemps = _fet_temps;

}
void VESC::GetFloatMotorTemps(float* outMotorTemps){
  outMotorTemps = _motor_temps;

}
void VESC::GetFloatDutyCycle(float* outDutyCycles){
  outDutyCycles = _duty_cycles;

}