#include <sys/select.h>
#include "Vesc.h"

// Voltage, Duty Cycle, RPM, AH, FET TEMP, MOTOR TEMP, CURRENT
int ids[4];
float volts[4];
float duty_cycles[4];
int rpms[4];
float ahs[4];
float fet_temps[4];
float motor_temps[4];
float currents[4];

VESC::VESC(int l_drive_id, int r_drive_id, int f_weapon_id, int b_weapon_id)
{
    ids[l_drive] = l_drive_id;
    ids[r_drive] = r_drive_id;
    ids[f_weapon] = f_weapon_id;
    ids[b_weapon] = b_weapon_id;


    // set volts, duty_cycles, rpms, ahs, fet_temps, motor_temps, currents to 0
    for (int i = 0; i < 4; i++)
    {
        volts[i] = 0;
        duty_cycles[i] = 0;
        rpms[i] = 0;
        ahs[i] = 0;
        fet_temps[i] = 0;
        motor_temps[i] = 0;
        currents[i] = 0;
    }

    CAN1.begin();
    CAN1.setBaudRate(CAN_RATE);
    CAN1.setMaxMB(16);
    CAN1.enableFIFO();
    CAN1.enableFIFOInterrupt();
    CAN1.onReceive(OnMessage);
}

#define MSG_ID_MASK 0xffffff00
#define VESC_ID_MASK 0xff
#define MSG_ID_INPUT_VOLTAGE 27
#define MSG_ID_DUTY_CYCLE 9
#define MSG_ID_AH 14
#define MSG_ID_TEMPS_AND_CURRENT 16
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
    if (ids[l_drive] == vesc_id)
    {
        enum_index = l_drive;
    }
    else if (ids[r_drive] == vesc_id)
    {
        enum_index = r_drive;
    }
    else if (ids[f_weapon] == vesc_id)
    {
        enum_index = f_weapon;
    }
    else if (ids[b_weapon] == vesc_id)
    {
        enum_index = b_weapon;
    }
    else
    {
        Serial.println("Unknown VESC ID");
        return;
    }

    Serial.print("Message ID: ");
    Serial.print(msg_id, HEX);
    Serial.print(", VESC ID: ");
    Serial.println(vesc_id);

    // Serial.print("Data: ");
    // for (uint8_t i = 0; i < msg.len; i++)
    // {
    //     Serial.print(msg.buf[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    if (msg_id == MSG_ID_INPUT_VOLTAGE)
    {
        float input_voltage = ((msg.buf[4] << 8) | msg.buf[5]) / VOLTAGE_SCALE;
        volts[enum_index] = input_voltage;
        // Serial.print("Input Voltage: ");
        // Serial.println(input_voltage);
    }
    else if (msg_id == MSG_ID_DUTY_CYCLE)
    {
        // duty cycle is a float from -1 to 1
        float duty_cycle = int16_t((msg.buf[6] << 8) | msg.buf[7]) / DUTY_CYCLE_SCALE;
        float rpm = ((msg.buf[3] << 24) | msg.buf[2] << 16 | msg.buf[1] << 8 | msg.buf[0]);
        duty_cycles[enum_index] = duty_cycle;
        rpms[enum_index] = rpm;
        // Serial.print("Duty Cycle: ");
        // Serial.println(duty_cycle);
        // Serial.print("RPM: ");
        // Serial.println(rpm);
    }
    else if (msg_id == MSG_ID_AH)
    {
        // ah means amp hours
        float ah = ((msg.buf[0] << 24) | msg.buf[1] << 16 | msg.buf[2] << 8 | msg.buf[3]) / AH_SCALE;
        ahs[enum_index] = ah;
        // Serial.print("Amp Hours: ");
        // Serial.println(ah);
    }
    else if (msg_id == MSG_ID_TEMPS_AND_CURRENT)
    {
        float fet_temp = (msg.buf[0] << 8 | msg.buf[1]) / TEMP_AND_CURRENT_SCALE;
        float motor_temp = int16_t((msg.buf[2] << 8 | msg.buf[3])) / TEMP_AND_CURRENT_SCALE;
        float current = int16_t(msg.buf[4] << 8 | msg.buf[5]) / TEMP_AND_CURRENT_SCALE;

        fet_temps[enum_index] = fet_temp;
        motor_temps[enum_index] = motor_temp;
        currents[enum_index] = current;
        // Serial.print("FET Temperature: ");
        // Serial.println(fet_temp);
        // Serial.print("Motor Temperature: ");
        // Serial.println(motor_temp);
        // Serial.print("Current: ");
        // Serial.println(current);
    }
}

void VESC::_SetMotorPower(float power, int motorIndex)
{
    if (abs(power) <= 0.001 && abs(lastPowers[motorIndex]) <= 0.001)
    {
        return;
    }

    lastPowers[motorIndex] = power;

    long frame_id = 0x00000000;

    CAN_message_t message;
    int dutyCycle = 100000 * power;
    message.id = frame_id | ids[motorIndex];
    message.len = 4;
    message.flags.extended = 1;

    // for each byte in the duty cycle, add it to the message
    // we need to do this in reverse order because the VESC expects the bytes in reverse order
    for (int i = 0; i < 4; i++)
    {
        message.buf[3 - i] = ((dutyCycle >> (8 * i)) & 0xff);
    }

    CAN1.write(message);
}

void VESC::Drive(float leftPower, float rightPower)
{
    _SetMotorPower(leftPower, l_drive);
    _SetMotorPower(rightPower, r_drive);

    Serial.print("Driving with leftPower: ");
    Serial.println(leftPower);
}

void VESC::DriveWeapons(float frontPower, float backPower)
{
    _SetMotorPower(frontPower, f_weapon);
    _SetMotorPower(backPower, b_weapon);

    // display each weapon power
    Serial.print("Driving weapons with frontPower: ");
    Serial.println(frontPower);
    Serial.print("Driving weapons with backPower: ");
    Serial.println(backPower);
}

void VESC::Update()
{
    CAN1.events();
}

void VESC::GetCurrents(unsigned char* outCurrents)
{
    for (int i = 0; i < 4; i++)
    {
        outCurrents[i] = (unsigned char) (currents[i] * TEMP_AND_CURRENT_SCALE);
    }
}

void VESC::GetVolts(unsigned char* outVolts)
{
    for (int i = 0; i < 4; i++)
    {
        outVolts[i] = (unsigned char) (volts[i] * VOLTAGE_SCALE);
    }
}

void VESC::GetRPMs(unsigned char* outRPMs)
{
    for (int i = 0; i < 4; i++)
    {
        outRPMs[i] = (unsigned char) (rpms[i] / 100);
    }
}

void VESC::GetFETTemps(unsigned char* outFetTemps)
{
    for (int i = 0; i < 4; i++)
    {
        outFetTemps[i] = (unsigned char) (fet_temps[i] * TEMP_AND_CURRENT_SCALE);
    }
}