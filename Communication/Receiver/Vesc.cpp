#include <sys/select.h>
#include "Vesc.h"

VESC::VESC(int l_drive_id, int r_drive_id, int f_weapon_id, int b_weapon_id)
{
    ids[l_drive] = l_drive_id;
    ids[r_drive] = r_drive_id;
    ids[f_weapon] = f_weapon_id;
    ids[b_weapon] = b_weapon_id;

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
        Serial.print("Input Voltage: ");
        Serial.println(input_voltage);
    }
    else if (msg_id == MSG_ID_DUTY_CYCLE)
    {
        // duty cycle is a float from -1 to 1
        float duty_cycle = int16_t((msg.buf[6] << 8) | msg.buf[7]) / DUTY_CYCLE_SCALE;
        float rpm = ((msg.buf[3] << 24) | msg.buf[2] << 16 | msg.buf[1] << 8 | msg.buf[0]);
        Serial.print("Duty Cycle: ");
        Serial.println(duty_cycle);
        Serial.print("RPM: ");
        Serial.println(rpm);
    }
    else if (msg_id == MSG_ID_AH)
    {
        // ah means amp hours
        float ah = ((msg.buf[0] << 24) | msg.buf[1] << 16 | msg.buf[2] << 8 | msg.buf[3]) / AH_SCALE;
        Serial.print("Amp Hours: ");
        Serial.println(ah);
    }
    else if (msg_id == MSG_ID_TEMPS_AND_CURRENT)
    {
        float fet_temp = (msg.buf[0] << 8 | msg.buf[1]) / TEMP_AND_CURRENT_SCALE;
        float motor_temp = int16_t((msg.buf[2] << 8 | msg.buf[3])) / TEMP_AND_CURRENT_SCALE;
        float current = int16_t(msg.buf[4] << 8 | msg.buf[5]) / TEMP_AND_CURRENT_SCALE;
        Serial.print("FET Temperature: ");
        Serial.println(fet_temp);
        Serial.print("Motor Temperature: ");
        Serial.println(motor_temp);
        Serial.print("Current: ");
        Serial.println(current);
    }
}

void VESC::_SetMotorPower(float power, int motorIndex)
{
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
}

void VESC::Update()
{
    CAN1.events();
}