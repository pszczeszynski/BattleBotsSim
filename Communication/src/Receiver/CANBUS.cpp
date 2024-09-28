#include "Receiver/CANBUS.h"

CANBUS::CANBUS()
{
    Can0.begin();
    Can0.setBaudRate(500000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(OnMessage);
}

extern enum board_placement placement;

void CANBUS::write(CAN_message_t msg)
{
    Can0.write(msg);
}

void CANBUS::OnMessage(const CAN_message_t &msg)
{
    int dev_id = msg.id & DEV_ID_MASK;

    switch(dev_id)
    {
        case LEFT_MOTOR_CAN_ID:
        case RIGHT_MOTOR_CAN_ID:
        case FRONT_WEAPON_CAN_ID:
        case BACK_WEAPON_CAN_ID:
            //VESC::OnMessage(msg);
            if(_vesc_handler)
            {
                (*_vesc_handler)(msg);
            }
            break;
        case LEFT_TEENSY_ID:
        case CENTER_TEENSY_ID:
        case RIGHT_TEENSY_ID:
            if (placement == (dev_id & 0x0f))
            {
                Serial.println("Configuration Error: receiving CAN message from same ID as self");
            }
            // handle arbitration/telemetry from other teensys
            if(_teensy_handler)
            {
                (*_teensy_handler)(msg);
            }
            break;
        default:
            Serial.println("warning: unrecognized can ID");

    }
}

void CANBUS::Update()
{
    Can0.events();
}

uint8_t CANBUS::GetCanID(enum board_placement placement)
{
    switch(placement)
    {
        case rxLeft:
            return LEFT_TEENSY_ID;
        case rxCenter:
            return CENTER_TEENSY_ID;
        case rxRight:
            return RIGHT_TEENSY_ID;
        default:
            return 0;
    }
}

MessageHandler_t CANBUS::_teensy_handler = nullptr;
MessageHandler_t CANBUS::_vesc_handler = nullptr;

void CANBUS::SetVESCHandler(MessageHandler_t vesc_handler)
{
    _vesc_handler = vesc_handler;
}

void CANBUS::SetTeensyHandler(MessageHandler_t teensy_handler)
{
    _teensy_handler = teensy_handler;
}

void CANBUS::SendTeensy(CANMessage *msg)
{
    CAN_message_t message;
    message.id = CANBUS::GetCanID(placement);
    message.len = sizeof(CANMessage);
    memcpy(message.buf, msg, sizeof(CANMessage));
    Can0.write(message);
}