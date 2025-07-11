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
        case SELF_RIGHTER_CAN_ID:
            //VESC::OnMessage(msg);
            if(_vesc_handler)
            {
                (*_vesc_handler)(msg);
            }
            break;
        case LEFT_DRIVE_TEENSY_ID:
        case RIGHT_DRIVE_TEENSY_ID:
        case REAR_WEP_TEENSY_ID:
        case FRONT_WEP_TEENSY_ID:
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
        case rxWepFront:
            return FRONT_WEP_TEENSY_ID;
        case rxWepRear:
            return REAR_WEP_TEENSY_ID;
        case rxDriveLeft:
            return LEFT_DRIVE_TEENSY_ID;
        case rxDriveRight:
            return RIGHT_DRIVE_TEENSY_ID;
        default:
            return 0;
    }
}

enum board_placement CANBUS::GetTeensyById(uint8_t id)
{
    switch(id)
    {
        case FRONT_WEAPON_CAN_ID:
            return rxWepFront;
        case REAR_WEP_TEENSY_ID:
            return rxWepRear;
        case LEFT_DRIVE_TEENSY_ID:
            return rxDriveLeft;
        case RIGHT_DRIVE_TEENSY_ID:
            return rxDriveRight;
        default:
            return invalidPlacement;  
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