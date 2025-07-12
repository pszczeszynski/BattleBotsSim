#pragma once

#include <FlexCAN_T4.h>

#include "Communication.h"
#include "Hardware.h"

#define CAN_RATE 500000 // must match vec set can rate

#define DEV_ID_MASK 0xff

typedef void (*MessageHandler_t)(const CAN_message_t &msg);

class CANBUS {
public:
  CANBUS();
  void Update();
  void write(CAN_message_t msg);
  void SendTeensy(CANMessage *msg);

  static uint8_t GetCanID(enum board_placement placement);
  static enum board_placement GetTeensyById(uint8_t id);
  static void OnMessage(const CAN_message_t &msg);

  static void SetVESCHandler(MessageHandler_t vesc_handler);
  static void SetTeensyHandler(MessageHandler_t teensy_handler);

private:
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

  static MessageHandler_t _vesc_handler;
  static MessageHandler_t _teensy_handler;
};