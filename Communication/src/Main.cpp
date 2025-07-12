#include "Hardware.h"
#include <Arduino.h>

#if defined(FORCE_TX_FIRMWARE)
#include "Transmitter.h"
#endif

#include "Receiver.h"

enum board_placement placement = invalidPlacement;

void readFirmwareSelect() {
  pinMode(FIRMWARE_SELECT_MSB_PIN, INPUT_PULLUP);
  pinMode(FIRMWARE_SELECT_LSB_PIN, INPUT_PULLUP);
  delay(10);

#if defined(FORCE_TX_FIRMWARE)
  placement = tx;
#elif defined(FORCE_RX_FIRMWARE)
  placement =
      (enum board_placement)((digitalRead(FIRMWARE_SELECT_MSB_PIN) << 1) +
                             digitalRead(FIRMWARE_SELECT_LSB_PIN));
#elif defined(FORCE_RX_DRIVE_LEFT_FIRMWARE)
  placement = rxDriveLeft;
#elif defined(FORCE_RX_WEP_FRONT_FIRMWARE)
  placement = rxWepFront;
#elif defined(FORCE_RX_WEP_REAR_FIRMWARE)
  placement = rxWepRear;
#elif defined(FORCE_RX_DRIVE_RIGHT_FIRMWARE)
  placement = rxDriveRight;
#else
#error "Must select TX or RX firmware target, unified target is deprecated"
#endif
}

void setup() {
  readFirmwareSelect();
  switch (placement) {
  case tx:
#if defined(FORCE_TX_FIRMWARE)
    tx_setup();
    break;
#endif
  case rxDriveLeft:
  case rxDriveRight:
  case rxWepFront:
  case rxWepRear:
    rx_setup();
    break;
  case invalidPlacement:
  default:
    // should not ever happen
    break;
  }
}

void loop() {
  switch (placement) {
  case tx:
#if defined(FORCE_TX_FIRMWARE)
    tx_loop();
    break;
#endif
  case rxDriveLeft:
  case rxDriveRight:
  case rxWepFront:
  case rxWepRear:
    rx_loop();
    break;
  case invalidPlacement:
  default:
    // should not ever happen
    break;
  }
}