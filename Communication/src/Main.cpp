#include <Arduino.h>
#include "Hardware.h"
#include "Transmitter.h"
#include "Receiver.h"


enum board_placement placement = invalidPlacement;

void readFirmwareSelect() {
    pinMode(FIRMWARE_SELECT_MSB_PIN, INPUT_PULLUP);
    pinMode(FIRMWARE_SELECT_LSB_PIN, INPUT_PULLUP);
    delay(10);

#if defined(FORCE_TX_FIRMWARE)
    placement = tx;
#elif defined(FORCE_RX_LEFT_FIRMWARE)
    placement = rxLeft;
#elif defined(FORCE_RX_CENTER_FIRMWARE)
    placement = rxCenter;
#elif defined(FORCE_RX_RIGHT_FIRMWARE)
    placement = rxRight;
#else
    placement = (enum board_placement)((digitalRead(FIRMWARE_SELECT_MSB_PIN) << 1) +
        digitalRead(FIRMWARE_SELECT_LSB_PIN));
#endif
}

void setup()
{
    readFirmwareSelect();
    switch(placement)
    {
        case tx:
            tx_setup();
            break;
        case rxLeft:
        case rxCenter:
        case rxRight:
            rx_setup();
            break;
        case invalidPlacement:
        default:
            // should not ever happen
            break;
    }
}

void loop()
{
    switch(placement)
    {
        case tx:
            tx_loop();
            break;
        case rxLeft:
        case rxCenter:
        case rxRight:
            rx_loop();
            break;
        case invalidPlacement:
        default:
            // should not ever happen
            break;
    }
}