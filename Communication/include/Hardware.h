#pragma once

////////////////////////////COMMON DEFINITIONS////////////////////////////

#define SERIAL_BAUD 460800

enum board_placement
{
    invalidPlacement = -1,
    tx = 0,
    rxLeft,
    rxCenter,
    rxRight,
};

///////////////////////////COMMON BOARD PINOUTS///////////////////////////
#define FIRMWARE_SELECT_MSB_PIN 0
#define FIRMWARE_SELECT_LSB_PIN 1

// status leds
#define STATUS_1_LED_PIN 3
#define STATUS_2_LED_PIN 4
#define STATUS_3_LED_PIN 5
#define STATUS_4_LED_PIN 6


///////////////////////////RX SPECIFIC PINOUTS///////////////////////////

// closest to the middle of the teensy
#define SELF_RIGHTER_MOTOR_PIN 14

#define LED_PIN 2

///////////////////////////TX SPECIFIC PINOUTS///////////////////////////

