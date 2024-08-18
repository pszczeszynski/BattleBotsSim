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

#define VSNS_5V_PIN A6 //digital pin 20
#define ISNS_5V_PIN A2 //digital pin 16
#define VSNS_3V3_PIN A1 //digital pin 15
#define VSNS_BATT_PIN A3 //digital pin 17

#define RADIO_CE_PIN 7
#define RADIO_CS_PIN 9


///////////////////////////RX SPECIFIC PINOUTS///////////////////////////

// closest to the middle of the teensy
#define SELF_RIGHTER_MOTOR_PIN 14

#define LED_PIN 2

/////////////////////TX SPECIFIC RAWHID DEFINITIONS//////////////////////
#define VENDOR_ID               0x16C0
#define PRODUCT_ID              0x0480
#define RAWHID_USAGE_PAGE       0xFFAC  // recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE            0x0300  // recommended: 0x0100 to 0xFFFF

#define RAWHID_TX_SIZE          64      // transmit packet size
#define RAWHID_TX_INTERVAL      1       // max # of ms between transmit packets
#define RAWHID_RX_SIZE          64      // receive packet size
#define RAWHID_RX_INTERVAL      1       // max # of ms between receive packets
