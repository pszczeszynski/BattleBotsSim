/*
 * nRF24L01 Transmitter Test Software
 *
 * Exercises the nRF24L01 Module.  This code runs on the transmitter 'Master' device.
 * Use the nRF24L01_Receiver_Test software for the receiving 'Slave' device
 *
 * This uses the RF24.h library which can be installed from the Arduino IDE
 * Pins used for the SPI interface are determined by the Arduino being used.
 * The other two pins are arbitrary and can be changed if needed.  Redfine them in the RF24
 * statement.  Default shown here is to use pins 7 & 8
 */
//#include "Communication.h"
#include "Communication.h"
#include "CircularDeque.h"
#include "GenericReceiver.h"
#include "Radio.h"
#include <SPI.h>
#include <RF24.h>
#include <cstring> // for std::memcpy
#include <Arduino.h>

// #define VENDOR_ID               0x16C0
// #define PRODUCT_ID              0x0480
// #define RAWHID_USAGE_PAGE       0xFFAC  // recommended: 0xFF00 to 0xFFFF
// #define RAWHID_USAGE            0x0300  // recommended: 0x0100 to 0xFFFF

#define RAWHID_TX_SIZE          64      // transmit packet size
#define RAWHID_TX_INTERVAL      1       // max # of ms between transmit packets
#define RAWHID_RX_SIZE          64      // receive packet size
#define RAWHID_RX_INTERVAL      1       // max # of ms between receive packets


// status leds
#define STATUS_1_LED_PIN 3
#define STATUS_2_LED_PIN 4
#define STATUS_3_LED_PIN 5
#define STATUS_4_LED_PIN 6

Radio<DriverStationMessage, RobotMessage>* radio;
const byte address[6] = "00001"; // Define address/pipe to use.

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    Serial.println("Powering on Transmitter!");
    Serial.println("Initializing...");
    // initialize the digital pin as an output.
    pinMode(STATUS_1_LED_PIN, OUTPUT);
    pinMode(STATUS_2_LED_PIN, OUTPUT);
    pinMode(STATUS_3_LED_PIN, OUTPUT);
    pinMode(STATUS_4_LED_PIN, OUTPUT);

    Serial.begin(460800);
    
    Serial.println("Initializing radio...");
    radio = new Radio<DriverStationMessage, RobotMessage>();
    Serial.println("Success!");

    Serial.println("Finished initializing");

    digitalWrite(STATUS_1_LED_PIN, HIGH);

}

// RawHID packets are always 64 bytes
byte buffer[64];
unsigned int packetCount = 0;


//===============================================================================
//  Main
//===============================================================================
long int total_packets = 0;
#define NO_MESSAGE_REINIT_TIME 70

#define SEND_TIMEOUT 1
#define RECEIVE_TIMEOUT 1
void loop()
{
    static unsigned long lastReceiveTime = 0;
    static unsigned long lastTime = 0;
    static int curr_packet_id = 0;

    int n;
    n = RawHID.recv(buffer, RECEIVE_TIMEOUT);

    if (n > sizeof(DriverStationMessage))
    {
        DriverStationMessage command;
        // reinterpret the buffer as a DriveCommand
        std::memcpy(&command, buffer, sizeof(command));

        radio->SetChannel(command.radioChannel);

        // send over radio to receiver
        radio->Send(command);

        // blink the LED
        digitalWrite(STATUS_2_LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(STATUS_2_LED_PIN, LOW);
    }

    bool hadData = false;
    // read data from rc
    RobotMessage message = radio->Receive();

    if (message.type != RobotMessageType::INVALID)
    {
        lastReceiveTime = millis();
        total_packets ++;

        char sendBuffer[64];
        memset(sendBuffer, 0, sizeof(sendBuffer));
        std::memcpy(sendBuffer, &message, sizeof(RobotMessage));

        n = RawHID.send(sendBuffer, SEND_TIMEOUT);

        hadData = true;

        // blink the LED
        digitalWrite(STATUS_3_LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(STATUS_3_LED_PIN, LOW);
    }

    // if (millis() - lastReceiveTime > NO_MESSAGE_REINIT_TIME)
    // {
    //     // Serial.println("ERROR: no messages -> re-init");
    //     radio->InitRadio();
    //     lastReceiveTime = millis();
    // }
}
