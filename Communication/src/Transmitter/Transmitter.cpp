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
#include "Transmitter.h"
#include "Hardware.h"
#include "Communication.h"
#include "CircularDeque.h"
#include "GenericReceiver.h"
#include "Radio.h"
#include "Utils.h"
#include <SPI.h>
#include <RF24.h>
#include <cstring> // for std::memcpy

Radio<DriverStationMessage, RobotMessage> tx_radio{};
const byte address[6] = "00001"; // Define address/pipe to use.
char ping_response_buffer[64];

//===============================================================================
//  Initialization
//===============================================================================
void tx_setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println("Powering on Transmitter!");
    Serial.println("Initializing...");
    // initialize the digital pin as an output.
    pinMode(STATUS_1_LED_PIN, OUTPUT);
    pinMode(STATUS_2_LED_PIN, OUTPUT);
    pinMode(STATUS_3_LED_PIN, OUTPUT);
    pinMode(STATUS_4_LED_PIN, OUTPUT);
    
    Serial.println("Success!");

    Serial.println("Finished initializing");

    // set up a hardcoded response to ping requests
    RobotMessage ping_response;
    ping_response.type = LOCAL_PING_RESPONSE;
    ping_response.valid = true;
    memset(ping_response_buffer, 0, sizeof(ping_response_buffer));
    std::memcpy(ping_response_buffer, &ping_response, sizeof(RobotMessage));

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
#define RECEIVE_TIMEOUT 0
void tx_loop()
{
    static unsigned long lastReceiveTime = 0;
    static unsigned long lastTime = 0;
    static int curr_packet_id = 0;

    int n;
    n = RawHID.recv(buffer, RECEIVE_TIMEOUT);

    // if the message is a tx ping, send a response back
    // forward to the robot otherwise
    if (n > sizeof(DriverStationMessage))
    {
        DriverStationMessage command;
        // reinterpret the buffer as a DriveCommand
        std::memcpy(&command, buffer, sizeof(command));

        if(command.type == LOCAL_PING_REQUEST)
        {
            n = RawHID.send(ping_response_buffer, SEND_TIMEOUT);
        }
        else
        {
            tx_radio.SetChannel(command.radioChannel);

            // send over radio to receiver
            SendOutput status = tx_radio.Send(command);

            // blink the LED
            digitalWrite(STATUS_2_LED_PIN, HIGH);
        }
    }
    else
    {
        digitalWrite(STATUS_2_LED_PIN, LOW);
    }

    bool hadData = false;
    // read data from rc
    RobotMessage message = tx_radio.Receive();

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
    //     tx_radio->InitRadio();
    //     lastReceiveTime = millis();
    // }
}