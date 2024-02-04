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
#include "Communication.h"
#include <SPI.h>
#include "Radio.h"
#include <RF24.h>
#include <cstring> // for std::memcpy

#define LED_PORT 16
Radio<DriveCommand, RobotMessage>* radio;
const byte address[6] = "00001"; // Define address/pipe to use.

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    Serial.println("Initializing...");
    // initialize the digital pin as an output.
    pinMode(LED_PORT, OUTPUT);

    Serial.begin(9600);//460800);
    // Serial.clear();
    
    Serial.println("Initializing radio...");
    radio = new Radio<DriveCommand, RobotMessage>();
    Serial.println("Success!");

    Serial.println("Finished initializing");

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
void loop()
{
    static unsigned long lastReceiveTime = 0;
    static unsigned long lastTime = 0;
    static int curr_packet_id = 0;

    int n;
    // n = RawHID.recv(buffer, 0); // 0 timeout = do not wait

    // if (n > sizeof(DriveCommand))
    // {
    //     DriveCommand command;
    //     // reinterpret the buffer as a DriveCommand
    //     std::memcpy(&command, buffer, sizeof(command));
    //     // send over radio to receiver
    //     radio->Send(command);

    //     Serial.println("Sent message");
    // }


    delay(10);
    bool hadData = false;
    // read data from rc
    RobotMessage message;// = radio->Receive();
    message.type = IMU_DATA;
    message.packetID = curr_packet_id;
    curr_packet_id ++;
    curr_packet_id %= 10000000;

    // debugging, add timestamp
    message.packetTimeMS = millis() % 10000;

    if (message.type != RobotMessageType::INVALID)
    {
        Serial.println("Time " + String(millis() % 1000));
        Serial.send_now();

        // Serial.println("data was valid");
        lastReceiveTime = millis();

        total_packets ++;


        char sendBuffer[64];
        // zero out
        memset(sendBuffer, 0, sizeof(sendBuffer));
        std::memcpy(sendBuffer, &message, sizeof(RobotMessage));

        // Serial.println("about to send via hid");
        n = RawHID.send(sendBuffer, SEND_TIMEOUT);
        // Serial.println("sent via hid");
        if (n <= 0)
        {
            // Serial.println("Error forwarding message");
        }

        // Serial.println("forwarded message to computer");
        hadData = true;
    }

    // Serial.println("end of loop()");

    // if we had no data, set the led off
    if (!hadData)
    {//
        // digitalWrite(LED_PORT, LOW);
    }

    // if (millis() - lastReceiveTime > NO_MESSAGE_REINIT_TIME)
    // {
    //     // Serial.println("ERROR: no messages -> re-init");
    //     radio->InitRadio();
    //     lastReceiveTime = millis();
    // }
}
