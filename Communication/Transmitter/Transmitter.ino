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

    Serial.begin(460800);
    Serial.clear();
    
    Serial.println("Initializing radio...");
    radio = new Radio<DriveCommand, RobotMessage>();
    Serial.println("Success!");

    Serial.println("Finished initializing");

}

GenericReceiver<DriveCommand> serialReceiver(sizeof(DriveCommand)*2, [](char &c)
                                             {
                                            // if there is data available
                                            if (Serial.available())
                                            {
                                                // read the data
                                                c = Serial.read();
                                                return true;
                                            }
                                            // else return false
                                            return false; });


//===============================================================================
//  Main
//===============================================================================
long int total_packets = 0;
#define NO_MESSAGE_REINIT_TIME 70
void loop()
{
    static unsigned long lastReceiveTime = 0;

    // read all existing data from serial
    serialReceiver.readUntilEnd();

    // if there are commands
    if (serialReceiver.isLatestDataValid())
    {

        // Serial.println("sending");
        // get the latest data from the serial receiver
        DriveCommand command = serialReceiver.getLatestData();
        // send over radio to receiver
        radio->Send(command);
    }

    // while there is data available to read
    while (radio->Available())
    {
        // read data from rc
        RobotMessage message = radio->Receive();
        if (message.type != RobotMessageType::INVALID)
        {

            lastReceiveTime = millis();

            total_packets ++;

            // print data to serial for driver station
            Serial.print("<<<");
            Serial.write((char*) &message, sizeof(message));
            Serial.print(">>>");

            // flush
            Serial.flush();
        }
    }

    static unsigned long lastPrintTime = 0;

    if (millis() - lastPrintTime > 1000)
    {
        // print packets / sec
        // Serial.print("Packets / sec: ");
        // Serial.println(total_packets * 1000.0 / (millis() - lastPrintTime));
        total_packets = 0;
        lastPrintTime = millis();
    }

    // // if we had no data, set the led off
    // if (!available)
    // {
    //     digitalWrite(LED_PORT, LOW);
    // }

    // if (millis() - lastReceiveTime > NO_MESSAGE_REINIT_TIME)
    // {
    //     // Serial.println("ERROR: no messages -> re-init");
    //     radio->InitRadio();
    //     lastReceiveTime = millis();
    // }
}
