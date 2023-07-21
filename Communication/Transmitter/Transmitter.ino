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
//#include <nRF24L01.h>
#include <RF24.h>
#include <cstring> // for std::memcpy
#include <EEPROM.h>

#define LED_PORT 16
RF24 radio(14, 10);              // CE, CSN      // Define instance of RF24 object called 'radio' and define pins used
const byte address[6] = "00001"; // Define address/pipe to use.


void InitRadio()
{
    radio.begin();                  // Start instance of the radio object
    radio.openReadingPipe(0, address); // Setup pipe to read data from the address
    radio.openWritingPipe(address); // Setup pipe to write data to the address that was defined
    radio.setPALevel(RF24_PA_MAX);  // Set the Power Amplified level to MAX in this case
    radio.startListening();
    radio.setAutoAck(false);
    // set data rate to 1 Mbps
    radio.setDataRate(RF24_1MBPS);
}
//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    // initialize the digital pin as an output.
    pinMode(LED_PORT, OUTPUT);

    Serial.begin(9600);
    Serial.clear();
    InitRadio();
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

void SendDriveCommand(DriveCommand &driveData)
{
    // write the data to the radio
    radio.stopListening();
    // send start bit
    // radio.write(&MESSAGE_START_CHAR, sizeof(MESSAGE_START_CHAR));
    radio.write(&driveData, sizeof(driveData));
    // send end bit
    // radio.write(&MESSAGE_END_CHAR, sizeof(MESSAGE_END_CHAR));
    radio.startListening();
}

unsigned long lastReceiveTime = 0;
unsigned long lastInitTime = 0;
int i =0;
//===============================================================================
//  Main
//===============================================================================
void loop()
{
    serialReceiver.update();

    // if there are commands
    if (serialReceiver.isLatestDataValid())
    {
        // write to radio
        DriveCommand command = serialReceiver.getLatestData();
        SendDriveCommand(command);
    }

    if (millis() - lastInitTime > 500)
    {
      lastInitTime = millis();
      InitRadio();
    }

    while (radio.available())
    {
        // turn on arduino led
        digitalWrite(LED_PORT, HIGH);

        // read data from rc
        RobotMessage message {0};
        radio.read(&message, sizeof(message));

        Serial.print(MESSAGE_START_CHAR);
        Serial.write((char*) &message, sizeof(message));
        Serial.print(MESSAGE_END_CHAR);
    }
    digitalWrite(LED_PORT, LOW);
}
