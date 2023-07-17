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
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(14, 10);              // CE, CSN      // Define instance of RF24 object called 'radio' and define pins used
const byte address[6] = "00001"; // Define address/pipe to use.
unsigned long count = 0;         // Use to count the number of messages sent
char data[10];                   // Create a char array to hold count as a string

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    Serial.begin(9600);
    radio.begin();                  // Start instance of the radio object
    radio.openWritingPipe(address); // Setup pipe to write data to the address that was defined
    radio.setPALevel(RF24_PA_MAX);  // Set the Power Amplified level to MAX in this case
    radio.stopListening();          // We are going to be the transmitter, so we will stop listening
}

//===============================================================================
//  Main
//===============================================================================
void loop()
{
    if (Serial.available() > 0)
    {
        String receivedData = Serial.readStringUntil('\n');
        // ltoa(receivedData,data, 10);             // Convert the count and put into the char array.
        char text[30] = "Sending Message: "; // Create our base message.
        strcat(text, receivedData.c_str());  // Append the count to the base message

        radio.write(&text, sizeof(text));    // Write the char array.
        Serial.println("It's working");
    }
}
