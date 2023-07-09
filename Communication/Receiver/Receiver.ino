/*
 * nRF24L01 Receiver Test Software
 * 
 * Exercises the nRF24L01 Module.  This code runs on the receiver 'slave' device.
 * Use the nRF24L01_Transmitter_Test software for the transmitting 'master' device
 * 
 * This uses the RF24.h library which can be installed from the Arduino IDE
 * Pins used for the SPI interface are determined by the Arduino being used. 
 * The other two pins are arbitrary and can be changed if needed.  Redfine them in the RF24  
 * statement.  Default shown here is to use pins 7 &
 */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(14, 10); // CE, CSN      // Define instance of RF24 object called 'radio' and define pins used
const byte address[6] = "00001";  // Define address/pipe to use. This can be any 5 alphnumeric letters/numbers
//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  Serial.begin(9600);                // Start serial port to display messages on Serial Monitor Window
  radio.begin();                     // Start instance of the radio object
  radio.openReadingPipe(0, address); // Setup pipe to write data to the address that was defined
  radio.setPALevel(RF24_PA_MAX);     // Set the Power Amplified level to MAX in this case
  radio.startListening();            // We are going to be the receiver, so we need to start listening
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  if (radio.available()) {  
    char text[32] = "";               // Clear buffer
    radio.read(&text, sizeof(text));  // Read incoming message into buffer
    Serial.println(text);             // Print the message to the Serial Monitor window
  }
}