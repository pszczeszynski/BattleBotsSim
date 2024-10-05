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
#include "PowerMonitor.h"
#include <SPI.h>
#include <RF24.h>
#include <cstring> // for std::memcpy

Radio<DriverStationMessage, RobotMessage> tx_radio{};
const byte address[6] = "00001"; // Define address/pipe to use.
char ping_response_buffer[64];
char sendBuffer[64];

volatile bool receivedPacket = false;
long int receivedPackets = 0;
long int sentPackets = 0;
long int lastReceivedPackets = 0;
long int lastSentPackets = 0;
uint32_t lastDebugTime = 0;
#define NO_MESSAGE_REINIT_TIME 70

#define RECEIVE_TIMEOUT 0

uint32_t lastSendTime = 0;
#define SEND_INTERVAL_TIME_US 5000
#define SEND_TIMEOUT 1

PowerMonitor txMonitor;

void HandleTxPacket()
{
    if (!tx_radio.Available()) return;
    RobotMessage message = tx_radio.Receive();

    if (message.type != RobotMessageType::INVALID)
    {
        receivedPackets ++;
        memset(sendBuffer, 0, sizeof(sendBuffer));
        std::memcpy(sendBuffer, &message, sizeof(RobotMessage));
        receivedPacket = true;

        // blink the LED
        digitalWrite(STATUS_3_LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(STATUS_3_LED_PIN, LOW);
    }
}

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
    attachInterrupt(digitalPinToInterrupt(RADIO_IRQ_PIN), HandleTxPacket, FALLING);

}

// RawHID packets are always 64 bytes
byte buffer[64];
unsigned int packetCount = 0;


//===============================================================================
//  Main
//===============================================================================
void tx_loop()
{
    static unsigned long lastTime = 0;
    static int curr_packet_id = 0;

    int n;
    n = RawHID.recv(buffer, RECEIVE_TIMEOUT);

    // if the message is a tx ping, send a response back
    // forward to the robot otherwise
    if (n > sizeof(DriverStationMessage))
    //if (micros() - lastSendTime > SEND_INTERVAL_TIME_US)
    {
        lastSendTime = micros();
        DriverStationMessage command;
        // reinterpret the buffer as a DriveCommand
        std::memcpy(&command, buffer, sizeof(command));
        /*command.type = DRIVE_COMMAND;
        command.driveCommand.backWeaponPower = 0;
        command.driveCommand.frontWeaponPower = 0;
        command.driveCommand.movement = 0;
        command.driveCommand.selfRighterPower = 0;
        command.driveCommand.turn = 0;
        command.radioChannel = TEENSY_RADIO_3;
        command.valid = true;*/

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
            sentPackets++;
        }
    }
    else
    {
        digitalWrite(STATUS_2_LED_PIN, LOW);
    }

    if(receivedPacket) {
        n = RawHID.send(sendBuffer, SEND_TIMEOUT);
        receivedPacket = false;
    }

    if (millis() - lastDebugTime > 1000) {
        Serial.print("Sent: ");
        Serial.print(sentPackets - lastSentPackets);
        Serial.print(", Received: ");
        Serial.println(receivedPackets - lastReceivedPackets);
        lastReceivedPackets = receivedPackets;
        lastSentPackets = sentPackets;
        lastDebugTime = millis();
    }
}
