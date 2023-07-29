#include "Radio.h"
#include <string.h> // for memcpy
#include "Communication.h"

Radio::Radio()
{
    InitRadio();
}

void Radio::InitRadio()
{
    const byte address[6] = "00001";
    radio.begin();
    radio.openReadingPipe(1, address);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_1MBPS);
}

unsigned long lastTimeReceivedMessage = 0;
int i = 0;

#define MIN_INTER_RECEIVE_TIME 10
unsigned long lastSendTime = 0;
#define MIN_INTER_SEND_TIME 15

/**
 * Receives a message from the driver station
 */
void Radio::Send(RobotMessage &message)
{
    unsigned long currTime = millis();
    if (currTime - lastSendTime < MIN_INTER_SEND_TIME)
    {
        return;
    }

    unsigned long startTimeMillis = millis();

    radio.stopListening();
    radio.write(&message, sizeof(RobotMessage));
    // while NOT (the transmitting fifo (first arg) is empty (second arg))
    while (!radio.isFifo(true, true))
    {

    }
    radio.startListening();

    unsigned long endTimeMillis = millis();

    unsigned long timeElapsed = endTimeMillis - startTimeMillis;
    Serial.print("Send time MS: ");
    Serial.println(timeElapsed);
}

/**
 * Receives a message from the driver station
 */
DriveCommand Radio::Receive()
{
    unsigned long currTime = millis();
    if (currTime - lastTimeReceivedMessage < MIN_INTER_RECEIVE_TIME)
    {
        return DriveCommand();
    }

    lastTimeReceivedMessage = currTime;

    DriveCommand driveCommand;
    // if there is data available
    if (radio.available())
    {
        // read the data
        radio.read(&driveCommand, sizeof(DriveCommand));
    }
    return driveCommand;
}

bool Radio::Available()
{
    return radio.available();
}