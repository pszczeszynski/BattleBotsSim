#include "Radio.h"
#include <string.h> // for memcpy
#include "Communication.h"

Radio::Radio()
{
    const byte address[6] = "00001";
    radio.begin();                     // Start instance of the radio object
    radio.openReadingPipe(0, address); // Setup pipe to write data to the address that was defined
    radio.openWritingPipe(address);    // Setup pipe to write data to the address that was defined
    radio.setPALevel(RF24_PA_MAX);     // Set the Power Amplified level to MAX in this case
    radio.startListening();            // We are going to be the receiver, so we need to start listening
    radio.setAutoAck(false);
}

bool receivedDriveRecently = false;
/**
 * Receives a message from the driver station
 */
DriveCommand Radio::Receive()
{
    DriveCommand driveData {0};
    // read the data from the radio
    radio.read(&driveData, sizeof(driveData));

    if (!driveData.valid)
    {
        return driveData;
    }
    receivedDriveRecently = true;

    Serial.print("Received drive command with movement: ");
    Serial.println(driveData.movement);

    // TODO: check + handle errors

    // return the data
    return driveData;
}

int i = 0;
unsigned long lastSendTime = 0;
#define MIN_INTER_SEND_TIME 5
/**
 * Sends a message from the robot to the driver station
 */
void Radio::Send(RobotMessage& message)
{
    unsigned long currTime = millis();
    if (currTime - lastSendTime < MIN_INTER_SEND_TIME)
    {
        return;
    }
    lastSendTime = currTime;
    //if (!receivedDriveRecently) return;
    receivedDriveRecently = false;
    i += 1;
    i %= 500;
    message.accel.x = i;
    radio.stopListening();
    radio.write(&message, sizeof(message));
    radio.startListening();
    Serial.print("Sent robot message: ");
    Serial.println(i);
    delay(10);
}

bool Radio::Available()
{
    return radio.available();
}