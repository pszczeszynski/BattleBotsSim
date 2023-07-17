#include "Radio.h"
#include <string.h> // for memcpy

Radio::Radio()
{
    const byte address[6] = "00001";
    radio.begin();                     // Start instance of the radio object
    radio.openReadingPipe(0, address); // Setup pipe to write data to the address that was defined
    radio.openWritingPipe(address);    // Setup pipe to write data to the address that was defined
    radio.setPALevel(RF24_PA_MAX);     // Set the Power Amplified level to MAX in this case
    radio.startListening();            // We are going to be the receiver, so we need to start listening
}

/**
 * Receives a message from the driver station
 */
DriveCommand Radio::Receive()
{
    DriveCommand driveData;
    radio.read(&driveData, sizeof(driveData));
    return driveData;
}

/**
 * Sends a message from the robot to the driver station
 */
void Radio::Send(RobotMessage& message)
{
    radio.write(&message, sizeof(message));
}

bool Radio::Available()
{
    return radio.available();
}