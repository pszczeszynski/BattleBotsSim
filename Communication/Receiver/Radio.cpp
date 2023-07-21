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
    radio.openReadingPipe(0, address);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX);
    radio.startListening();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_1MBPS);
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
#define MIN_INTER_SEND_TIME 15
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


    // check if radio died
    if (!radio.isChipConnected())
    {
        Serial.println("Radio died");
        InitRadio();

        return;
    }
    if (i % 100 == 0)
    {
        // reset radio
        InitRadio();

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
    if (i % 500 == 0)
    {
    Serial.print("Sent robot message: ");
    Serial.println(i);

    }
    delay(10);
}

bool Radio::Available()
{
    return radio.available();
}