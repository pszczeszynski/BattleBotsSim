/**
 * Receiver.ino
 */
#include "Communication.h"

#define VERBOSE_RADIO
#include "Radio.h"


#define SERIAL_BAUD 9600

Radio<RobotMessage, DriveCommand>* radio;


long startTime = 0;

void setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println("Initializing radio...");
    radio = new Radio<RobotMessage, DriveCommand>();
    Serial.println("Success!");


    startTime = millis();
}

int messageCount = 0;



void loop()
{
    // while (!radio->Available())
    // {
    //     // wait for a message
    // }

    // // get the message
    // DriveCommand message = radio->Receive();

    // if (!message.valid)
    // {
    //     Serial.println("Invalid message received");
    //     return;
    // }
    // // print
    // Serial.print("Received message with movement: ");
    // Serial.println(message.movement);


    RobotMessage response;
    //memset to 0
    memset(&response, 0, sizeof(RobotMessage));

    // set first byte to a char of 'a'
    ((char*)&response)[0] = 'a' + messageCount % 26;

    messageCount++;

    long currentTime = millis();
    long timeSinceStart = currentTime - startTime;

    // // print messages / sec
    // Serial.print("Messages / sec: ");
    // Serial.println(messageCount / (timeSinceStart / 1000.0));


    // send the message
    SendOutput result = radio->Send(response);
}