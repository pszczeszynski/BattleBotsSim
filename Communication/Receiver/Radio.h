#pragma once
#include "Communication.h"

/**
 * Radio.h
 *
 * This is used for both the transmitter and the receiver, and thus
 * should be the same on both. Please don't change just one.
 */

#define RADIO_BAUD 460800

enum SendOutput
{
    SEND_SUCCESS,
    FIFO_FAIL,
    HW_FAULT
};


// int count = 0;
// CircularDeque<char> recentChars{300};

template <typename SendType, typename ReceiveType>
class Radio
{
public:
    Radio();
    void InitRadio();

    SendOutput Send(SendType &message);
    ReceiveType Receive();

    bool Available();

private:
    GenericReceiver<DriveCommand> serialReceiver{sizeof(ReceiveType) * 2, [](char &c)
                                                 {
                                                     // if there is data available
                                                     if (Serial1.available())
                                                     {
                                                        //  count += 1;

                                                        //  if (count % 1000 == 0)
                                                        //  {
                                                        //      Serial.println("messages: " + String(count / sizeof(ReceiveType)));
                                                        //  }

                                                        //  recentChars.push_back(c);

                                                        //  // display recent chars
                                                        // Serial.print("Recent chars: ");
                                                        // for (int i = 0; i < 300; i++)
                                                        // {
                                                        //     Serial.print(recentChars[i]);
                                                        // }

                                                        // Serial.println();


                                                         // read the data
                                                         c = Serial1.read();
                                                         return true;
                                                     }
                                                     // else return false
                                                     return false;
                                                 }};
};

template <typename SendType, typename ReceiveType>
Radio<SendType, ReceiveType>::Radio()
{
    InitRadio();
}

template <typename SendType, typename ReceiveType>
void Radio<SendType, ReceiveType>::InitRadio()
{
    Serial1.begin(RADIO_BAUD);
}

int MIN_INTER_SEND_TIME = 15;
long lastSendTime = 0;
long intervalStartTime = 0;
int packetCount = 0;

/**
 * Receives a message from the driver station
 */
template <typename SendType, typename ReceiveType>
SendOutput Radio<SendType, ReceiveType>::Send(SendType &message)
{
    long currentTime = millis();

    // Check if it's been more than 1 second since the last interval started
    if (currentTime - intervalStartTime >= 1000)
    {
        // Calculate the average packets per second
        float avgPacketsPerSecond = packetCount * 1000.0 / (currentTime - intervalStartTime);

        // Print the average rate
        Serial.print("Average packets/sec: ");
        Serial.println(avgPacketsPerSecond);

        // Reset the interval and packet count
        intervalStartTime = currentTime;
        packetCount = 0;
    }

    if (currentTime - lastSendTime < MIN_INTER_SEND_TIME)
    {
        return SEND_SUCCESS;
    }

    // Increment packet count since a packet is being sent
    packetCount++;

    lastSendTime = currentTime;


    // Serial1.write(MESSAGE_START_CHAR);
    // Send each byte of the array via Serial (which is connected to the radio)
    for (int i = 0; i < 10; i++)
    {
        // send the random char
        Serial1.write((char) 9);
    }
    // Serial1.write(MESSAGE_END_CHAR);

    Serial1.flush();

    // return success
    return SEND_SUCCESS;
}

template <typename SendType, typename ReceiveType>
ReceiveType Radio<SendType, ReceiveType>::Receive()
{
    ReceiveType receiveMessage;
    // Zero out
    memset(&receiveMessage, 0, sizeof(receiveMessage));

    // if there is a valid message
    if (serialReceiver.isLatestDataValid())
    {
        // get the latest data from the serial receiver
        receiveMessage = serialReceiver.getLatestData();
        Serial.println("VALID");
    }

    // Read the data from the radio
    serialReceiver.update();

    // return the message
    return receiveMessage;
}

template <typename SendType, typename ReceiveType>
bool Radio<SendType, ReceiveType>::Available()
{
    // Read the data from the radio
    serialReceiver.update();

    // Check if there is enough data in the buffer
    return serialReceiver.isLatestDataValid();
}
