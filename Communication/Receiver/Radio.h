#pragma once

/**
 * Radio.h
 *
 * This is used for both the transmitter and the receiver, and thus
 * should be the same on both. Please don't change just one.
 */

#define RADIO_BAUD 230400

enum SendOutput
{
    SEND_SUCCESS,
    FIFO_FAIL,
    HW_FAULT
};

template <typename SendType, typename ReceiveType>
class Radio
{
public:
    Radio();
    void InitRadio();

    SendOutput Send(SendType &message);
    ReceiveType Receive();

    bool Available();
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

unsigned long lastTimeReceivedMessage = 0;

#define SEND_FIFO_TIMEOUT_MS 100

/**
 * Receives a message from the driver station
 */
template <typename SendType, typename ReceiveType>
SendOutput Radio<SendType, ReceiveType>::Send(SendType &message)
{
    // convert to char array
    char *msgChars = (char*) &message;

    // Send each byte of the array via Serial (which is connected to the radio)
    for (int i = 0; i < sizeof(message); i++)
    {
        Serial1.write(msgChars[i]);
    }

    Serial1.flush();

    Serial.println("Sent message");


    // return success
    return SEND_SUCCESS;
}

template <typename SendType, typename ReceiveType>
ReceiveType Radio<SendType, ReceiveType>::Receive()
{
    ReceiveType receiveMessage;
    // Clear the receiveMessage object
    memset(&receiveMessage, 0, sizeof(ReceiveType));

    Serial.println("Available: " + String(Serial1.available()));

    // Check if there is enough data in the buffer
    if (Serial1.available() >= sizeof(ReceiveType))
    {
        // Read the message
        size_t bytesRead = Serial1.readBytes((char *)&receiveMessage, sizeof(ReceiveType));

        // Check if the read was successful
        if (bytesRead == sizeof(ReceiveType))
        {
            return receiveMessage; // Data was read successfully
        }
        else
        {
            // Handle the error, bytesRead was not as expected
            // You may want to clear the buffer or take other actions
            Serial.println("ERROR: bytesRead was not as expected");
        }
    }
    else
    {
        // Handle the error, not enough data in the buffer
        Serial.println("ERROR: not enough data in the radio buffer");
    }

    return receiveMessage; // Data was not read
}

template <typename SendType, typename ReceiveType>
bool Radio<SendType, ReceiveType>::Available()
{
    // Check if there is enough data in the buffer
    return Serial1.available() >= sizeof(ReceiveType);
}
