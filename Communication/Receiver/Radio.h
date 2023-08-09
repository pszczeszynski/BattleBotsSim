#pragma once
#include <RF24.h>


/**
 * Radio.h
 * 
 * This is used for both the transmitter and the receiver, and thus
 * should be the same on both. Please don't change just one.
*/

template <typename SendType, typename ReceiveType>
class Radio
{
public:
    Radio();
    void InitRadio();

    void Send(SendType& message);
    ReceiveType Receive();

    bool Available();
private:
    RF24 radio{14, 10};
};

template <typename SendType, typename ReceiveType>
Radio<SendType, ReceiveType>::Radio()
{
    InitRadio();
}

template <typename SendType, typename ReceiveType>
void Radio<SendType, ReceiveType>::InitRadio()
{
    Serial.println("Initializing radio");
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

/**
 * Receives a message from the driver station
 */
template <typename SendType, typename ReceiveType>
void Radio<SendType, ReceiveType>::Send(SendType &message)
{
    radio.stopListening();
    radio.write(&message, sizeof(SendType));

    // Wait for fifo to be empty
    // while NOT (the transmitting fifo (first arg) is empty (second arg))
    while (!radio.isFifo(true, true))
    {
        // spin
    }
    radio.startListening();
}

template <typename SendType, typename ReceiveType>
ReceiveType Radio<SendType, ReceiveType>::Receive()
{
    ReceiveType receiveMessage;

    if (radio.available())
    {
        radio.read(&receiveMessage, sizeof(ReceiveType));
    }

    return receiveMessage;
}

template <typename SendType, typename ReceiveType>
bool Radio<SendType, ReceiveType>::Available()
{
    return radio.available();
}
