#pragma once
#include <RF24.h>

#define VERBOSE_RADIO
/**
 * Radio.h
 * 
 * This is used for both the transmitter and the receiver, and thus
 * should be the same on both. Please don't change just one.
*/

enum SendOutput {
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

    SendOutput Send(SendType& message);
    ReceiveType Receive();

    bool Available();

private:
    RF24 radio{7, 9}; // CE, CSN
};

template <typename SendType, typename ReceiveType>
Radio<SendType, ReceiveType>::Radio()
{
    InitRadio();
}

template <typename SendType, typename ReceiveType>
void Radio<SendType, ReceiveType>::InitRadio()
{
#ifdef VERBOSE_RADIO
    Serial.println("Initializing radio");
#endif
    const byte address[6] = "00001";
    radio.begin();
    radio.openReadingPipe(1, address);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_1MBPS);
}

unsigned long lastTimeReceivedMessage = 0;

#define SEND_FIFO_TIMEOUT_MS 100

/**
 * Receives a message from the driver station
 */
template <typename SendType, typename ReceiveType>
SendOutput Radio<SendType, ReceiveType>::Send(SendType &message)
{
    radio.stopListening();
    radio.write(&message, sizeof(SendType));
    SendOutput ret = SEND_SUCCESS;

    // check time
    unsigned long currentTime = millis();
    // Wait for fifo to be empty
    // while NOT (the transmitting fifo (first arg) is empty (second arg))
    while (!radio.isFifo(true, true))
    {
        // spin

        // error check
        if (millis() - currentTime > SEND_FIFO_TIMEOUT_MS)
        {
#ifdef VERBOSE_RADIO
            Serial.println("Radio fifo failed to clear");
#endif
            // reinit
            InitRadio();
            ret = FIFO_FAIL;
            break;
        }
    }

    // if the radio has failed
    if (radio.failureDetected)
    {
#ifdef VERBOSE_RADIO
        Serial.println("Radio hardware failure detected");
#endif
        // reinit
        InitRadio();
        // reset failure flag
        radio.failureDetected = false;
        ret = HW_FAULT;
    }

    radio.startListening();
    return ret;
}

template <typename SendType, typename ReceiveType>
ReceiveType Radio<SendType, ReceiveType>::Receive()
{
    ReceiveType receiveMessage;
    // initialize to 0
    memset(&receiveMessage, 0, sizeof(ReceiveType));
    // Serial.println("is radio available" + (String)radio.available());
    if (radio.available())
    {
        radio.read(&receiveMessage, sizeof(ReceiveType));
        Serial.println("Received message");
    }

    return receiveMessage;
}

template <typename SendType, typename ReceiveType>
bool Radio<SendType, ReceiveType>::Available()
{
    return radio.available();
}
