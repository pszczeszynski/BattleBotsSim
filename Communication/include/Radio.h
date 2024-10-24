#pragma once
#include <RF24.h>
#include "Communication.h"
#include "Hardware.h"

// BELOW MUST CHANGE FOR EACH RECEIVER TEENSY
#define CHANNEL TEENSY_RADIO_1
#define POWER RF24_PA_MAX
#define POWER_STATUS_MSG "Setting power to HIGH"
#define VERBOSE_RADIO

/**
 * Radio.h
 * 
 * This is used for both the transmitter and the receiver.
 * It defines the Radio class, which is used to send and receive messages
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
    Radio(uint8_t channel);
    bool InitRadio(uint8_t channel);

    SendOutput Send(SendType& message);
    ReceiveType Receive();
    void SetChannel(unsigned char channel);

    bool Available();

private:
    RF24 radio{RADIO_CE_PIN, RADIO_CS_PIN};
    unsigned char _channel = CHANNEL;
};

template <typename SendType, typename ReceiveType>
Radio<SendType, ReceiveType>::Radio()
{
    InitRadio(TEENSY_RADIO_1);
}

template <typename SendType, typename ReceiveType>
Radio<SendType, ReceiveType>::Radio(uint8_t channel)
{
    InitRadio(channel);
}

template <typename SendType, typename ReceiveType>
bool Radio<SendType, ReceiveType>::InitRadio(uint8_t channel)
{
    const byte address[6] = "00001";
    if (!radio.begin()) return false;
    radio.openReadingPipe(1, address);
    radio.openWritingPipe(address);
    radio.setPALevel(POWER);
    radio.startListening();
    radio.setAutoAck(false);
    if (!radio.setDataRate(RF24_2MBPS)) return false;
    radio.setChannel(channel);
    return true;
}

#define SEND_FIFO_TIMEOUT_MS 1

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
            InitRadio(_channel);
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
        InitRadio(_channel);
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

template <typename SendType, typename ReceiveType>
void Radio<SendType, ReceiveType>::SetChannel(unsigned char channel)
{
    if (channel == _channel)
    {
        return;
    }

    radio.setChannel(channel);

    _channel = channel;
}
