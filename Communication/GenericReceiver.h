#pragma once
#include "CircularDeque.h"
#include "Communication.h"
#include <functional>

template <typename T>
class GenericReceiver
{
    enum class State {
        Idle,
        Receiving,
        MessageComplete,
        Error
    };

    State currentState = State::Idle;
    CircularDeque<char> serialBuffer;

    T latestData;
    bool latestDataValid = false;

public:
    GenericReceiver(int buffer_size, std::function<bool(char&)> single_char_read_lambda)
        : serialBuffer(buffer_size), readChar(single_char_read_lambda) 
    {
    }

private:
    std::function<bool(char&)> readChar;


    void readUntilNextPacket()
    {
        char c;

        // read until we get a message
        while (!latestDataValid)
        {
            // keep calling read char until we get a character
            while (!readChar(c))
            {
            }

            switch (currentState)
            {
                case State::Idle:
                    if (c == MESSAGE_START_CHAR)
                    {
                        currentState = State::Receiving;
                        serialBuffer.clear(); // Clear buffer to start fresh
                    }
                    else
                    {
                        break; // If not start char, stay in Idle
                    }

                case State::Receiving:
                    serialBuffer.push_back(c);
                    if (c == MESSAGE_END_CHAR)
                    {
                        if (serialBuffer.Size() == sizeof(T) + 2) // Including start and end char
                        {
                            currentState = State::MessageComplete;
                        }
                        else
                        {
                            currentState = State::Error; // Message size mismatch
                        }
                    }
                    else if (serialBuffer.Size() > sizeof(T) + 2)
                    {
                        currentState = State::Error; // Buffer overflow
                    }
                    else
                    {
                        break; // Normal case, continue receiving
                    }

                case State::MessageComplete:
                    processMessage();
                    currentState = State::Idle; // Reset state
                    break;

                case State::Error:
                    serialBuffer.clear();
                    currentState = State::Idle; // Reset state after error
                    break;
            }
        }
    }

    void processMessage()
    {
        // Assuming message format: [START_CHAR][DATA][END_CHAR]
        serialBuffer.pop_front(1); // Remove start char
        serialBuffer.pop_back(); // Remove end char
        serialBuffer.copy_to((char *)&latestData, 0, sizeof(T));
        latestDataValid = true;
        serialBuffer.clear(); // Clear buffer after processing message
    }

public:
    T getLatestData()
    {
        T& ret = latestData;
        latestDataValid = false;
        return ret;
    }

    bool isLatestDataValid() const
    {
        return latestDataValid;
    }

    void update()
    {
        readUntilNextPacket();
    }
};
