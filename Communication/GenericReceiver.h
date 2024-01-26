#pragma once
#include "CircularDeque.h"
#include "Communication.h"
#include <functional>

template <typename T>
class GenericReceiver
{
    enum class State
    {
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
    GenericReceiver(int buffer_size, std::function<bool(char &)> single_char_read_lambda)
        : serialBuffer(buffer_size), readChar(single_char_read_lambda)
    {
    }

private:
    std::function<bool(char &)> readChar;

    void _handleError()
    {
        serialBuffer.clear();
        currentState = State::Idle; // Reset state
    }

    void _processChar(char c)
    {
        switch (currentState)
        {
        case State::Idle:
            if (c == MESSAGE_START_CHAR)
            {
                currentState = State::Receiving;
                serialBuffer.clear(); // Clear buffer to start fresh, fall through to Receiving
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
                    processMessage();
                    currentState = State::Idle; // Reset state
                }
                else
                {
                    // std::cout << "hit end char but shouldn't have. size of serial buffer: " << serialBuffer.Size() << std::endl;
                    // _handleError();
                }
            }
            else if (serialBuffer.Size() > sizeof(T) + 2)
            {
                // std::cout << "didn't find end char. size of serial buffer: " << serialBuffer.Size() << std::endl;
                _handleError();
            }

            break; // Normal case, continue receiving
        }
    }

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

            _processChar(c);
        }
    }

    void processMessage()
    {
        // Assuming message format: [START_CHAR][DATA][END_CHAR]
        serialBuffer.pop_front(); // Remove start char
        serialBuffer.pop_back();  // Remove end char
        int copy_result = serialBuffer.copy_to((char *)&latestData, 0, sizeof(T));

        if (copy_result != sizeof(T))
        {
            return;
        }

        latestDataValid = true;
        serialBuffer.clear(); // Clear buffer after processing message
    }

public:
    T getLatestData()
    {
        T &ret = latestData;
        latestDataValid = false;
        return ret;
    }

    bool isLatestDataValid() const
    {
        return latestDataValid;
    }

    void waitUntilData()
    {
        readUntilNextPacket();
    }

    bool readUntilEndOrNextPacket()
    {
        char c;
        while (readChar(c))
        {
            _processChar(c);
        }
    }
};
