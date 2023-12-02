#pragma once
#include "CircularDeque.h"
#include "Communication.h"
#include <functional>

// #define COMPUTER
#ifdef COMPUTER
#include <iostream>

#include "Clock.h"
#endif

// Allows printing and turning off all prints
#define VERBOSE_MODE 0
#define DEBUG_PRINT(x) \
    if (VERBOSE_MODE)         \
    std::cout << x

#define DEBUG_ERROR(x) \
    if (false) \
    std::cerr << x

template <typename T>
class GenericReceiver
{
    int messages_processed = 0;

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
        DEBUG_PRINT("GenericReceiver initialized with buffer size: " << buffer_size << std::endl);
    }

private:
    std::function<bool(char&)> readChar;


    void readUntilNextPacket()
    {
        char c;

        // read until we get a message
        while (!latestDataValid)
        {
#ifdef COMPUTER
            Clock readClock;
            Clock loopClock;
            double waitingTime = 0;
            int loopCount = 0;
#endif
            // keep calling read char until we get a character
            while (!readChar(c))
            {
#ifdef COMPUTER
                loopCount ++;
                waitingTime += loopClock.getElapsedTime();
                loopClock.markStart();
#endif
            }

#ifdef COMPUTER
            std::cout << "loop count: " << loopCount << std::endl;
            std::cout << "waiting time: " << waitingTime * 1000 << "ms" << std::endl;
            // if (readClock.getElapsedTime() > 0.02)
            // {
                std::cout << "TOTAL TIME GETTING DATA " << readClock.getElapsedTime() * 1000 << "ms" << std::endl;
            // }
#endif

            DEBUG_PRINT("Read char: " << c << std::endl);
            switch (currentState)
            {
                case State::Idle:
                    DEBUG_PRINT("State: Idle\n");
                    if (c == MESSAGE_START_CHAR)
                    {
                        DEBUG_PRINT("Start char received. Switching to Receiving state.\n");
                        currentState = State::Receiving;
                        serialBuffer.clear(); // Clear buffer to start fresh
                    }
                    else
                    {
                        break; // If not start char, stay in Idle
                    }

                case State::Receiving:
                    DEBUG_PRINT("State: Receiving\n");
                    serialBuffer.push_back(c);
                    if (c == MESSAGE_END_CHAR)
                    {
                        DEBUG_PRINT("End char received.\n");
                        if (serialBuffer.Size() == sizeof(T) + 2) // Including start and end char
                        {
                            DEBUG_PRINT("Correct message size. Switching to MessageComplete state.\n");
                            currentState = State::MessageComplete;
                        }
                        else
                        {
                            DEBUG_ERROR("Error: Message size mismatch. Expected size: " << sizeof(T) + 2 << ", Actual size: " << serialBuffer.Size() << std::endl);
                            currentState = State::Error; // Message size mismatch
                        }
                    }
                    else if (serialBuffer.Size() > sizeof(T) + 2)
                    {
                        DEBUG_ERROR("Error: Buffer overflow. Expected max size: " << sizeof(T) + 2 << ", Current size: " << serialBuffer.Size() << std::endl);
                        currentState = State::Error; // Buffer overflow
                    }
                    else
                    {
                        break; // Normal case, continue receiving
                    }

                case State::MessageComplete:
                    DEBUG_PRINT("State: MessageComplete\n");
                    processMessage();
                    currentState = State::Idle; // Reset state
                    break;

                case State::Error:
                    DEBUG_ERROR("State: Error\n");
                    serialBuffer.clear();
                    currentState = State::Idle; // Reset state after error
                    break;
            }
        }
    }

    void processMessage()
    {
        DEBUG_PRINT("Processing message...\n");
        // Sanity check for buffer size
        if (serialBuffer.Size() != sizeof(T) + 2)
        {
            // std::cerr << "Warning: Unexpected buffer size during message processing. Expected size: " << sizeof(T) + 2 << ", Actual size: " << serialBuffer.Size() << std::endl;
        }

        // Assuming message format: [START_CHAR][DATA][END_CHAR]
        serialBuffer.pop_front(1); // Remove start char
        serialBuffer.pop_back(); // Remove end char
        serialBuffer.copy_to((char *)&latestData, 0, sizeof(T));
        latestDataValid = true;
        DEBUG_PRINT("Message processed successfully. Buffer cleared.\n");
        serialBuffer.clear(); // Clear buffer after processing message

        messages_processed++;
        if (messages_processed % 100 == 0)
        {
            std::cout << "Messages processed: " << messages_processed << std::endl;
        }
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
        // DEBUG_PRINT("Updating receiver...\n");
        readUntilNextPacket();
        // DEBUG_PRINT("Update complete. Latest data valid: " << std::boolalpha << latestDataValid << std::endl);
    }
};
