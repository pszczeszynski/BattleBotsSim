#pragma once
#include "CircularDeque.h"
#include "Communication.h"
#include <functional>

template <typename T>
class GenericReceiver
{
    T latestData;
    bool latestDataValid = false;
    CircularDeque<char> serialBuffer;

public:
    GenericReceiver(int buffer_size, std::function<bool(char&)> single_char_read_lambda)
        : serialBuffer(buffer_size), readChar(single_char_read_lambda) {}

private:
    std::function<bool(char&)> readChar;

    void readData()
    {
        latestDataValid = false;
        char c;
        
        while (readChar(c))
        {
            serialBuffer.push_back(c);
        }

        bool dataProcessed = true;
        while ((serialBuffer.Size() > sizeof(T)) && dataProcessed)
        {
            dataProcessed = false;

            unsigned int bufferStartIndex = 0;
            for (; bufferStartIndex < serialBuffer.Size(); bufferStartIndex++)
            {
                if (serialBuffer[bufferStartIndex] == MESSAGE_START_CHAR)
                {
                    bufferStartIndex += 1;
                    break;
                }
            }

            unsigned int bufferEndIndex = serialBuffer.Size();
            if (bufferStartIndex < serialBuffer.Size())
            {
                for (bufferEndIndex = bufferStartIndex + sizeof(T); bufferEndIndex < serialBuffer.Size(); bufferEndIndex++)
                {
                    if (serialBuffer[bufferEndIndex] == MESSAGE_END_CHAR)
                    {
                        break;
                    }
                }
            }

            if (bufferEndIndex < serialBuffer.Size())
            {
                serialBuffer.copy_to((char *)&latestData, bufferStartIndex, sizeof(T));
                dataProcessed = true;
                serialBuffer.pop_front(bufferEndIndex + 1);
                latestDataValid = true;
            }
        }
    }

public:
    T getLatestData() const
    {
        return latestData;
    }

    bool isLatestDataValid() const
    {
        return latestDataValid;
    }

    void update()
    {
        readData();
    }
};
