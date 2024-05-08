#include <iostream>
#include <queue>
#include <functional>
#include "GenericReceiver.h"

// EXPECTED OUTPUT:
// Received data: 1230

// Test data structure
struct TestStruct
{
    int data;
};

// Simulate a serial port with a queue
std::queue<char> simulateSerialPort()
{
    std::queue<char> serialQueue;

    // Generate test data
    for (int i = 1; i <= 10; i++)
    {
        // Prepare a TestStruct with increasing integers
        TestStruct ts;
        ts.data = i * 123;

        // Separate messages with MESSAGE_START_CHAR and MESSAGE_END_CHAR
        serialQueue.push(MESSAGE_START_CHAR);

        // Cast TestStruct to char and store in the queue
        char buffer[sizeof(TestStruct)];
        std::memcpy(buffer, &ts, sizeof(TestStruct));

        for (size_t j = 0; j < sizeof(TestStruct); j++)
        {
            serialQueue.push(buffer[j]);
        }

        serialQueue.push(MESSAGE_END_CHAR);
    }

    return serialQueue;
}


int main()
{
    std::queue<char> serialQueue = simulateSerialPort();

    // Initialize the GenericReceiver with buffer size of 100 and lambda to read a single character
    GenericReceiver<TestStruct> receiver(100, [&serialQueue](char& c)
                                         {
                                             if(serialQueue.empty()) 
                                             {
                                                 return false;
                                             }
                                             c = serialQueue.front();
                                             serialQueue.pop();
                                             return true;
                                         });

    // Continually read from the simulated serial port until the queue is empty
    while (!serialQueue.empty())
    {
        receiver.waitUntilData();

        if (receiver.isLatestDataValid())
        {
            TestStruct data = receiver.getLatestData();
            std::cout << "Received data: " << data.data << std::endl;
        }
    }

    return 0;
}
