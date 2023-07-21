#include "CircularDeque.h"
#include <iostream>
void testCircularDeque()
{
    int testCount = 0;
    int passedCount = 0;

    auto printTestResult = [&](bool condition, const std::string &testName)
    {
        testCount++;
        if (condition)
        {
            std::cout << "Test " << testCount << ": " << testName << " - PASSED\n";
            passedCount++;
        }
        else
        {
            std::cout << "Test " << testCount << ": " << testName << " - FAILED\n";
        }
    };
    // Test case 1: Create a deque with capacity 5
    CircularDeque<int> deque(5);
    printTestResult(deque.isEmpty(), "Create deque with capacity 5 - initial state");

    // Test case 2: Push elements to the back
    deque.push_back(10);
    deque.push_back(20);
    deque.push_back(30);
    printTestResult(!deque.isEmpty(), "Push elements to the back - deque not empty");
    printTestResult(!deque.isFull(), "Push elements to the back - deque not full");

    // Test case 3: Pop elements from the front
    deque.pop_front(2);
    printTestResult(!deque.isEmpty(), "Pop elements from the front - deque not empty");
    printTestResult(!deque.isFull(), "Pop elements from the front - deque not full");
    deque.pop_front(1);
    printTestResult(deque.isEmpty(), "Pop elements from the front - deque empty");
    printTestResult(!deque.isFull(), "Pop elements from the front - deque not full");

    // Test case 4: Push more elements to the back
    deque.push_back(40);
    deque.push_back(50);
    deque.push_back(60);
    deque.push_back(70);
    deque.push_back(80);
    printTestResult(!deque.isEmpty(), "Push more elements to the back - deque not empty");
    printTestResult(deque.isFull(), "Push more elements to the back - deque is full");

    // Test case 5: Pop elements from the front until empty
    deque.pop_front(5);
    printTestResult(deque.isEmpty(), "Pop elements from the front until empty - deque empty");
    printTestResult(!deque.isFull(), "Pop elements from the front until empty - deque not full");

    // Test case 6: Push elements again to test circular behavior
    deque.push_back(90);
    deque.push_back(100);
    deque.push_back(110);
    deque.push_back(120);
    printTestResult(!deque.isEmpty(), "Push elements again - deque not empty");
    printTestResult(!deque.isFull(), "Push elements again - deque not full");

    // Test case 7: Pop elements from the front partially
    deque.pop_front(2);
    printTestResult(!deque.isEmpty(), "Pop elements from the front partially - deque not empty");
    printTestResult(!deque.isFull(), "Pop elements from the front partially - deque not full");

    // Test case 8: Push elements until full
    deque.push_back(130);
    deque.push_back(140);
    deque.push_back(150);
    printTestResult(!deque.isEmpty(), "Push elements until full - deque not empty");
    printTestResult(deque.isFull(), "Push elements until full - deque is full");

    // Test case 9: Push element when the deque is full (should not add)
    deque.push_back(150);
    printTestResult(!deque.isEmpty(), "Push element when deque is full - deque not empty");
    printTestResult(deque.isFull(), "Push element when deque is full - deque is full");

    // Test case 10: Pop all elements until empty
    deque.pop_front(6);
    printTestResult(deque.isEmpty(), "Pop all elements until empty - deque empty");
    printTestResult(!deque.isFull(), "Pop all elements until empty - deque not full");

    // Test case 11: Pop elements from an empty deque (should not throw error)
    deque.pop_front(1);
    printTestResult(deque.isEmpty(), "Pop elements from an empty deque - deque empty");
    printTestResult(!deque.isFull(), "Pop elements from an empty deque - deque not full");

    std::cout << "\nPassed " << passedCount << " out of " << testCount << " tests.\n";
}

// Define a struct of two doubles
struct TwoDoubles
{
    double a;
    double b;

    // Define an equality operator for convenience in testing
    bool operator==(const TwoDoubles &other) const
    {
        return a == other.a && b == other.b;
    }
};

void testCopyTo()
{
    // Create a TwoDoubles object and initialize it
    TwoDoubles original;
    original.a = 3.14159;
    original.b = 2.71828;

    // Create a CircularDeque of chars
    CircularDeque<char> deque(sizeof(TwoDoubles));

    // push back some random chars just to offset
    deque.push_back('a');
    deque.push_back('b');
    deque.push_back('c');
    deque.push_back('d');
    deque.push_back('e');
    deque.push_back('f');

    // Add original to the deque, interpreting it as chars
    const char *originalAsChars = reinterpret_cast<const char *>(&original);
    for (size_t i = 0; i < sizeof(TwoDoubles); i++)
    {
        deque.push_back(originalAsChars[i]);
    }

    // Create a new TwoDoubles object and clear it to zero
    TwoDoubles copy;
    std::memset(&copy, 0, sizeof(TwoDoubles));

    // Use copy_to to copy data from the deque to copy
    deque.copy_to(reinterpret_cast<char *>(&copy), 0, sizeof(TwoDoubles));

    // Check if original and copy are the same
    std::cout << "Test copy_to: ";
    if (original == copy)
    {
        std::cout << "PASSED\n";
    }
    else
    {
        std::cout << "FAILED\n";
    }
}

int main()
{
    testCopyTo();
    testCircularDeque();
    return 0;
}
