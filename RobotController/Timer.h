
#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <vector>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

class Timer
{
    std::chrono::high_resolution_clock::time_point t1;

public:
	Timer();
    void Start();
    double GetTimeMS();
};

#endif //TIMER_H
