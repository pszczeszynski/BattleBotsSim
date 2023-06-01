#include "Timer.h"


Timer::Timer()
{
    Start();
}

void Timer::Start()
{
	//record the current time
	t1 = std::chrono::high_resolution_clock::now();
}

double Timer::GetTimeMS()
{
	auto currentTime = std::chrono::high_resolution_clock::now();
	return std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - t1).count();
}
