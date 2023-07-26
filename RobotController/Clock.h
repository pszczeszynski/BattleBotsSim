/**
 * Use this for timing things
 */

#pragma once
#include <chrono>
#include "Clock.h"

class Clock
{
public:
	Clock();
	void markStart();
	double getElapsedTime();
private:
	//time the clock was started
	std::chrono::high_resolution_clock::time_point startTime;
};

