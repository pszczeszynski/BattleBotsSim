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
	void markEnd();
	double getElapsedTime();
	double getMaxTimeDifference() const { return _lastMaxTimeDifference; }

	double getAverageTime();

private:
	void resetCounters();
	std::chrono::high_resolution_clock::time_point startTime;
	std::chrono::high_resolution_clock::time_point lastResetTime;

	double maxTimeDifference = 0.0; // Stores the maximum time difference between markStart calls

	double _lastMaxTimeDifference = 0.0;

	double _totalTime = 0.0;
	int _totalFrames = 0;
};
