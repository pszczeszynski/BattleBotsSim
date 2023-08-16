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
	int getFPS() const { return _lastFPS; }
	double getMaxTimeDifference() const { return _lastMaxTimeDifference; }

private:
	void resetCounters();
	std::chrono::high_resolution_clock::time_point startTime;
	std::chrono::high_resolution_clock::time_point lastResetTime;

	int fps = 0;					// Stores the frames per second
	double maxTimeDifference = 0.0; // Stores the maximum time difference between markStart calls

	int _lastFPS = 0;
	double _lastMaxTimeDifference = 0.0;
};
