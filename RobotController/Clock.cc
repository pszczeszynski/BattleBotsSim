#include "Clock.h"


Clock::Clock()
{
	// initialize the start time
	startTime = std::chrono::high_resolution_clock::now();
	// initialize the last reset time
	lastResetTime = startTime;
}

/**
 * \brief
 * Marks the start time for reference
 */
void Clock::markStart()
{
	auto currentTime = std::chrono::high_resolution_clock::now();

	startTime = currentTime;

	double elapsedTimeSinceLastReset = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastResetTime).count();
	if (elapsedTimeSinceLastReset >= 1.0) // One second has passed
	{
		resetCounters();
		lastResetTime = currentTime;
	}
}

void Clock::markEnd()
{
	double timeDifference = getElapsedTime();
	// every 3 seconds, reset the average time
	if (_totalTime > 3.0)
	{
		_totalTime = 0.0;
		_totalFrames = 0;
	}
	if (timeDifference > maxTimeDifference)
	{
		maxTimeDifference = timeDifference;
	}

	_totalTime += timeDifference;
	++_totalFrames;
}

/**
 * \brief
 * Gets the elapsed time since markStart() was called in seconds
 */
double Clock::getElapsedTime()
{
	auto currentTime = std::chrono::high_resolution_clock::now();
	return std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
}

void Clock::resetCounters()
{
	_lastMaxTimeDifference = maxTimeDifference;
	maxTimeDifference = 0.0;
}

double Clock::getAverageTime()
{
	if (_totalFrames == 0)
	{
		return 0.0;
	}

	return _totalTime / _totalFrames;
}