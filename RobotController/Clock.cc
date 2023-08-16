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
	double timeDifference = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();

	if (timeDifference > maxTimeDifference)
	{
		maxTimeDifference = timeDifference;
	}

	startTime = currentTime;
	++fps;

	double elapsedTimeSinceLastReset = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastResetTime).count();
	if (elapsedTimeSinceLastReset >= 1.0) // One second has passed
	{
		resetCounters();
		lastResetTime = currentTime;
	}
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
	_lastFPS = fps;
	_lastMaxTimeDifference = maxTimeDifference;

	fps = 0;
	maxTimeDifference = 0.0;
}
