#include "Clock.h"

/**
 * \brief
 * Marks the start time for reference
 */
void Clock::markStart()
{
	//record the current time
	startTime = std::chrono::high_resolution_clock::now();
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