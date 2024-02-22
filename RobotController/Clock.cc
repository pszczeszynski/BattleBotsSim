#include "Clock.h"


Clock Clock::programClock;  // Definition of the static member

//
// Clock tracks the elapsed time from when markStart is called and markEnd
// The constructor calls markStart and multiple markStarts can be called to reset elapsed time.
// elapsedTime will give the current elapsed time if the coutner has been started, or the last elapsed time when markEnd was called.
Clock::Clock()
{
	markStart(); // start the timer
				 // initialize the last reset time
				 // lastResetTime = startTime; // was never used
}

/**
 * \brief
 * Marks the start time for reference
 *
 * \param startOffset The offset to add to the start time
 */
void Clock::markStart(double startOffset)
{
	startTime = std::chrono::high_resolution_clock::now();
	_offset = startOffset;
	timeDifference = 0;
	_running = true;
}

/**
 * \brief
 * Running tracks if markStart and markEnd were called
 */
bool Clock::isRunning()
{
	return _running;
}

/**
 * \brief
 * Marks the end time and returns the time difference in seconds if running, otherwise returns the previous timeDifference
 */
double Clock::markEnd()
{
	// Only mark end if previously running
	if (!_running)
	{
		return timeDifference;
	};

	timeDifference = getElapsedTime();

	if (timeDifference > maxTimeDifference)
	{
		maxTimeDifference = timeDifference;
	}

	_totalTime += timeDifference;
	++_totalFrames;

	// every 1 second, reset the average time
	if (_totalTime > 1.0)
	{
		resetCounters();
	}

	_running = false;

	return timeDifference;
}

/**
 * \brief
 * Gets the elapsed time since markStart() was called in seconds
 */
double Clock::getElapsedTime()
{
	// Return last time difference if the clock isn't running
	if (!_running)
	{
		return timeDifference;
	}

	// Otherwise return the current time difference
	auto currentTime = std::chrono::high_resolution_clock::now();
	return std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count() + _offset;
}

void Clock::resetCounters()
{
	_lastMaxTimeDifference = maxTimeDifference;
	maxTimeDifference = 0.0;
	_totalTime = 0.0;
	_totalFrames = 0;
}

double Clock::getAverageTime()
{
	if (_totalFrames == 0)
	{
		return 0.0;
	}

	return _totalTime / _totalFrames;
}
