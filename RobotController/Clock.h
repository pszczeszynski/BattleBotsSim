/**
 * Use this for timing things
 */

#pragma once
#include <chrono>
#include "Clock.h"

class Clock
{
public:
	static Clock programClock;
	
	Clock();
	virtual void markStart(double startOffset = 0); // Pass in a starting elapsed time
	virtual double markEnd();
	bool isRunning();
	double getElapsedTime();
	double getMaxTimeDifference() const { return _lastMaxTimeDifference; }
	double getAverageTime();
protected:
	bool _running = true;
private:
	double _offset = 0;
	void resetCounters();
	std::chrono::high_resolution_clock::time_point startTime;

	double timeDifference = 0;
	double maxTimeDifference = 0.0; // Stores the maximum time difference between markStart calls
	double _lastMaxTimeDifference = 0.0;

	double _totalTime = 0.0;
	int _totalFrames = 0;
};
