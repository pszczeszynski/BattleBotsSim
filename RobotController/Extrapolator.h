#pragma once
#include "Clock.h"

#define WEIGHTED_AVERAGE_SAMPLES 5
template <typename T>
class Extrapolator
{
public:
    Extrapolator(T initialValue)
    {
        lastValue = initialValue;
        clock.markStart();
    }

    T SetValue(T newValue)
    {
        currentValue = newValue;

        T newDifference = T(newValue - lastValue);
        weightedAverageDifference = T(newDifference - weightedAverageDifference) * (1.0 / WEIGHTED_AVERAGE_SAMPLES) + weightedAverageDifference * (1.0 - (1.0 / WEIGHTED_AVERAGE_SAMPLES));
        lastValue = newValue;

        lastElapsedTime = clock.getElapsedTime();
        clock.markStart();

        return newValue;
    }

    T Extrapolate(double time)
    {
        T extrapolatedValue = T(currentValue + T(weightedAverageDifference * (time / lastElapsedTime)));
        return extrapolatedValue;
    }

    T GetVelocity()
    {
        return weightedAverageDifference;
    }

private:
    T weightedAverageDifference;
    T lastValue;
    T currentValue;
    Clock clock;
    double lastElapsedTime;
};
