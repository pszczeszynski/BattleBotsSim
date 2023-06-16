#pragma once
#include "Clock.h"

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

        if (clock.getElapsedTime() > 0.05)
        {
            derivative = (newValue - lastValue) / clock.getElapsedTime();
            lastValue = newValue;
            clock.markStart();
        }

        return newValue;
    }

    T Extrapolate(double time)
    {
        T extrapolatedValue = currentValue + derivative * time;
        return extrapolatedValue;
    }

    T GetVelocity()
    {
        return derivative;
    }

private:
    T derivative;
    T lastValue;
    T currentValue;
    Clock clock;
};
