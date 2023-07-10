#pragma once
#include "Clock.h"

#define UPDATES_PER_DERIVATIVE_CHANGE 3
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

        if (updatesSinceLastDerivativeChange > UPDATES_PER_DERIVATIVE_CHANGE)
        {
            updatesSinceLastDerivativeChange = 0;
            derivative = T(T(newValue - lastValue) / clock.getElapsedTime());
            lastValue = newValue;
            clock.markStart();
        }
        updatesSinceLastDerivativeChange++;

        return newValue;
    }

    T Extrapolate(double time)
    {
        T extrapolatedValue = currentValue + T(derivative * time);
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
    int updatesSinceLastDerivativeChange = 0;
};
