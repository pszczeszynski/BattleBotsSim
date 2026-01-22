#pragma once
#include "Clock.h"
#include "MathUtils.h"

#define TIME_WEIGHT_MS 100
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
        // save the current value
        currentValue = newValue;

        // start a timer
        lastElapsedTime = clock.getElapsedTime();
        clock.markStart();

        // compute derivative
        T newDerivative = T(newValue - lastValue) / lastElapsedTime;

        // if 100ms passes, it is 100% of the new value
        double weightFactor = (lastElapsedTime * 1000 / TIME_WEIGHT_MS);

        // weight the new difference in to the old difference with weight 1 / WEIGHTED_AVERAGE_SAMPLES
        weightedDerivative = newDerivative * weightFactor + weightedDerivative * (1 - weightFactor);

        // save the last value for next time
        lastValue = newValue;

        return newValue;
    }

    T Extrapolate(double time)
    {
        T extrapolatedValue = T(currentValue + T(weightedDerivative * time));
        return extrapolatedValue;
    }

    T GetVelocity()
    {
        return weightedDerivative;
    }

    void LogDebugInfo()
    {
        loggingDebugInfo = true;
    }
    void StopLoggingDebugInfo()
    {
        loggingDebugInfo = false;
    }

private:
    T weightedDerivative;
    T lastValue;
    T currentValue;
    Clock clock;
    double lastElapsedTime;

    bool loggingDebugInfo = false;
};

class AngleExtrapolator
{
public:
    AngleExtrapolator(double initialValue)
    {
        lastValue = initialValue;
        clock.markStart();
    }

    double SetValue(double newValue)
    {
        // save the current value
        currentValue = newValue;

        // start a timer
        lastElapsedTime = clock.getElapsedTime();
        clock.markStart();

        // compute derivative
        double newDerivative = angle_wrap(newValue - lastValue) / lastElapsedTime;

        // if 100ms passes, it is 100% of the new value
        double weightFactor = (lastElapsedTime * 1000 / TIME_WEIGHT_MS);

        // enforce a max weight factor of 1
        weightFactor = weightFactor <= 1 ? weightFactor : 1;

        // weight the new difference in to the old difference with weight 1 / WEIGHTED_AVERAGE_SAMPLES
        weightedDerivative = newDerivative * weightFactor + weightedDerivative * (1 - weightFactor);

        // save the last value for next time
        lastValue = newValue;

        if (loggingDebugInfo)
        {
            printf("    Extrapolator: this elapsed time: %f, new derivative deg/s: %f\n", lastElapsedTime, TO_DEG * weightedDerivative);
        }

        return newValue;
    }

    double Extrapolate(double time)
    {
        return angle_wrap(currentValue + weightedDerivative * time);
    }

    double GetVelocity()
    {
        return weightedDerivative;
    }

    void LogDebugInfo()
    {
        loggingDebugInfo = true;
    }
    void StopLoggingDebugInfo()
    {
        loggingDebugInfo = false;
    }

private:
    double currentValue;
    double weightedDerivative;
    double lastValue;
    Clock clock;
    double lastElapsedTime;

    bool loggingDebugInfo = false;
};

// the state of the robot system
struct RobotSimState
{
    cv::Point2f velocity;
    double angularVelocity;
    cv::Point2f position;
    double angle;
};

/**
 * @class RobotSimulator
 * Simulates the robot's movement in many iterations.
 * This is used to extrapolate the robot's position and angle into the future.
 */
class RobotSimulator
{
public:
    RobotSimulator()
    {
    }

private:
    /**
     * Simulates the state of the robot for a single step using linear extrapolation
     */
    RobotSimState SimulateStep(RobotSimState &initialState, double timeSeconds)
    {
        RobotSimState newState;

        // compute the new position
        newState.position = initialState.position + initialState.velocity * timeSeconds;

        double deltaAngle = initialState.angularVelocity * timeSeconds;

        // rotate curr velocity by angular angular deltaAngle
        newState.velocity = rotate_point(initialState.velocity, deltaAngle);

        // compute the new angle
        newState.angle = angle_wrap(initialState.angle + deltaAngle);

        // compute the new angular velocity
        newState.angularVelocity = initialState.angularVelocity;


        // return the new state
        return newState;
    }

public:
    /**
     * Simulates the state of the robot over a given time period
     *
     * @param initialState The initial state of the robot
     * @param timeSeconds The time period to simulate over
     * @param steps The number of steps to simulate (more = more accurate but slower)
     */
    RobotSimState Simulate(RobotSimState &initialState, double timeSeconds, int steps)
    {
        // start with the initial state
        RobotSimState currentState = initialState;

        // for each step
        for (int i = 0; i < steps; i++)
        {
            // simulate a single step
            currentState = SimulateStep(currentState, timeSeconds / steps);
        }

        // return the final state
        return currentState;
    }
};