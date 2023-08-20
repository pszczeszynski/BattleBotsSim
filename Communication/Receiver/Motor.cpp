#include "Motor.h"
#include <cmath>

Motor::Motor(int pwmPin)
{
    motor.attach(pwmPin);
}

#define PWM_MIN 1000
#define PWM_MAX 2000
#define SIG 0.01

void Motor::SetPower(double power)
{
    // if the power is the same as last time, don't do anything
    if (abs(power - lastPower) < SIG && !(power == 0 && lastPower != 0))
    {
        return;
    }

    // map the power to the pwm range
    int pwm = (power * (PWM_MAX - PWM_MIN) / 2) + (PWM_MAX + PWM_MIN) / 2;
    pwm = min(pwm, PWM_MAX);
    pwm = max(pwm, PWM_MIN);

    // write the pwm
    motor.writeMicroseconds(pwm);

    // save the power for next time
    lastPower = power;
}