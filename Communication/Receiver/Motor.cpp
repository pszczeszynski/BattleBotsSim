#include "Motor.h"
#include <cmath>
// for map

Motor::Motor(int pwmPin)
{
    motor.attach(pwmPin);
}

// Ask Ben about fwd/bkwd
#define PWM_MIN 1450
#define PWM_MAX 1550
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
    if (pwm > PWM_MAX)
    {
        pwm = PWM_MAX;
    }
    else if (pwm < PWM_MIN)
    {
        pwm = PWM_MIN;
    }

    motor.writeMicroseconds(pwm);
    Serial.println("pwm");
    Serial.println(pwm);

    // save the power for next time
    lastPower = power;
}