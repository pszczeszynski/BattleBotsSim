// #include "Servo.h"
#include "Motor.h"
#include <cmath>
// for map



Motor::Motor(int pwmPin)
{
    motor.attach(pwmPin);
}

//Ask Ben about fwd/bkwd
#define PWM_MIN 1400
#define PWM_MAX 1600
#define SIG 0.01

void Motor::SetPower(double power)
{
    // if the power is the same as last time, don't do anything
    if (abs(power - lastPower) < SIG)
    {
        return;
    }

    // map the power to the pwm range
    int pwm = (power * 200.0) + 1500;

    // write the pwm to the motor
    motor.writeMicroseconds(pwm);
    Serial.println("pwm");
    Serial.println(pwm);

    // save the power for next time
    lastPower = power;
}