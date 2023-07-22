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
    static double lastPower = 0;

    // if the power is the same as last time, don't do anything
    if (abs(power - lastPower) < SIG)
    {
        // Serial.println("No change");
        return;
    }

    // map the power to the pwm range
    int pwm = (power * 500.0) + 1500;

    //int pwm = map(power, -1, 1, PWM_MIN, PWM_MAX);      <-- Map doesn't work with floats

    //print data to serial
    // Serial.println(power);
    // Serial.println(pwm);

    // write the pwm to the motor
    motor.writeMicroseconds(pwm);

    // save the power for next time
    lastPower = power;
}