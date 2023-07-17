
#pragma once
#include <Servo.h>
#include <Arduino.h>

class Motor
{
public:
    Motor(int pwmPin);
    void SetPower(double speed);
private:
    Servo motor;
};