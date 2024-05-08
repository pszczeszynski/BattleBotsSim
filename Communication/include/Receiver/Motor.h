
#pragma once
#include <Servo.h>
#include <Arduino.h>

class Motor
{
public:
    Motor(int pwmPin);
    void init();
    void SetPower(double speed);
private:
    Servo motor;
    double lastPower = 0;
    int _pwmPin;
};