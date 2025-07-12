
#pragma once
#include <Arduino.h>
#include <Servo.h>

class Motor {
public:
  Motor(int pwmPin);
  void init();
  void SetPower(double speed);

private:
  Servo motor;
  double lastPower = 0;
  int _pwmPin;
};