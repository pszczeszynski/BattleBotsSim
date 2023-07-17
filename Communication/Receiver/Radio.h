#pragma once
#include <Arduino.h>
#include "Communication.h"
#include <RF24.h>

class Radio
{
public:
    Radio();
    void Send(RobotMessage& message);

    DriveCommand Receive();

    bool Available();
private:
    RF24 radio{14, 10};

};