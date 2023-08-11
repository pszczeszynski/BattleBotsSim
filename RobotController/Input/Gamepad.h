#pragma once
#include <windows.h>
#include <XInput.h>

class Gamepad
{
public:
    Gamepad(int index);
    void Update();

    // Button checkers
    bool GetButtonA();
    bool GetButtonB();
    bool GetButtonX();
    bool GetButtonY();
    bool GetLeftBumper();
    bool GetRightBumper();

    // Stick values, range from -1.0 to 1.0
    float GetLeftStickX();
    float GetLeftStickY();
    float GetRightStickX();
    float GetRightStickY();

private:
    int controllerIndex;
    XINPUT_STATE controllerState;
};
