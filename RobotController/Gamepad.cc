#include "Gamepad.h"

Gamepad::Gamepad(int index) : controllerIndex(index) {}

void Gamepad::Update()
{
    ZeroMemory(&controllerState, sizeof(XINPUT_STATE));
    XInputGetState(controllerIndex, &controllerState);
}

bool Gamepad::GetButtonA()
{
    return (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_A) != 0;
}

bool Gamepad::GetButtonB()
{
    return (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0;
}

bool Gamepad::GetButtonX()
{
    return (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_X) != 0;
}

bool Gamepad::GetButtonY()
{
    return (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_Y) != 0;
}

bool Gamepad::GetLeftBumper()
{
    return (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0;
}

bool Gamepad::GetRightBumper()
{
    return (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
}

#define RANGE 32767.0f

float Gamepad::GetLeftStickX()
{
    return controllerState.Gamepad.sThumbLX / RANGE;
}

float Gamepad::GetLeftStickY()
{
    return controllerState.Gamepad.sThumbLY / RANGE;
}

float Gamepad::GetRightStickX()
{
    return controllerState.Gamepad.sThumbRX / RANGE;
}

float Gamepad::GetRightStickY()
{
    return controllerState.Gamepad.sThumbRY / RANGE;
}
