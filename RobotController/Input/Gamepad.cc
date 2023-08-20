#include "Gamepad.h"

Gamepad::Gamepad() {}


XBox::XBox(int index) : _controllerIndex(index){}

void XBox::Update()
{
    ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));
    XInputGetState(_controllerIndex, &_controllerState);
}

bool XBox::GetButtonA()
{
    return (_controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_A) != 0;
}

bool XBox::GetButtonB()
{
    return (_controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0;
}

bool XBox::GetButtonX()
{
    return (_controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_X) != 0;
}

bool XBox::GetButtonY()
{
    return (_controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_Y) != 0;
}

bool XBox::GetLeftBumper()
{
    return (_controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0;
}

bool XBox::GetRightBumper()
{
    return (_controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
}

float XBox::GetLeftStickX()
{
    return _controllerState.Gamepad.sThumbLX / RANGE;
}

float XBox::GetLeftStickY()
{
    return _controllerState.Gamepad.sThumbLY / RANGE;
}

float XBox::GetRightStickX()
{
    return _controllerState.Gamepad.sThumbRX / RANGE;
}

float XBox::GetRightStickY()
{
    return _controllerState.Gamepad.sThumbRY / RANGE;
}

float XBox::GetLeftTrigger()
{
    return _controllerState.Gamepad.bLeftTrigger / 255.0f;
}

float XBox::GetRightTrigger()
{
    return _controllerState.Gamepad.bRightTrigger / 255.0f;
}



DualSense::DualSense() 
{
    _controllersCount = 0;
    _initialized = false;

    // Call enumerate function and switch on return value
    switch (DS5W::enumDevices(_infos, 16, &_controllersCount)) {
        case DS5W_OK:
            std::cout << "DS5W_OK" << std::endl;
            break;

        // The buffer was not big enough. Ignore for now
        case DS5W_E_INSUFFICIENT_BUFFER:
            std::cout << "Insufficient Buffer" << std::endl;
            break;

        // Any other error will terminate the application
        default :
            std::cout << "Error Getting DualSense Info" << std::endl;
    }

    if (!_controllersCount) std::cout << "No Controllers Detected" << std::endl;
    
    InitController(0);
}

DualSense::~DualSense()
{
    // Free Device Context
    DS5W::freeDeviceContext(&_con);

}

bool DualSense::InitController(int index)
{
    // Init controller
    _initialized = !DS5W_FAILED(DS5W::initDeviceContext(&_infos[index], &_con));
    if (!_initialized) {
        std::cout << "Error Initializing Controller " << index << std::endl;
        return false;
    }
    return true;
}

void DualSense::Update()
{
    if (_initialized) DS5W::getDeviceInputState(&_con, &_inState); 
}

bool DualSense::GetButtonA()
{
    if (_initialized) return (_inState.buttonsAndDpad & DS5W_ISTATE_BTX_CROSS);
    return false;
}

bool DualSense::GetButtonB()
{
    if (_initialized) return (_inState.buttonsAndDpad & DS5W_ISTATE_BTX_CIRCLE);
    return false;
}

bool DualSense::GetButtonX()
{
    if (_initialized) return (_inState.buttonsAndDpad & DS5W_ISTATE_BTX_SQUARE);
    return false;
}

bool DualSense::GetButtonY()
{
    if (_initialized) return (_inState.buttonsAndDpad & DS5W_ISTATE_BTX_TRIANGLE);
    return false;
}

bool DualSense::GetLeftBumper()
{
    if (_initialized) return (_inState.buttonsA & DS5W_ISTATE_BTN_A_LEFT_BUMPER);
    return false;
}

bool DualSense::GetRightBumper()
{
    if (_initialized) return (_inState.buttonsA & DS5W_ISTATE_BTN_A_RIGHT_BUMPER);
    return false;
}

float DualSense::GetLeftStickX()
{
    if (_initialized) return INT8(_inState.leftStick.x) / RANGE;
    return 0;
}

float DualSense::GetLeftStickY()
{
    if (_initialized) return INT8(_inState.leftStick.y) / RANGE;
    return 0;
}

float DualSense::GetRightStickX()
{
    if (_initialized) return INT8(_inState.rightStick.x) / RANGE;
    return 0;
}

float DualSense::GetRightStickY()
{
    if (_initialized) return INT8(_inState.rightStick.y) / RANGE;
    return 0;
}

float DualSense::GetLeftTrigger()
{
    if (_initialized) return _inState.leftTrigger / 255.0f;
    return 0;
}

float DualSense::GetRightTrigger()
{
    if (_initialized) return _inState.rightTrigger / 255.0f;
    return 0;
}
