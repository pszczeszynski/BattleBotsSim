#pragma once
// #include <windows.h>
// #include <XInput.h>
//#include "../libs/DS5W/ds5w.h"
#include <iostream>

class Gamepad
{
public:
    Gamepad();

    virtual void Update() = 0;

    // Button checkers
    virtual bool GetButtonA() = 0;
    virtual bool GetButtonB() = 0;
    virtual bool GetButtonX() = 0;
    virtual bool GetButtonY() = 0;
    virtual bool GetLeftBumper() = 0;
    virtual bool GetRightBumper() = 0;
    virtual bool GetDpadLeft() = 0;
    virtual bool GetDpadRight() = 0;
    virtual bool GetDpadUp() = 0;
    virtual bool GetDpadDown() = 0;

    // Stick values, range from -1.0 to 1.0
    virtual float GetLeftStickX() = 0;
    virtual float GetLeftStickY() = 0;
    virtual float GetRightStickX() = 0;
    virtual float GetRightStickY() = 0;

    virtual float GetLeftTrigger() = 0;
    virtual float GetRightTrigger() = 0;
};


// class XBox : public Gamepad
// {
// public:
//     XBox(int index = 0);

//     virtual void Update() override;

//     // Button checkers
//     virtual bool GetButtonA() override;
//     virtual bool GetButtonB() override;
//     virtual bool GetButtonX() override;
//     virtual bool GetButtonY() override;
//     virtual bool GetLeftBumper() override;
//     virtual bool GetRightBumper() override;
//     virtual bool GetDpadLeft() override;
//     virtual bool GetDpadRight() override;
//     virtual bool GetDpadUp() override;
//     virtual bool GetDpadDown() override;

//     // Stick values, range from -1.0 to 1.0
//     virtual float GetLeftStickX() override;
//     virtual float GetLeftStickY() override;
//     virtual float GetRightStickX() override;
//     virtual float GetRightStickY() override;

//     virtual float GetLeftTrigger() override;
//     virtual float GetRightTrigger() override;

// private:
//     int _controllerIndex;
//     XINPUT_STATE _controllerState;
//     const float RANGE = 32767.0f;
// };



class DualSense : public Gamepad
{
public:
    DualSense();
    ~DualSense();

    bool InitController(int index);
    virtual void Update() override;

    // Button checkers
    virtual bool GetButtonA() override;
    virtual bool GetButtonB() override;
    virtual bool GetButtonX() override;
    virtual bool GetButtonY() override;
    virtual bool GetLeftBumper() override;
    virtual bool GetRightBumper() override;
    virtual bool GetDpadLeft() override;
    virtual bool GetDpadRight() override;
    virtual bool GetDpadUp() override;
    virtual bool GetDpadDown() override;

    // Stick values, range from -1.0 to 1.0
    virtual float GetLeftStickX() override;
    virtual float GetLeftStickY() override;
    virtual float GetRightStickX() override;
    virtual float GetRightStickY() override;

    virtual float GetLeftTrigger() override;
    virtual float GetRightTrigger() override;
private:
    //Array of Controller Infos
    //DS5W::DeviceEnumInfo _infos[16];

    // Number of controllers found
    unsigned int _controllersCount;

    // Successful controller initialization
    bool _initialized;

    // Context for controller
    //DS5W::DeviceContext _con;

    // Inputstate
    //DS5W::DS5InputState _inState;
    const float RANGE = 255.0f;
};
