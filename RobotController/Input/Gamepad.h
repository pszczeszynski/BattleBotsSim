#pragma once

#ifdef WINDOWS
    #include <windows.h>
    #include <XInput.h> 
    #include "../libs/DS5W/ds5w.h"
#else 
    #include <linux/joystick.h>
    #include <fcntl.h>
    #include <unistd.h>
#endif

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

class Deck : public Gamepad
{
public:
    Deck();

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
    int read_event(int fd, struct js_event *event);

    bool buttons[10];
    float axes[8];
    const char device[15] = "/dev/input/js0";
    int js;
    struct js_event event;
    const float RANGE = 32767.0;

    const int aButton = 0;
    const int bButton = 1;
    const int xButton = 2;
    const int yButton = 3;

    const int leftBumper = 4;
    const int rightBumper = 5;

    const int leftStickClick = 9;
    const int rightStickClick = 10;

    const int leftX = 0;
    const int leftY = 1;
    const int rightX = 3;
    const int rightY = 4;

    const int leftTrigger = 2;
    const int rightTrigger = 5;

    const int dpadUpDown = 7;
    const int dpadLeftRight = 6;
};

#ifdef WINDOWS
class XBox : public Gamepad
{
public:
    XBox(int index = 0);

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
    int _controllerIndex;
    XINPUT_STATE _controllerState;
    const float RANGE = 32767.0f;
};



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
#endif