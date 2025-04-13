#pragma once
#include "../RobotOdometry.h"

// A widget to select what algorithms of the robot are enabled at a time. Includes:
// 1. Rotation
    // use gyro(s)?
    // use CVRotation?
    // use computer vision rotation?
// 2. Positioning
    // use neural net?
    // use blob detection?
    // use background subtraction?
// 3. 


class VariantsWidget
{
public:
    VariantsWidget();


    void Draw();
private:
    void _DrawStartStopButton(const char* label, bool& enabledFlag, OdometryAlg algorithm);

};