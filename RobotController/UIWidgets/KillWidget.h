#pragma once
#include "imgui.h"

class KillWidget
{
public:
    static KillWidget& GetInstance();
    KillWidget();
    void Draw();
    bool IsPressingButton() const;
    

private:
    static KillWidget instance;
    bool _buttonPressed;
};