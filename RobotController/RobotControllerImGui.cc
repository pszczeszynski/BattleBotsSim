#include "RobotControllerImGui.h"

void MyApplication::RenderUI()
{
    ImGui::Begin("Test Window");
    bool test = ImGui::Button("Hello");
    static float value = 0.0f;
    if (test)
    {
        value++;
    }
    if (value < 20)
    {
        ImGui::DragFloat("Value", &value);
    }
    ImGui::End();
}
