#include "ManualControlWidget.h"
#include "imgui.h"
#include "../RobotConfig.h"

ManualControlWidget::ManualControlWidget()
{
}

void ManualControlWidget::Draw()
{
    // draw the manual control data
    ImGui::Begin("Manual Control");

    // draw the sliders
    ImGui::SliderInt("Move Scale", &MASTER_MOVE_SCALE_PERCENT, 0, 100);
    ImGui::SliderInt("Turn Scale", &MASTER_TURN_SCALE_PERCENT, 0, 100);

    // draw the buttons
    if (ImGui::Button("STOP"))
    {
        MASTER_MOVE_SCALE_PERCENT = 0;
        MASTER_TURN_SCALE_PERCENT = 0;
    }

    if (ImGui::Button("Invert Turn"))
    {
        INVERT_TURN = !INVERT_TURN;
    }
    ImGui::SameLine();
    ImGui::Text(INVERT_TURN ? "INVERTED" : "NORMAl");

    if (ImGui::Button("Invert Movement"))
    {
        INVERT_MOVEMENT = !INVERT_MOVEMENT;
    }
    ImGui::SameLine();
    ImGui::Text(INVERT_MOVEMENT ? "INVERTED" : "NORMAl");

    ImGui::End();
}