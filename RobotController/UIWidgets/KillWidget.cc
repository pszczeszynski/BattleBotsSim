#include "KillWidget.h"
#include "../RobotConfig.h"

KillWidget KillWidget::instance;

KillWidget::KillWidget()
{
    _buttonPressed = false;
}

KillWidget& KillWidget::GetInstance()
{
    return instance;
}

/**
 * @brief Draws the kill mode config window
*/
void KillWidget::Draw()
{
    ImGui::Begin("Kill Config");
    ImGui::Button("Kill");
    _buttonPressed = ImGui::IsItemActive();
    ImGui::SliderInt("Angle Extrapolate (ms)", &KILL_KD_PERCENT, 0, 1000);
    ImGui::End();
}

/**
 * @brief Returns true if the kill button was pressed
*/
bool KillWidget::IsPressingButton() const
{
    return _buttonPressed;
}