#include "KillWidget.h"
#include "../RobotConfig.h"

KillWidget* KillWidget::instance = nullptr;

KillWidget::KillWidget()
{
    _buttonPressed = false;
    instance = this;
}

KillWidget& KillWidget::GetInstance()
{
    return *KillWidget::instance;
}

/**
 * @brief Draws the kill mode config window
*/
void KillWidget::Draw()
{
    ImGui::Begin("Kill Config");
    ImGui::Button("Kill");
    _buttonPressed = ImGui::IsItemActive();
    ImGui::SliderInt("Angle Extrapolate (ms)", &KILL_ANGLE_EXTRAPOLATE_MS, 0, 1000);
    ImGui::End();
}

/**
 * @brief Returns true if the kill button was pressed
*/
bool KillWidget::IsPressingButton() const
{
    return _buttonPressed;
}