#include "VariantsWidget.h"
#include <imgui.h>
#include "../RobotConfig.h"
#include "UIUtilities.h"

VariantsWidget::VariantsWidget()
{

}

void VariantsWidget::Draw()
{
    ImGui::Begin("Variant Selection");
    CenterText("Robot Odometry");

    if (ImGui::Button("Rotation Neural Net"))
    {
        // toggle the rotation network on or off
        ROTATION_NET_ENABLED = !ROTATION_NET_ENABLED;
    }

    ImGui::SameLine();
    ImGui::Text(ROTATION_NET_ENABLED ? "ENABLED" : "DISABLED");


    ImGui::Spacing();

    // gyro
    if (ImGui::Button("Gyro"))
    {
        GYRO_ENABLED = !GYRO_ENABLED;
    }

    ImGui::SameLine();
    ImGui::Text(GYRO_ENABLED ? "ENABLED" : "DISABLED");

    ImGui::End();
}