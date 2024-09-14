#include "ManualControlWidget.h"
#include "imgui.h"
#include "../RobotConfig.h"
#include "../RobotLink.h"
#include "../RobotController.h"

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

    // Toggle button to enable/disable keyboard control
    if (WASD_ENABLED)
    {
        // Change button color to green when keyboard is enabled
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f));        // Green
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.2f, 1.0f)); // Slightly lighter green
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));  // Darker green
    }
    else
    {
        // Change button color to red when keyboard is disabled
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));        // Red
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.2f, 0.2f, 1.0f)); // Slightly lighter red
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));  // Darker red
    }

    // Button logic for toggling keyboard
    if (ImGui::Button(WASD_ENABLED ? "Disable WASD" : "Enable WASD", ImVec2(ImGui::GetWindowWidth(), 20)))
    {
        // Toggle WASD enabled state
        WASD_ENABLED = !WASD_ENABLED;
    }

    // Pop the color styles to revert to default styling after the button
    ImGui::PopStyleColor(3);

    // draw the buttons
    if (ImGui::Button("STOP", ImVec2(ImGui::GetWindowWidth(), 20)))
    {
        MASTER_MOVE_SCALE_PERCENT = 0;
        MASTER_TURN_SCALE_PERCENT = 0;
        MAX_FRONT_WEAPON_SPEED = 0;
        MAX_BACK_WEAPON_SPEED = 0;
    }

    // spacers
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

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


    // text for weapons
    ImGui::Text("Weapons");
    // sliders for the weapon speeds
    ImGui::SliderFloat("Bar Speed", &MAX_FRONT_WEAPON_SPEED, 0, 300.0);
    ImGui::SliderFloat("Disk Speed", &MAX_BACK_WEAPON_SPEED, 0, 300.0);


#ifdef SIMULATION
    // line for simulation stuff
    ImGui::Separator();
    ImGui::Text("Simulation");
    // button to reset
    if (ImGui::Button("Reset Simulation"))
    {
        IRobotLink& simLink = RobotController::GetInstance().GetRobotLink();
        RobotLinkSim& sim = dynamic_cast<RobotLinkSim&>(simLink);
        sim.ResetSimulation();
    }
#endif


    ImGui::End();
}