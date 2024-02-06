#include "RobotTelemetryWidget.h"
#include <vector>
#include "../RobotController.h"
#include "imgui_internal.h"
#include "GraphWidget.h"

RobotTelemetryWidget::RobotTelemetryWidget()
{
}

void CenterText(const char* text)
{
    // Center the text
    float windowWidth = ImGui::GetWindowWidth();
    float textWidth = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX((windowWidth - textWidth) / 2);

    ImGui::Text(text);
}


void DrawCANDataTable()
{
    // add Separator
    ImGui::Separator();
    CANData& data = RobotController::GetInstance().GetCANData();

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    // Center the text
    CenterText("Robot CAN Data");

    ImGui::PopStyleVar(2);

    const char* rowLabels[] = { "L Drive", "R Drive", "F Weapon", "B Weapon" };

    if (ImGui::BeginTable("table1", 5, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        // add column headers
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Voltage");
        ImGui::TableSetupColumn("Current");
        ImGui::TableSetupColumn("ESC Temperature");
        ImGui::TableSetupColumn("RPM");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 4; i++)
        {
            ImGui::TableNextRow();
            // Set the row label
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", rowLabels[i]);
            ImGui::Separator();

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d V", static_cast<int>(data.motorVoltage[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d Amps", static_cast<int>(data.motorCurrent[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%d °C", static_cast<int>(data.escFETTemp[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%d RPM", (int)(static_cast<int>(data.motorERPM[i]) * ((int)ERPM_FIELD_SCALAR) * ERPM_TO_RPM));
            ImGui::Separator();
        }
        ImGui::EndTable();
    }
}

// Make sure MAX_RPM and MAX_POWER are defined
#define MAX_POWER 1.0

void DrawRPMProgressBar(float targetPower, int currRPM)
{
    // Calculate the percentages
    float rpmPercentage = static_cast<float>(currRPM) / MAX_WEAPON_RPM;
    float powerPercentage = static_cast<float>(targetPower) / MAX_POWER;

    // Choose the progress bar color
    ImU32 barColor = currRPM < static_cast<int>(powerPercentage * MAX_WEAPON_RPM) * 0.9 ? IM_COL32(255, 0, 0, 255) : IM_COL32(0, 255, 0, 255);

    // Draw the progress bar
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
    ImGui::ProgressBar(rpmPercentage, ImVec2(-1, 0), ""); // '-1' for full width, '0' for default height
    ImGui::PopStyleColor();

    // Calculate the position for the target power line
    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 max = ImGui::GetItemRectMax();
    float powerLineX = min.x + (max.x - min.x) * powerPercentage;

    // Draw the target power line
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    window->DrawList->AddLine(ImVec2(powerLineX, min.y), ImVec2(powerLineX, max.y), IM_COL32(255, 0, 255, 255), 2.0f);
}


void RobotTelemetryWidget::Draw()
{
    ImGui::Begin("Robot Telemetry");

    DrawCANDataTable();

    ImGui::Spacing();
    ImGui::Spacing();

    #define FRONT_WEAPON_INDEX 2
    #define BACK_WEAPON_INDEX 3


    // center
    CenterText("Weapons");

    ImGui::Text("Front Weapon RPM: %d", (int)(Weapons::GetInstance().GetFrontWeaponRPM()));
    // Draw the RPM progress bars
    DrawRPMProgressBar(Weapons::GetInstance().GetFrontWeaponTargetPower(),
                       Weapons::GetInstance().GetFrontWeaponRPM());

    ImGui::Spacing();

    ImGui::Text("Back Weapon RPM: %d", (int)(Weapons::GetInstance().GetBackWeaponRPM()));
    DrawRPMProgressBar(Weapons::GetInstance().GetBackWeaponTargetPower(),
                       Weapons::GetInstance().GetBackWeaponRPM());

    ImGui::End();

}