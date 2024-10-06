#include "RobotTelemetryWidget.h"
#include <vector>
#include "../RobotController.h"
#include "imgui_internal.h"
#include "GraphWidget.h"
#include "UIUtilities.h"

RobotTelemetryWidget::RobotTelemetryWidget()
{
}

void DrawCANDataTable()
{
    // add Separator
    ImGui::Separator();
    CANData data = RobotController::GetInstance().GetRobotLink().GetLastCANMessage().canData;

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    // Center the text
    CenterText("Robot CAN Data");

    ImGui::PopStyleVar(2);

    const char *rowLabels[] = {"L Drive", "R Drive", "F Weapon", "B Weapon"};

    if (ImGui::BeginTable("table1", 6, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        // add column headers
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Voltage");
        ImGui::TableSetupColumn("Current");
        ImGui::TableSetupColumn("ESC Temperature");
        ImGui::TableSetupColumn("Motor Temperature");
        ImGui::TableSetupColumn("RPM");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 4; i++)
        {
            ImGui::TableNextRow();
            // Check conditions for changing row color
            // bool setRed = data.motorTemp[i] == 0 || data.escFETTemp[i] > 80 || data.motorTemp[i] > 90;

            if (data.motorTemp[i] == 0)
            {
                // don't set color
            }
            else if (data.escFETTemp[i] > 80 || data.motorTemp[i] > 90 || data.motorVoltage[i] <= 57)
            {
                // Set row color to red
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(255, 0, 0, 255));
            }
            else if (data.escFETTemp[i] > 60 || data.motorTemp[i] > 60)
            {
                // Set row color to yellow
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(255, 255, 0, 255));
            }
            else
            {
                // Set row color to green
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(0, 150, 0, 255));
            }

            // Your existing code to set row data
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
            ImGui::Text("%d °C", static_cast<int>(data.motorTemp[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(5);
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

void DrawRadioData()
{
    // get latest message
    RadioData data = RobotController::GetInstance().GetRobotLink().GetLastRadioMessage().radioData;

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    // Center the text
    CenterText("Radio Data");

    ImGui::PopStyleVar(2);

    const char* rowLabels[] = { "Average Delay (ms)", "Max Delay (ms)", "Invalid Packets per 10", "Movement", "Turn", "Front Weapon", "Back Weapon" };

    // 2 columns (1 for label, 1 for value)
    
    if (ImGui::BeginTable("table2", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        for (int i = 0; i < 7; i++)
        {
            ImGui::TableNextRow();
            // Set the row label
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", rowLabels[i]);
            ImGui::Separator();

            ImGui::TableSetColumnIndex(1);
            switch (i)
            {
            case 0:
                ImGui::Text("%d", (int) data.averageDelayMS);
                break;
            case 1:
                ImGui::Text("%d", (int) data.maxDelayMS);
                break;
            case 2:
                ImGui::Text("%d", (int) data.invalidPackets);
                break;
            case 3:
                ImGui::Text("%f", data.movement);
                break;
            case 4:
                ImGui::Text("%f", data.turn);
                break;
            case 5:
                ImGui::Text("%f", data.frontWeaponPower);
                break;
            case 6:
                ImGui::Text("%f", data.backWeaponPower);
                break;
            }
            ImGui::Separator();
        }
        ImGui::EndTable();
    }
}

void DrawBoardTelemetryData()
{
    // get latest message
    BoardTelemetryData data = RobotController::GetInstance().GetRobotLink().GetLastBoardTelemetryMessage().boardTelemetryData;

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    ImGui::Spacing();
    CenterText("Power Telemetry");

    ImGui::PopStyleVar(2);

    const char* powerRowLabels[] = { "Tx/Rx", "VBat", "5V volt", "5V amp", "3v3 Volt", "Temp"};
    if (ImGui::BeginTable("table3", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%s", powerRowLabels[0]);
        ImGui::Separator();
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%s", "Tx");
        ImGui::Separator();
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%s", "Rx");
        ImGui::Separator();

        for (int i = 1; i < 6; i++)
        {
            ImGui::TableNextRow();
            // Set the row label
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", powerRowLabels[i]);
            ImGui::Separator();

            ImGui::TableSetColumnIndex(2);
            switch (i)
            {
            case 1:
                ImGui::Text("%f V", data.voltage_batt);
                break;
            case 2:
                ImGui::Text("%f V", data.voltage_5v);
                break;
            case 3:
                ImGui::Text("%f A", data.current_5v);
                break;
            case 4:
                ImGui::Text("%f V", data.voltage_3v3);
                break;
            case 5:
                ImGui::Text("%f C", data.temperature);
                break;
            }
            ImGui::Separator();
        }
        ImGui::EndTable();
    }
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


    ImGui::Spacing();
    ImGui::Spacing();

    DrawRadioData();
    DrawBoardTelemetryData();

    ImGui::End();
}