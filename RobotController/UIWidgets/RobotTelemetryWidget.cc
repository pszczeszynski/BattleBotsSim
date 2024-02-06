#include "RobotTelemetryWidget.h"
#include <vector>
#include "../RobotController.h"
#include "imgui_internal.h"

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

/**
 * \brief
 * Draws a graph with the given data
 * 
*/
void DrawGraph(const char *title, const std::vector<float> &data, float minYValue, float maxYValue,
               const char *unit, const ImVec2 &graphSize = ImVec2(0, 150.0f),
               const ImVec4 &graphColor = ImVec4(0.2f, 0.6f, 0.86f, 1.0f)) // Default graph color
{
    // Ensure there are always 3 labels
    const int numLabels = 3;

    // Center the text
    CenterText(title);

    ImVec2 graphPos = ImGui::GetCursorScreenPos();
    float graphHeight = graphSize.y;
    float range = maxYValue - minYValue;  // Total range of the data
    float step = range / (numLabels - 1); // Step value for each label

    // Draw y-axis labels
    for (int i = 0; i < numLabels; i++)
    {
        char label[32];
        // Adjust label to include the min value in the calculation
        snprintf(label, sizeof(label), "%.0f%s", minYValue + i * step, unit);

        float yPosition;
        if (i == 0)
        { // Bottommost label
            yPosition = graphPos.y + graphHeight - ImGui::GetTextLineHeight();
        }
        else if (i == numLabels - 1)
        { // Topmost label
            yPosition = graphPos.y;
        }
        else
        { // Middle label
            yPosition = graphPos.y + graphHeight - i * (graphHeight / (numLabels - 1)) - ImGui::GetTextLineHeight() * 0.5;
        }

        ImGui::SetCursorScreenPos(ImVec2(graphPos.x, yPosition));
        ImGui::TextUnformatted(label);
    }

    // Move cursor back to original position before drawing graph
    ImGui::SetCursorScreenPos(graphPos);
    float xOffset = 45.0f;
    ImGui::SameLine(xOffset);

    float graphWidth = ImGui::GetContentRegionAvail().x - xOffset;

    // Set the color for the graph line
    ImGui::PushStyleColor(ImGuiCol_PlotLines, graphColor);

    // Display the graph, adjusting for the minimum value
    ImGui::PlotLines("", data.data(), data.size(), 0, nullptr, minYValue, maxYValue, ImVec2(graphWidth, graphHeight));

    // Pop the style color to revert to the previous setting
    ImGui::PopStyleColor();
}

void DrawRadioPacketLoss()
{
    // Get the message history
    std::deque<RobotMessage> messageHistory = RobotController::GetInstance().GetRobotLink().GetMessageHistory();

    std::vector<float> lossData;
    for (const RobotMessage &message : messageHistory)
    {
        lossData.push_back(message.receiveDelay);
    }
    DrawGraph("Radio Packet Loss", lossData, 0.0f, 50.0f, "ms");


    // add a second line for the packet id
    std::vector<float> relativePacketIDs;
    for (int i = 1; i < messageHistory.size(); i++)
    {
        relativePacketIDs.push_back(messageHistory[i].packetID - messageHistory[i-1].packetID);
    }

    DrawGraph("Relative Packet ID", relativePacketIDs, -1.0f, 1.0f, "");

    // // add third line for relative times
    // std::vector<float> relativeTimes;
    // for (int i = 1; i < messageHistory.size(); i++)
    // {
    //     relativeTimes.push_back(messageHistory[i].packetTimeMS - messageHistory[i-1].packetTimeMS);
    // }

    // // add text 
    // ImGui::Spacing();
    // // say "relative packet id"
    // ImGui::Text("Relative Packet ID");
    // // Change the color to red
    // ImGui::GetStyle().Colors[ImGuiCol_PlotLines] = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);

    // ImGui::PlotLines("", relativePacketIDs.data(), relativePacketIDs.size(), 0, NULL, -4.0, 4.0, ImVec2(graphWidth, graphHeight));

    // ImGui::Spacing();
    // // say "relative time"
    // ImGui::Text("Relative Time (MS) from transmitter teensy");
    // // Change the color to red
    // ImGui::GetStyle().Colors[ImGuiCol_PlotLines] = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);

    // ImGui::PlotLines("", relativeTimes.data(), relativeTimes.size(), 0, NULL, -25, 25.0, ImVec2(graphWidth, graphHeight));

    // //


    // // Add text for average delay
    // float averageDelay = 0;
    // for (float delay : delays)
    // for (float delay : delays)
    // {
    //     averageDelay += delay;
    // }
    // averageDelay /= delays.size();

    // // add small vertical space
    // ImGui::Spacing();
    // ImGui::Spacing();

    // ImGui::Text("Average Delay: %.2fms", averageDelay * 1000);
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

    DrawRadioPacketLoss();
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();


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