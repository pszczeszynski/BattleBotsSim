#include "RobotTelemetryWidget.h"
#include <vector>
#include "../RobotController.h"

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

void DrawRadioPacketLoss()
{
    // Center the text
    CenterText("Radio Packet Loss");

    const std::deque<RobotMessage> &messageHistory = 
        RobotController::GetInstance().GetRobotLink().GetMessageHistory();

    // extract just delays
    std::vector<float> delays;
    for (const RobotMessage &message : messageHistory)
    {
        delays.push_back(message.receiveDelay);
    }

    // Calculate position for the labels
    ImVec2 graphPos = ImGui::GetCursorScreenPos(); 
    float graphHeight = 150.0f;
    float maxYValue = 0.05f;
    int numLabels = 3;
    float step = maxYValue / (numLabels - 1);

    // Draw y-axis labels
    for (int i = 0; i < numLabels; i++)
    {
        char label[32];
        snprintf(label, sizeof(label), "%.0fms", i * step * 1000); // Convert to ms

        float yPosition;

        if (i == 0) // Bottommost label
        {
            yPosition = graphPos.y + graphHeight - ImGui::GetTextLineHeight();
        }
        else if (i == numLabels - 1) // Topmost label
        {
            yPosition = graphPos.y;
        }
        else // Middle labels
        {
            yPosition = graphPos.y + graphHeight - i * (graphHeight / (numLabels - 1)) - ImGui::GetTextLineHeight() * 0.5;
        }

        ImGui::SetCursorScreenPos(ImVec2(graphPos.x, yPosition));
        ImGui::TextUnformatted(label);
    }

    // Move cursor back to original position before drawing graph
    ImGui::SetCursorScreenPos(graphPos);
    float xOffset = 45.0f;
    ImGui::SameLine(xOffset);

    // Get the width available for the graph
    float graphWidth = ImGui::GetContentRegionAvail().x - xOffset;

    // Display as graph
    ImGui::PlotLines("", delays.data(), delays.size(), 0, NULL, 0.0f, maxYValue, ImVec2(graphWidth, graphHeight));


    // Add text for average delay
    float averageDelay = 0;
    for (float delay : delays)
    {
        averageDelay += delay;
    }
    averageDelay /= delays.size();

    // add small vertical space
    ImGui::Spacing();
    ImGui::Spacing();

    ImGui::Text("Average Delay: %.2fms", averageDelay * 1000);
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
            ImGui::Text("%d RPM", static_cast<int>(data.motorRPM[i]));
            ImGui::Separator();
        }
        ImGui::EndTable();
    }

    ImGui::End();
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
}