#include "RobotTelemetryWidget.h"
#include <vector>
#include "../RobotController.h"

RobotTelemetryWidget::RobotTelemetryWidget()
{
}

void RobotTelemetryWidget::Draw()
{
    ImGui::Begin("Robot Telemetry");
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
    float graphHeight = 120.0f;
    float maxYValue = 0.05f; 
    int numLabels = 3; // Number of labels you want
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

    // display as graph
    ImGui::PlotLines("Packet Loss", delays.data(), delays.size(), 0, NULL, 0.0f, maxYValue, ImVec2(0, graphHeight));

    // add text for average delay
    float averageDelay = 0;
    for (float delay : delays)
    {
        averageDelay += delay;
    }
    averageDelay /= delays.size();
    ImGui::Text("Average Delay: %.2fms", averageDelay * 1000);


    // TABLE COLUMN HEADERS
    ImGui::Columns(4, "Telemetry");
    ImGui::Separator();
    ImGui::Text("Voltage");
    ImGui::NextColumn();
    ImGui::Text("Current");
    ImGui::NextColumn();
    ImGui::Text("ESC Temperature");
    ImGui::NextColumn();
    ImGui::Text("RPM");

    CANData& data = RobotController::GetInstance().GetCANData();

    for (int i = 0; i < 4; i++)
    {
        ImGui::Text(std::to_string((int) data.motorVoltage[i]).c_str());
        ImGui::NextColumn();
        ImGui::Text(std::to_string((int) data.motorCurrent[i]).c_str());
        ImGui::NextColumn();
        ImGui::Text(std::to_string((int) data.escFETTemp[i]).c_str());
        ImGui::NextColumn();
        ImGui::Text(std::to_string((int) data.motorRPM[i]).c_str());
        ImGui::NextColumn();
    }

    ImGui::End();
}