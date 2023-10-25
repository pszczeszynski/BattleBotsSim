#include "RobotTelemetryWidget.h"
#include <vector>
#include "../RobotController.h"

RobotTelemetryWidget::RobotTelemetryWidget()
{
}

void RobotTelemetryWidget::Draw()
{
    ImGui::Begin("Robot Telemetry");
    std::vector<float> dummyLoss = {0.0, 0.5f, 0.0, 0.3};

    // display as graph
    ImGui::PlotLines("Packet Loss", dummyLoss.data(), dummyLoss.size(), 0, NULL, 0.0f, 1.0f, ImVec2(0, 80));

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

    CANData data = RobotController::GetInstance().GetCANData();

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