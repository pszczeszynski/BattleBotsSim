#include "VariantsWidget.h"
#include <imgui.h>
#include "../RobotConfig.h"
#include "../RobotOdometry.h"
#include "../RobotController.h"
#include "UIUtilities.h"

VariantsWidget::VariantsWidget()
{

}

void VariantsWidget::Draw()
{
    ImGui::Begin("Variant Selection");

    RobotOdometry& odometry = RobotController::GetInstance().odometry;

    // **********************
    // Blob Detection
    CenterText("Blob");

    bool isRunning = odometry.IsRunning(OdometryAlg::Blob );

    if (ImGui::Button(  isRunning ? "Stop Blob" : "Run Blob") )
    {
        if( isRunning) { odometry.Stop(OdometryAlg::Blob); }
        else { odometry.Run(OdometryAlg::Blob); }
    }

    ImGui::SameLine();
    ImGui::Text(isRunning ? "Running" : "Stopped");

    // **********************
    // Heuristic Detection
    CenterText("Heuristic");

    isRunning = odometry.IsRunning(OdometryAlg::Heuristic );

    if (ImGui::Button(  isRunning ? "Stop Heuristic" : "Run Heuristic") )
    {
        if( isRunning) { odometry.Stop(OdometryAlg::Heuristic); }
        else { odometry.Run(OdometryAlg::Heuristic); }
    }

    ImGui::SameLine();
    ImGui::Text(isRunning ? "Running" : "Stopped");


    // ************** OLD CODE *******************

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