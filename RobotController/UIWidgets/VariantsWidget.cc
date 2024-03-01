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

    RobotOdometry &odometry = RobotController::GetInstance().odometry;

    // **********************
    // Blob Detection
    CenterText("Blob");

    bool isRunning = odometry.IsRunning(OdometryAlg::Blob);

    if( ImGui::Button(ODO_BLOB_ENABLED ? "Stop Blob" : "Run Blob") )
    {
        ODO_BLOB_ENABLED = !ODO_BLOB_ENABLED;
    }

    if( isRunning != ODO_BLOB_ENABLED)
    {
        if (!ODO_BLOB_ENABLED)
        {
            odometry.Stop(OdometryAlg::Blob);
        }
        else
        {
            odometry.Run(OdometryAlg::Blob);
        }
    }
     

    ImGui::SameLine();
    ImGui::Text(isRunning ? "Running" : "Stopped");

    // **********************
    // Heuristic Detection
    CenterText("Heuristic");

    isRunning = odometry.IsRunning(OdometryAlg::Heuristic);

    if( ImGui::Button(ODO_HEUR_ENABLED ? "Stop Heuristic" : "Run Heuristic") )
    {
        ODO_HEUR_ENABLED = !ODO_HEUR_ENABLED;
    }

    if( isRunning != ODO_HEUR_ENABLED)
    {
        if (!ODO_HEUR_ENABLED)
        {
            odometry.Stop(OdometryAlg::Heuristic);
        }
        else
        {
            odometry.Run(OdometryAlg::Heuristic);
        }
    }

    ImGui::SameLine();
    ImGui::Text(isRunning ? "Running" : "Stopped");

    // **********************
    // IMU
    CenterText("IMU");

    isRunning = odometry.IsRunning(OdometryAlg::IMU);

    if (ImGui::Button(isRunning ? "Stop IMU" : "Run IMU"))
    {
        ODO_IMU_ENABLED = !ODO_IMU_ENABLED;
    }

    if( isRunning != ODO_IMU_ENABLED)
    {
        if (!ODO_IMU_ENABLED)
        {
            odometry.Stop(OdometryAlg::IMU);
        }
        else
        {
            odometry.Run(OdometryAlg::IMU);
        }
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