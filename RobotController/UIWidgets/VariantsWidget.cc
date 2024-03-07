#include "VariantsWidget.h"
#include <imgui.h>
#include "../RobotConfig.h"
#include "../RobotOdometry.h"
#include "../RobotController.h"
#include "UIUtilities.h"
#include "ImageWidget.h"
#include "../Globals.h"

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

    if (ImGui::Button(ODO_BLOB_ENABLED ? "Stop Blob" : "Run Blob"))
    {
        ODO_BLOB_ENABLED = !ODO_BLOB_ENABLED;
    }

    if (isRunning != ODO_BLOB_ENABLED)
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

    if (ImGui::Button(ODO_HEUR_ENABLED ? "Stop Heuristic" : "Run Heuristic"))
    {
        ODO_HEUR_ENABLED = !ODO_HEUR_ENABLED;
    }

    if (isRunning != ODO_HEUR_ENABLED)
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

    if (isRunning != ODO_IMU_ENABLED)
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

    isRunning = odometry.IsRunning(OdometryAlg::Neural);

    if (ImGui::Button("Rotation Neural Net"))
    {
        // toggle the rotation network on or off
        ROTATION_NET_ENABLED = !ROTATION_NET_ENABLED;
    }

    if (isRunning != ROTATION_NET_ENABLED)
    {
        if (!ROTATION_NET_ENABLED)
        {
            odometry.Stop(OdometryAlg::Neural);
        }
        else
        {
            odometry.Run(OdometryAlg::Neural);
        }
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


    // center
    ImGui::Spacing();
    CenterText("Algorithm override");

    // 2 buttons on the same line, each can be turned on. Off color is grey. On color is yellow

    // set color to grey if not editing, otherwise set to yellow
    if (EDITING_HEU)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
    }

    if (ImGui::Button("Heuristic"))
    {
        EDITING_HEU = !EDITING_HEU;
    }

    ImGui::PopStyleColor();




    ImGui::SameLine();


    if (EDITING_BLOB)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
    }

    if (ImGui::Button("Blob"))
    {
        EDITING_BLOB = !EDITING_BLOB;
    }

    ImGui::PopStyleColor();


    ImGui::End();
}