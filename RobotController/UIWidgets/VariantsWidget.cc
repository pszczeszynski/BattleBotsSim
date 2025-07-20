#include "VariantsWidget.h"
#include <imgui.h>
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "UIUtilities.h"
#include "ImageWidget.h"
#include "../Globals.h"

VariantsWidget::VariantsWidget()
{
}

void VariantsWidget::_DrawStartStopButton(const char* label, bool& enabledFlag, OdometryAlg algorithm)
{
    ImGui::PushID(algorithm);
    RobotOdometry& odometry = RobotController::GetInstance().odometry;
    bool isRunning = odometry.IsRunning(algorithm);
    
    // Set button color based on running state
    if (isRunning) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f)); // Green when running
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.6f, 0.0f, 1.0f));
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.0f, 0.0f, 1.0f)); // Red when stopped
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.0f, 0.0f, 1.0f));
    }

    if (ImGui::Button(isRunning ? "Running" : "Stopped"))
    {
        enabledFlag = !enabledFlag;
    }

    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::Text(label);

    // Sync state if needed
    if (isRunning != enabledFlag)
    {
        if (!enabledFlag)
        {
            odometry.Stop(algorithm);
        }
        else
        {
            odometry.Run(algorithm);
        }
    }

    ImGui::PopID();
}


void VariantsWidget::Draw()
{
    ImGui::Begin("Variant Selection");

    // **********************
#ifdef USE_OPENCV_TRACKER
    _DrawStartStopButton("OpenCV Tracker", ODO_OPENCV_ENABLED, OdometryAlg::OpenCV);
    
    // slider for min confidence
    ImGui::PushID(OdometryAlg::OpenCV);
    ImGui::SliderFloat("Min Confidence", &TRACKER_MIN_CONFIDENCE, 0.0f, 1.0f);
    ImGui::PopID();
#endif

    // **********************
    _DrawStartStopButton("Blob Detection", ODO_BLOB_ENABLED, OdometryAlg::Blob);

    // **********************
    _DrawStartStopButton("Heuristic", ODO_HEUR_ENABLED, OdometryAlg::Heuristic);

    // **********************
    _DrawStartStopButton("Position Neural Net", POSITION_NET_ENABLED, OdometryAlg::Neural);
    _DrawStartStopButton("Rotation Neural Net", ROTATION_NET_ENABLED, OdometryAlg::NeuralRot);
    
    ImGui::PushID(OdometryAlg::Neural);
    // slider for min confidence
    ImGui::SliderFloat("Min Confidence", &NN_MIN_CONFIDENCE, 0.0f, 1.0f);
    ImGui::PopID();


    // **********************
    // Algorithm override
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    CenterText("Algorithm override");

    // Helper for edit buttons
    auto DrawEditButton = [](const char* label, bool& state) {
        if (state) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.5f, 0.0f, 1.0f)); // Yellow when active
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.5f, 0.5f, 1.0f)); // Grey when inactive
        }

        if (ImGui::Button(label)) {
            state = !state;
        }
        ImGui::PopStyleColor();
    };

    DrawEditButton("Heuristic", EDITING_HEU);
    ImGui::SameLine();
    DrawEditButton("Blob", EDITING_BLOB);
#ifdef USE_OPENCV_TRACKER
    ImGui::SameLine();
    DrawEditButton("OpenCV Tracker", EDITING_OPENCV);
#endif

    // **********************
    // Neural Rotation Fusion
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    CenterText("Neural Rotation Fusion");

    // ANGLE_FUSE_CONF_THRESH
    ImGui::SliderFloat("Confidence Thresh", &ANGLE_FUSE_CONF_THRESH, 0.0f, 1.0f);
    // ANGLE_FUSE_SPEED
    ImGui::SliderFloat("Fuse Speed", &ANGLE_FUSE_SPEED, 0.0f, 10.0f);

    ImGui::End();
}
