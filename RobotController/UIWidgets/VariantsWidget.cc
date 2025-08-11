#include "VariantsWidget.h"
#include <imgui.h>
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "UIUtilities.h"
#include "ImageWidget.h"
#include "../Globals.h"
#include "ColorScheme.h"

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
        ColorScheme::PushSuccessColors();
    } else {
        ColorScheme::PushErrorColors();
    }

    if (ImGui::Button(isRunning ? "Running" : "Stopped"))
    {
        enabledFlag = !enabledFlag;
    }

    ColorScheme::PopStatusColors();

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
            ColorScheme::PushButtonColors(ColorScheme::PRIMARY_ORANGE);
        } else {
            ColorScheme::PushButtonColors(ColorScheme::TEXT_DISABLED);
        }

        if (ImGui::Button(label)) {
            state = !state;
        }
        ColorScheme::PopButtonColors();
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
