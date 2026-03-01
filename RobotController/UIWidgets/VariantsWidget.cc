#include "VariantsWidget.h"

#include <imgui.h>

#include "../RobotConfig.h"
#include "../RobotController.h"
#include "ColorScheme.h"
#include "UIUtilities.h"

VariantsWidget::VariantsWidget() {}

void VariantsWidget::_DrawStartStopButton(const char* label, bool& enabledFlag,
                                          OdometryAlg algorithm) {
  ImGui::PushID(algorithm);
  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  bool isRunning = odometry.IsRunning(algorithm);

  // Set button color based on running state
  if (isRunning) {
    ColorScheme::PushSuccessColors();
  } else {
    ColorScheme::PushErrorColors();
  }

  if (ImGui::Button(isRunning ? "Running" : "Stopped")) {
    enabledFlag = !enabledFlag;
  }

  ColorScheme::PopStatusColors();

  ImGui::SameLine();
  ImGui::Text(label);

  // Sync state if needed
  if (isRunning != enabledFlag) {
    if (!enabledFlag) {
      odometry.Stop(algorithm);
    } else {
      odometry.Run(algorithm);
    }
  }

  ImGui::PopID();
}

void VariantsWidget::Draw() {
  ImGui::Begin("Variant Selection");

  // **********************
  _DrawStartStopButton("OpenCV Tracker", ODO_OPENCV_ENABLED,
                       OdometryAlg::OpenCV);

  // slider for min confidence
  ImGui::PushID(OdometryAlg::OpenCV);
  ImGui::SliderFloat("Min Confidence", &TRACKER_MIN_CONFIDENCE, 0.0f, 1.0f);
  ImGui::PopID();

  // **********************
  _DrawStartStopButton("Blob Detection", ODO_BLOB_ENABLED, OdometryAlg::Blob);
  _DrawStartStopButton("Heuristic", ODO_HEUR_ENABLED, OdometryAlg::Heuristic);
  _DrawStartStopButton("LK Flow", ODO_LKFLOW_ENABLED, OdometryAlg::LKFlow);
  _DrawStartStopButton("Position Neural Net", POSITION_NET_ENABLED,
                       OdometryAlg::Neural);
  _DrawStartStopButton("Rotation Neural Net", ROTATION_NET_ENABLED,
                       OdometryAlg::NeuralRot);

  ImGui::PushID(OdometryAlg::Neural);
  ImGui::SliderFloat("Min Confidence", &NN_MIN_CONFIDENCE, 0.0f, 1.0f);
  ImGui::PopID();

  // **********************
  // Neural Rotation Fusion
  ImGui::Spacing();
  ImGui::Spacing();
  ImGui::Spacing();
  CenterText("Neural Rotation Fusion");

  ImGui::SliderFloat("Confidence Thresh", &ANGLE_FUSE_CONF_THRESH, 0.0f, 1.0f);
  ImGui::SliderFloat("Fuse Speed", &ANGLE_FUSE_SPEED, 0.0f, 10.0f);

  ImGui::End();
}
