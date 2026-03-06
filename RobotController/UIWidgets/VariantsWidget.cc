// VariantsWidget.cpp
//
// 2-column table (State | Variant). All settings live BELOW the table,
// clearly labeled, with minimal nesting.
//
// Notes:
// - Clicking RUNNING/STOPPED immediately starts/stops the algorithm and flips
//   the corresponding enabled flag.
// - Settings are grouped by algorithm and shown below in a simple, readable
//   layout (no right-side nesting).

#include "VariantsWidget.h"

#include <imgui.h>

#include "../RobotConfig.h"
#include "../RobotController.h"
#include "ColorScheme.h"
#include "UIUtilities.h"

namespace {

bool StatusPillButton(bool isRunning) {
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 5));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 999.0f);

  if (isRunning) {
    ColorScheme::PushSuccessColors();
  } else {
    ColorScheme::PushErrorColors();
  }

  const char* txt = isRunning ? "RUNNING" : "STOPPED";
  const bool pressed = ImGui::Button(txt);

  ColorScheme::PopStatusColors();
  ImGui::PopStyleVar(2);
  return pressed;
}

void DrawAlgRow2Col(OdometryAlg alg, const char* name, bool& enabledFlag) {
  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  bool isRunning = odometry.IsRunning(alg);

  // Sync odometry state with config (e.g. after Load Config)
  if (isRunning != enabledFlag) {
    if (enabledFlag) odometry.Run(alg);
    else odometry.Stop(alg);
    isRunning = enabledFlag;
  }

  ImGui::PushID((int)alg);
  ImGui::TableNextRow();

  // Col 0: state
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  if (StatusPillButton(isRunning)) {
    enabledFlag = !enabledFlag;
    if (enabledFlag) odometry.Run(alg);
    else             odometry.Stop(alg);
    isRunning = enabledFlag;
  }

  // Col 1: variant name
  ImGui::TableSetColumnIndex(1);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(name);

  ImGui::PopID();
}

void SettingsHeader(const char* title) {
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::TextUnformatted(title);
  ImGui::Spacing();
}

void LabeledSlider(const char* label, const char* id, float* v, float lo, float hi,
  const char* fmt = "%.3f") {
ImGui::AlignTextToFramePadding();
ImGui::TextDisabled("%s", label);
ImGui::SameLine();
ImGui::SetNextItemWidth(-FLT_MIN);
ImGui::SliderFloat(id, v, lo, hi, fmt);
}
}  // namespace

VariantsWidget::VariantsWidget() {}

void VariantsWidget::Draw() {
  ImGui::Begin("Variant Selection");

  // =========================
  // Table (2 columns only)
  // =========================
  ImGui::TextUnformatted("Odometry Variants");
  ImGui::Separator();
  ImGui::Spacing();

  ImGuiTableFlags flags =
      ImGuiTableFlags_RowBg |
      ImGuiTableFlags_BordersInnerV |
      ImGuiTableFlags_BordersOuter |
      ImGuiTableFlags_SizingStretchProp;

  if (ImGui::BeginTable("##variants", 2, flags)) {
    ImGui::TableSetupColumn("State",   ImGuiTableColumnFlags_WidthFixed, 110.0f);
    ImGui::TableSetupColumn("Variant", ImGuiTableColumnFlags_WidthStretch, 1.0f);

    DrawAlgRow2Col(OdometryAlg::OpenCV,   "OpenCV Tracker",       ODO_OPENCV_ENABLED);
    DrawAlgRow2Col(OdometryAlg::Blob,     "Blob Detection",       ODO_BLOB_ENABLED);
    DrawAlgRow2Col(OdometryAlg::Heuristic,"Heuristic",            ODO_HEUR_ENABLED);
    DrawAlgRow2Col(OdometryAlg::LKFlow,   "LK Flow",              ODO_LKFLOW_ENABLED);
    DrawAlgRow2Col(OdometryAlg::Neural,   "Position Neural Net",  POSITION_NET_ENABLED);
    DrawAlgRow2Col(OdometryAlg::NeuralRot,"Rotation Neural Net",  ROTATION_NET_ENABLED);

    ImGui::EndTable();
  }

  // =========================
  // Settings (below table)
  // =========================
  SettingsHeader("Settings");

  // OpenCV tracker settings
  {
    SettingsHeader("OpenCV Tracker");
    ImGui::PushID("opencv_settings");
    LabeledSlider("Min confidence", "##min_confidence", &TRACKER_MIN_CONFIDENCE, 0.0f, 1.0f, "%.3f");
    ImGui::PopID();
  }

  // Position NN settings
  {
    SettingsHeader("Position Neural Net");
    ImGui::PushID("pos_nn_settings");
    LabeledSlider("Min confidence", "##min_confidence", &NN_MIN_CONFIDENCE, 0.0f, 1.0f, "%.3f");
    ImGui::PopID();
  }

  // Fusion settings
  {
    SettingsHeader("Neural Rotation Fusion");
    ImGui::PushID("fusion_settings");
    LabeledSlider("Confidence thresh", "##confidence_thresh", &ANGLE_FUSE_CONF_THRESH, 0.0f, 1.0f, "%.3f");
    LabeledSlider("Fuse speed", "##fuse_speed", &ANGLE_FUSE_SPEED, 0.0f, 10.0f, "%.3f");
    ImGui::PopID();
  }

  ImGui::End();
}