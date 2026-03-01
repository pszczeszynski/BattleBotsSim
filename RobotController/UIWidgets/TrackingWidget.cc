#include "TrackingWidget.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#include "../Clock.h"
#include "../Input/InputState.h"
#include "../Odometry/OdometryData.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "../SafeDrawing.h"
#include "CameraWidget.h"
#include "ColorScheme.h"
#include "TrackingFieldMaskEditor.h"

namespace {

void DrawX(cv::Mat& mat, cv::Point2f pos, cv::Scalar color, int size) {
  cv::line(mat, pos + cv::Point2f(-size, -size), pos + cv::Point2f(size, size),
           color, 2);
  cv::line(mat, pos + cv::Point2f(-size, size), pos + cv::Point2f(size, -size),
           color, 2);
}

}  // namespace

TrackingWidget* TrackingWidget::_instance = nullptr;
cv::Point2f TrackingWidget::robotMouseClickPoint = cv::Point2f(0, 0);
cv::Point2f TrackingWidget::opponentMouseClickPoint = cv::Point2f(0, 0);
double TrackingWidget::robotMouseClickAngle = 0;
double TrackingWidget::opponentMouseClickAngle = 0;

static constexpr const char* kFieldMaskPath = "./backgrounds/fieldMask.jpg";

TrackingWidget::TrackingWidget()
    : _fieldMask{},
      _maskEditor(std::filesystem::path(kFieldMaskPath)),
      ImageWidget("Tracking", _trackingMat, false) {
  _instance = this;

  _maskEditor.LoadOrInit(_fieldMask, cv::Size(WIDTH, HEIGHT));

  // Initialize default colors for each variant using our color scheme
  for (size_t i = 0; i < kDebugVariantCount; ++i) {
    DebugVariant v = static_cast<DebugVariant>(i);
    variantColors[i] = ColorScheme::GetVariantColor(v);
    variantOffsets[i] = cv::Point(0, 0);
  }

  RestoreGUISettings(VISION_TRACKING_GUI);
}

void TrackingWidget::_GrabFrame() {
  // Fetch camera each call to avoid stale pointer if subsystem is
  // reinitialized.
  ICameraReceiver* camera = ICameraReceiver::GetInstance();
  if (camera != nullptr && camera->NewFrameReady(_lastFrameId)) {
    _lastFrameId = camera->GetFrame(
        variantImages[static_cast<size_t>(DebugVariant::Camera)], _lastFrameId);
  }
}

TrackingWidget* TrackingWidget::GetInstance() { return _instance; }

void TrackingWidget::_MaskOutRegions() {
  _maskEditor.Update(
      CameraWidget::DrawMask, IsMouseOver(), InputState::GetInstance(),
      [this] { return GetMousePos(); }, _fieldMask);
  _maskEditor.SaveIfDirty(_fieldMask);
  _maskEditor.RenderOverlay(CameraWidget::DrawMask, _trackingMat, _fieldMask);
}

void TrackingWidget::ClearMask() {
  _maskEditor.Clear(_fieldMask, cv::Size(WIDTH, HEIGHT));
  _maskEditor.SaveIfDirty(_fieldMask);
}

cv::Mat& TrackingWidget::GetMask() { return _fieldMask; }

void TrackingWidget::_DrawAlgorithmData() {
  if (_trackingMat.empty()) {
    return;
  }
  // draw arrow on opponent robot showing gamepad stick angle
  OdometryData opponentData =
      RobotController::GetInstance().odometry.Opponent();
  cv::Point2f opponentPos = opponentData.GetPositionOrZero();

  // Read gamepad right stick values
  float stickX = RobotController::GetInstance().gamepad.GetRightStickX();
  float stickY = RobotController::GetInstance().gamepad.GetRightStickY();

  // Calculate angle from stick position (only if stick is being moved)
  float stickMagnitude = sqrt(stickX * stickX + stickY * stickY);
  if (stickMagnitude > 0.1) {  // deadzone threshold
    float stickAngle = atan2(-stickY, stickX);
    cv::Point2f stickArrowEnd =
        opponentPos + cv::Point2f{cos(stickAngle), sin(stickAngle)} * 50;
    safe_arrow(_trackingMat, opponentPos, stickArrowEnd,
               cv::Scalar(0, 255, 255), 3);
  }

  // BGR scalars from variant colors
  auto toScalar = [this](DebugVariant v) {
    const ImVec4& c = variantColors[static_cast<size_t>(v)];
    return cv::Scalar(c.z * 255, c.y * 255, c.x * 255);
  };
  cv::Scalar blobColor = toScalar(DebugVariant::Blob);
  cv::Scalar neuralColor = toScalar(DebugVariant::Neural);
  cv::Scalar heuristicColor = toScalar(DebugVariant::Heuristic);
  cv::Scalar fusionColor = toScalar(DebugVariant::Fusion);
  cv::Scalar lkFlowColor = toScalar(DebugVariant::LKFlow);
  cv::Scalar opencvColor = toScalar(DebugVariant::Opencv);

  // Map OdometryAlg to display color (uses variant color when available)
  auto colorForAlgorithm = [this, &toScalar](OdometryAlg alg) -> cv::Scalar {
    DebugVariant v = DebugVariant::Fusion;
    switch (alg) {
      case OdometryAlg::Blob:
        v = DebugVariant::Blob;
        break;
      case OdometryAlg::Heuristic:
        v = DebugVariant::Heuristic;
        break;
      case OdometryAlg::Neural:
        v = DebugVariant::Neural;
        break;
      case OdometryAlg::NeuralRot:
        v = DebugVariant::NeuralRot;
        break;
      case OdometryAlg::OpenCV:
        v = DebugVariant::Opencv;
        break;
      case OdometryAlg::LKFlow:
        v = DebugVariant::LKFlow;
        break;
      default:
        v = DebugVariant::Fusion;
        break;
    }
    return toScalar(v);
  };

  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  BlobDetection& _odometry_Blob = odometry.GetBlobOdometry();
  HeuristicOdometry& _odometry_Heuristic = odometry.GetHeuristicOdometry();
  CVPosition& _odometry_Neural = odometry.GetNeuralOdometry();
  LKFlowTracker& _odometry_LKFlow = odometry.GetLKFlowOdometry();
  OpenCVTracker& _odometry_OpenCV = odometry.GetOpenCVOdometry();

  _DrawAlgIfActive(_odometry_Blob, showBlob, blobColor, true);
  _DrawAlgIfActive(_odometry_Heuristic, showHeuristic, heuristicColor, true);
  _DrawAlgIfActive(_odometry_Neural, showNeural, neuralColor, false);
  _DrawAlgIfActive(_odometry_LKFlow, showLKFlow, lkFlowColor, true);
  _DrawAlgIfActive(_odometry_OpenCV, showOpencv, opencvColor, true);
  if (showFusion) {
    OdometryData robotData = odometry.Robot();
    OdometryData opponentData = odometry.Opponent();
    cv::Scalar robotPosColor = robotData.pos.has_value()
                                   ? colorForAlgorithm(robotData.pos->algorithm)
                                   : fusionColor;
    cv::Scalar opponentPosColor =
        opponentData.pos.has_value()
            ? colorForAlgorithm(opponentData.pos->algorithm)
            : fusionColor;
    cv::Scalar robotAngleColor =
        robotData.angle.has_value()
            ? colorForAlgorithm(robotData.angle->algorithm)
            : fusionColor;
    cv::Scalar opponentAngleColor =
        opponentData.angle.has_value()
            ? colorForAlgorithm(opponentData.angle->algorithm)
            : fusionColor;
    _DrawPositions(robotData, opponentData, _trackingMat, robotPosColor,
                   opponentPosColor);
    _DrawAngles(robotData, opponentData, _trackingMat, robotAngleColor,
                opponentAngleColor);
    _DrawAlgorithmLabels(robotData, opponentData, _trackingMat, robotPosColor,
                         opponentPosColor);
  }

  _HandleMouseOverInput();
}

void TrackingWidget::_HandleMouseOverInput() {
  if (!IsMouseOver()) return;
  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  ManualOverrideOdometry& overrideOdo = odometry.GetManualOverrideOdometry();
  if (!InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift)) {
    if (InputState::GetInstance().IsMouseDown(0)) {
      robotMouseClickPoint = GetMousePos();
      overrideOdo.SetPosition(
          PositionData(robotMouseClickPoint, cv::Point2f(0, 0),
                       Clock::programClock.getElapsedTime()),
          false);
    }
    if (InputState::GetInstance().IsMouseDown(1)) {
      opponentMouseClickPoint = GetMousePos();
      overrideOdo.SetPosition(
          PositionData(opponentMouseClickPoint, cv::Point2f(0, 0),
                       Clock::programClock.getElapsedTime()),
          true);
    }
  } else {
    if (InputState::GetInstance().IsMouseDown(0)) {
      cv::Point2f robotPos = odometry.Robot().GetPositionOrZero();
      cv::Point2f currMousePos = GetMousePos();
      double newAngle =
          atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
      overrideOdo.SetAngle(
          AngleData(Angle(newAngle), 0, Clock::programClock.getElapsedTime()),
          false);
      robotMouseClickAngle = newAngle;
    } else if (InputState::GetInstance().IsMouseDown(1)) {
      cv::Point2f opponentPos = odometry.Opponent().GetPositionOrZero();
      cv::Point2f currMousePos = GetMousePos();
      double newAngle =
          atan2(currMousePos.y - opponentPos.y, currMousePos.x - opponentPos.x);
      overrideOdo.SetAngle(
          AngleData(Angle(newAngle), 0, Clock::programClock.getElapsedTime()),
          true);
      opponentMouseClickAngle = newAngle;
    }
  }
}

void TrackingWidget::_DrawAngles(OdometryData& robot, OdometryData& opponent,
                                 cv::Mat& currMatt, cv::Scalar robotColor,
                                 cv::Scalar opponentColor) {
  constexpr int lineThickness = 3;
  constexpr double arrowLength = 50.0;
  if (robot.angle.has_value()) {
    double angle = robot.angle.value().angle;
    cv::Point2f pos = robot.GetPositionOrZero();
    cv::Point2f arrowEnd =
        pos + cv::Point2f(arrowLength * cos(angle), arrowLength * sin(angle));
    safe_arrow(currMatt, pos, arrowEnd, robotColor, lineThickness);
  }
  if (opponent.angle.has_value()) {
    double angle = opponent.angle.value().angle;
    cv::Point2f pos = opponent.GetPositionOrZero();
    cv::Point2f arrowEnd =
        pos + cv::Point2f(arrowLength * cos(angle), arrowLength * sin(angle));
    safe_arrow(currMatt, pos, arrowEnd, opponentColor, lineThickness);
  }
}

void TrackingWidget::_DrawPositions(OdometryData& robot, OdometryData& opponent,
                                    cv::Mat& currMatt, cv::Scalar robotColor,
                                    cv::Scalar opponentColor) {
  constexpr int sizeWithData = 20;
  if (robot.pos.has_value()) {
    OdometryData robot_ext =
        robot.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    DrawX(currMatt, robot_ext.pos.value().position, robotColor, sizeWithData);
  }
  if (opponent.pos.has_value()) {
    OdometryData opponent_ext =
        opponent.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    safe_circle(currMatt, opponent_ext.pos.value().position, sizeWithData,
                opponentColor, 2);
  }
}

void TrackingWidget::_DrawAlgorithmLabels(OdometryData& robot,
                                          OdometryData& opponent,
                                          cv::Mat& currMatt,
                                          cv::Scalar robotColor,
                                          cv::Scalar opponentColor) {
  constexpr int kLabelOffsetAbove = 28;
  constexpr double kFontScale = 0.6;
  constexpr int kFontThickness = 2;
  const int fontFace = cv::FONT_HERSHEY_SIMPLEX;

  auto drawLabelAbove = [&](cv::Point2f position, const char* label,
                            cv::Scalar color) {
    if (label == nullptr || label[0] == '\0') return;
    cv::Size textSize =
        cv::getTextSize(label, fontFace, kFontScale, kFontThickness, nullptr);
    // putText uses bottom-left origin; place baseline above the position
    int x = static_cast<int>(position.x) - textSize.width / 2;
    int y = static_cast<int>(position.y) - kLabelOffsetAbove;
    if (y - textSize.height < 0) y = textSize.height;
    if (x < 0) x = 0;
    if (x + textSize.width > currMatt.cols) x = currMatt.cols - textSize.width;
    cv::putText(currMatt, label, cv::Point(x, y), fontFace, kFontScale, color,
                kFontThickness, cv::LINE_AA);
  };

  if (robot.pos.has_value()) {
    OdometryData robot_ext =
        robot.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    const char* label = OdometryAlgToString(robot.pos->algorithm);
    drawLabelAbove(robot_ext.pos.value().position, label, robotColor);
  }
  if (opponent.pos.has_value()) {
    OdometryData opponent_ext =
        opponent.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    const char* label = OdometryAlgToString(opponent.pos->algorithm);
    drawLabelAbove(opponent_ext.pos.value().position, label, opponentColor);
  }
}

template <typename T>
void TrackingWidget::_DrawAlgIfActive(T& alg, bool show, cv::Scalar color,
                                      bool drawAngles) {
  if (!alg.IsRunning() || !show) return;
  OdometryData robotData = alg.GetData(false);
  OdometryData opponentData = alg.GetData(true);
  _DrawPositions(robotData, opponentData, _trackingMat, color, color);
  if (drawAngles)
    _DrawAngles(robotData, opponentData, _trackingMat, color, color);
}

void TrackingWidget::Update() {
  // Get the updated camera frame
  _GrabFrame();

  // Render all the frames together
  _RenderFrames();

  _cropEditor.Update(GetMousePos(), IsMouseOver(), _trackingMat);
  // Allow the user to mask out areas of the image.
  // They will be set to black when you get a frame from a camera
  _MaskOutRegions();

  _DrawAlgorithmData();

  UpdateMat(_trackingMat);

  // Save to video (if enabled)
  SaveToVideo();
}

// Store a clone of the image to avoid aliasing and lifetime issues.
void TrackingWidget::UpdateDebugImage(DebugVariant variant,
                                      const cv::Mat& image) {
  variantImages[static_cast<size_t>(variant)] =
      image.empty() ? image : image.clone();
}

void TrackingWidget::_DrawShowButton(DebugVariant variant, bool& enabledFlag) {
  const char* label = DebugVariantToString(variant);
  ImGui::PushID(label);

  ImVec4 color = variantColors[static_cast<size_t>(variant)];

  if (enabledFlag) {
    ColorScheme::PushSuccessColors();
  } else {
    ColorScheme::PushButtonColors(ColorScheme::PRIMARY_BLUE);
  }

  if (ImGui::Button(enabledFlag ? "Shown" : "Hidden")) {
    enabledFlag = !enabledFlag;
  }

  ColorScheme::PopButtonColors();

  ImGui::SameLine();
  ImGui::Text("%s", label);

  ImGui::SameLine();
  float colorArray[4] = {color.x, color.y, color.z, color.w};
  if (ImGui::ColorEdit4(("Color##" + std::string(label)).c_str(), colorArray,
                        ImGuiColorEditFlags_NoInputs)) {
    variantColors[static_cast<size_t>(variant)] =
        ImVec4(colorArray[0], colorArray[1], colorArray[2], colorArray[3]);
  }

  ImGui::SameLine();
  ImGui::PushItemWidth(60);
  cv::Point offset = variantOffsets[static_cast<size_t>(variant)];
  int offsetArray[2] = {offset.x, offset.y};
  if (ImGui::DragInt2(("Offset##" + std::string(label)).c_str(), offsetArray, 1,
                      -1000, 1000)) {
    variantOffsets[static_cast<size_t>(variant)] =
        cv::Point(offsetArray[0], offsetArray[1]);
  }
  ImGui::PopItemWidth();

  ImGui::PopID();
}

void TrackingWidget::_RenderFrames() {
  _trackingMat = cv::Mat();

  const cv::Mat& cameraImg =
      variantImages[static_cast<size_t>(DebugVariant::Camera)];
  if (cameraImg.empty()) {
    return;
  }

  cv::Size outputSize = cameraImg.size();
  _trackingMat = cv::Mat::zeros(outputSize, CV_8UC3);

  std::vector<std::pair<DebugVariant, bool>> variants = {
      {DebugVariant::Camera, showCamera},
      {DebugVariant::Blob, showBlob},
      {DebugVariant::Heuristic, showHeuristic},
      {DebugVariant::Neural, showNeural},
      {DebugVariant::Fusion, showFusion},
      {DebugVariant::NeuralRot, showNeuralRot},
      {DebugVariant::Opencv, showOpencv},
      {DebugVariant::LKFlow, showLKFlow},
  };

  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  if (showBlob) {
    odometry.GetBlobOdometry().GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::Blob)],
        variantOffsets[static_cast<size_t>(DebugVariant::Blob)]);
  }
  if (showHeuristic) {
    odometry.GetHeuristicOdometry().GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::Heuristic)],
        variantOffsets[static_cast<size_t>(DebugVariant::Heuristic)]);
  }
  if (showNeural) {
    odometry.GetNeuralOdometry().GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::Neural)],
        variantOffsets[static_cast<size_t>(DebugVariant::Neural)]);
  }
  if (showFusion) {
    odometry.GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::Fusion)],
        variantOffsets[static_cast<size_t>(DebugVariant::Fusion)]);
  }
  if (showNeuralRot) {
    odometry.GetNeuralRotOdometry().GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::NeuralRot)],
        variantOffsets[static_cast<size_t>(DebugVariant::NeuralRot)]);
  }
  if (showLKFlow) {
    odometry.GetLKFlowOdometry().GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::LKFlow)],
        variantOffsets[static_cast<size_t>(DebugVariant::LKFlow)]);
  }
  if (showOpencv) {
    odometry.GetOpenCVOdometry().GetDebugImage(
        variantImages[static_cast<size_t>(DebugVariant::Opencv)],
        variantOffsets[static_cast<size_t>(DebugVariant::Opencv)]);
  }

  std::vector<VariantLayer> layers;
  layers.reserve(variants.size());
  for (const auto& vp : variants) {
    if (!vp.second) continue;

    size_t vi = static_cast<size_t>(vp.first);
    const cv::Mat* img = &variantImages[vi];

    if (img->empty()) {
      std::cerr << "TrackingWidget: skipping " << DebugVariantToString(vp.first)
                << " (empty)" << std::endl;
      continue;
    }
    if (img->size() != outputSize) {
      std::cerr << "TrackingWidget: skipping " << DebugVariantToString(vp.first)
                << " (size mismatch " << img->cols << "x" << img->rows
                << " != " << outputSize.width << "x" << outputSize.height << ")"
                << std::endl;
      continue;
    }
    if (img->type() != CV_8UC1 && img->type() != CV_8UC3) {
      std::cerr << "TrackingWidget: skipping " << DebugVariantToString(vp.first)
                << " (unsupported type " << img->type() << ")" << std::endl;
      continue;
    }

    layers.push_back({vp.first, img, true, variantColors[vi]});
  }

  _frameCompositor.Compose(layers, outputSize, _trackingMat);
}

void TrackingWidget::AutoMatchStart(bool left) {
  // Set leftstart and rightstart to middle of background starting boxes
  cv::Point2f leftStart =
      cv::Point2f((STARTING_LEFT_TL_x + STARTING_LEFT_BR_x) / 2,
                  (STARTING_LEFT_TL_y + STARTING_LEFT_BR_y) / 2);
  cv::Point2f rightStart =
      cv::Point2f((STARTING_RIGHT_TL_x + STARTING_RIGHT_BR_x) / 2,
                  (STARTING_RIGHT_TL_y + STARTING_RIGHT_BR_y) / 2);

  // Initialize left/right click positions and angles to center of our starting
  // rectangles
  if (left) {
    robotMouseClickPoint = leftStart;
    robotMouseClickAngle = deg2rad(0);
    opponentMouseClickPoint = rightStart;
    opponentMouseClickAngle = deg2rad(-180.0f);
  } else {
    robotMouseClickPoint = rightStart;
    robotMouseClickAngle = deg2rad(-180.0f);
    opponentMouseClickPoint = leftStart;
    opponentMouseClickAngle = deg2rad(0);
  }

  RobotController::GetInstance().odometry.AutoMatchStart();
}

void TrackingWidget::DrawGUI() {
  ImGui::Begin("Tracking Config");

  _DrawShowButton(DebugVariant::Camera, showCamera);
  _DrawShowButton(DebugVariant::Blob, showBlob);
  _DrawShowButton(DebugVariant::Heuristic, showHeuristic);
  _DrawShowButton(DebugVariant::Neural, showNeural);
  _DrawShowButton(DebugVariant::NeuralRot, showNeuralRot);
  _DrawShowButton(DebugVariant::Opencv, showOpencv);
  _DrawShowButton(DebugVariant::LKFlow, showLKFlow);
  _DrawShowButton(DebugVariant::Fusion, showFusion);

  // Add two line spacers
  ImGui::Separator();

  ImGui::Dummy(ImVec2(0.0f, 30.0f));

  // ADD Heuristic status line
  ImGui::SetWindowFontScale(1.5f);

  // Create a child window with fixed height for the text
  ImGui::BeginChild("HeuristicStatus", ImVec2(0, 60.0f),
                    false);  // 60 pixels height, adjust as needed
  ImGui::TextWrapped("Heuristic Status: %s",
                     HeuristicOdometry::statusstring.c_str());
  ImGui::EndChild();

  ImGui::SetWindowFontScale(1.0f);

  // Remove ImGui::Dummy, as the child window handles spacing
  ImGui::Dummy(ImVec2(0.0f, 30.0f));

  RobotOdometry& odometry = RobotController::GetInstance().odometry;

  HeuristicOdometry& heuristic = odometry.GetHeuristicOdometry();

  // Set leftstart and rightstart to middle of background starting boxes
  cv::Point2f leftStart =
      cv::Point2f((STARTING_LEFT_TL_x + STARTING_LEFT_BR_x) / 2,
                  (STARTING_LEFT_TL_y + STARTING_LEFT_BR_y) / 2);
  cv::Point2f rightStart =
      cv::Point2f((STARTING_RIGHT_TL_x + STARTING_RIGHT_BR_x) / 2,
                  (STARTING_RIGHT_TL_y + STARTING_RIGHT_BR_y) / 2);

  ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_BLUE);
  if (ImGui::Button("Auto Start Left", ImVec2(150, 40))) {
    AutoMatchStart(true);
  }
  ImGui::PopStyleColor();

  ImGui::SameLine();
  ImGui::Text("        ");
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_RED);
  if (ImGui::Button("Auto Start Right", ImVec2(150, 40))) {
    AutoMatchStart(false);
  }

  ImGui::PopStyleColor();

  ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_BLUE);
  if (ImGui::Button("Start Left", ImVec2(150, 40))) {
    odometry.MatchStart();
  }
  ImGui::PopStyleColor();

  ImGui::SameLine();
  ImGui::Text("        ");
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_RED);
  if (ImGui::Button("Start Right", ImVec2(150, 40))) {
    odometry.MatchStart();
  }
  ImGui::PopStyleColor();

  ImGui::Dummy(ImVec2(0.0f, 20.0f));

  ImGui::Text("   ");
  ImGui::SameLine();

  ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_BLUE);
  if (ImGui::Button("Auto Recover", ImVec2(150, 40))) {
    heuristic.RecoverDetection();
  }
  ImGui::PopStyleColor();

  ImGui::Dummy(ImVec2(0.0f, 80.0f));

  // Add input field for outputVideoFile
  ImGui::Text("Output Video File:");
  ImGui::InputText("##trackvid", outputVideoFileBuffer,
                   sizeof(outputVideoFileBuffer));
  outputVideoFile = outputVideoFileBuffer;  // Update std::string from buffer

  ImGui::Dummy(ImVec2(0.0f, 10.0f));
  bool doPopStyle = false;
  // Add button to toggle save_video_enabled
  if (save_video_enabled) {
    // Push red color for button background when recording
    ColorScheme::PushErrorColors();
    doPopStyle = true;
  }
  if (ImGui::Button(save_video_enabled ? "Stop Recording"
                                       : "Start Recording")) {
    save_video_enabled = !save_video_enabled;  // Toggle the boolean
  }

  if (doPopStyle) {
    // Pop the custom color after rendering the button
    ColorScheme::PopStatusColors();
  }

  ImGui::End();
}

TrackingWidgetSettings TrackingWidget::_GetSettings() const {
  TrackingWidgetSettings s;
  s.showCamera = showCamera;
  s.showBlob = showBlob;
  s.showHeuristic = showHeuristic;
  s.showNeural = showNeural;
  s.showNeuralRot = showNeuralRot;
  s.showFusion = showFusion;
  s.showOpencv = showOpencv;
  s.showLKFlow = showLKFlow;
  s.outputVideoFile = outputVideoFile;
  s.variantColors = variantColors;
  s.variantOffsets = variantOffsets;
  return s;
}

void TrackingWidget::_ApplySettings(const TrackingWidgetSettings& s) {
  showCamera = s.showCamera;
  showBlob = s.showBlob;
  showHeuristic = s.showHeuristic;
  showNeural = s.showNeural;
  showNeuralRot = s.showNeuralRot;
  showFusion = s.showFusion;
  showOpencv = s.showOpencv;
  showLKFlow = s.showLKFlow;
  outputVideoFile = s.outputVideoFile;
  variantColors = s.variantColors;
  variantOffsets = s.variantOffsets;

  strncpy(outputVideoFileBuffer, outputVideoFile.c_str(),
          sizeof(outputVideoFileBuffer) - 1);
  outputVideoFileBuffer[sizeof(outputVideoFileBuffer) - 1] = '\0';
}

std::string TrackingWidget::SaveGUISettings() {
  return SerializeTrackingWidgetSettings(_GetSettings());
}

void TrackingWidget::RestoreGUISettings(const std::string& settings) {
  TrackingWidgetSettings s =
      DeserializeTrackingWidgetSettings(settings, _GetSettings());
  _ApplySettings(s);
}

void TrackingWidget::SaveToVideo() {
  if (!save_video_enabled) {
    if (save_video_enabled_old && video.isOpened()) {
      // If we were previously saving, close the video file
      video.release();
    }

    save_video_enabled_old = false;
    return;
  }

  // Try avc1 firST
  if (!video.isOpened()) {
    int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    video.open(outputVideoFile, fourcc, 60.0,
               cv::Size(_trackingMat.cols, _trackingMat.rows), true);
  }

  // Make sure we are initialized
  if (!video.isOpened()) {
    // Try XVID codec
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    video.open(outputVideoFile, fourcc, 60.0,
               cv::Size(_trackingMat.cols, _trackingMat.rows), true);
    if (!video.isOpened()) {
      // Fallback to MJPG
      fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
      video.open(outputVideoFile, fourcc, 60.0,
                 cv::Size(_trackingMat.cols, _trackingMat.rows), true);
      if (!video.isOpened()) {
        save_video_enabled = false;
        return;
      }
    }
  }

  save_video_enabled_old = true;
  video.write(_trackingMat);
}
