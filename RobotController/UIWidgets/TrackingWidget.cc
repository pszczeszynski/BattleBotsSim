#include "TrackingWidget.h"

#include <algorithm>

#include "../Clock.h"
#include "../Input/InputState.h"
#include "../Odometry/OdometryData.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "../SafeDrawing.h"
#include "CameraWidget.h"
#include "ColorScheme.h"

TrackingWidget* TrackingWidget::_instance = nullptr;
cv::Point2f TrackingWidget::robotMouseClickPoint = cv::Point2f(0, 0);
cv::Point2f TrackingWidget::opponentMouseClickPoint = cv::Point2f(0, 0);
double TrackingWidget::robotMouseClickAngle = 0;
double TrackingWidget::opponentMouseClickAngle = 0;

#define MASK_PATH "./backgrounds/fieldMask.jpg"

TrackingWidget::TrackingWidget()
    :  // initialize the mask to all visible (0 = visible, 255 = masked)
      _fieldMask{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}},
      ImageWidget("Tracking", _trackingMat, false) {
  _instance = this;

  std::filesystem::path dirPath = MASK_PATH;

  if (std::filesystem::exists(dirPath)) {
    cv::Mat loaded = cv::imread(MASK_PATH);
    if (!loaded.empty()) {
      if (loaded.channels() == 1) {
        _fieldMask = loaded.clone();
      } else if (loaded.channels() == 3 || loaded.channels() == 4) {
        cv::cvtColor(loaded, _fieldMask, loaded.channels() == 3
                                              ? cv::COLOR_BGR2GRAY
                                              : cv::COLOR_BGRA2GRAY);
      } else {
        _fieldMask = cv::Mat{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}};
      }
      if (_fieldMask.type() != CV_8UC1 || _fieldMask.cols != WIDTH ||
          _fieldMask.rows != HEIGHT) {
        cv::Mat resized;
        cv::resize(_fieldMask, resized, cv::Size(WIDTH, HEIGHT));
        _fieldMask = resized;
      }
    }
  }

  // Initialize default colors for each variant using our color scheme
  for (size_t i = 0; i < kDebugVariantCount; ++i) {
    DebugVariant v = static_cast<DebugVariant>(i);
    variantColors[i] = ColorScheme::GetVariantColor(v);
    variantOffsets[i] = cv::Point(0, 0);
  }

  RestoreGUISettings(VISION_TRACKING_GUI);
}

void TrackingWidget::_GrabFrame() {
  static int last_id = 0;

  // 1. populate the tracking mat (for display purposes)
  static ICameraReceiver* camera = ICameraReceiver::GetInstance();
  if (camera != nullptr && camera->NewFrameReady(last_id)) {
    last_id = camera->GetFrame(GetDebugImage(DebugVariant::Camera), last_id);
  }
}

void TrackingWidget::_AdjustFieldCrop() {
  bool enableCrop = !CameraWidget::LockCamera;

  static int cornerToAdjust = -1;
  static cv::Point2f mousePosLast = cv::Point2f(0, 0);
  static cv::Point2f cornerHandles[4] = {
      cv::Point2f(0, 0), cv::Point2f(WIDTH, 0), cv::Point2f(WIDTH, HEIGHT),
      cv::Point2f(0, HEIGHT)};

  const double CORNER_DIST_THRESH = 20.0;

  // get the curr mouse position
  cv::Point2f currMousePos = GetMousePos();

  cv::Mat& drawingImage = _trackingMat;

  // draw the corners
  if (enableCrop && !drawingImage.empty()) {
    for (int i = 0; i < 4; i++) {
      if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH) {
        safe_circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH * 1.5,
                    cv::Scalar(255, 100, 255), 2);
      } else {
        safe_circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH,
                    cv::Scalar(255, 0, 255), 2);
      }
    }
  }

  // if the user isn't pressing shift and is over the image
  if (!InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift)) {
    // corner adjustment

    // If the user left clicks near one of the corners
    if (enableCrop && InputState::GetInstance().IsMouseDown(0)) {
      if (cornerToAdjust == -1) {
        // Check each corner
        for (int i = 0; i < 4; i++) {
          // if the user is near a corner
          if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH) {
            // set the corner to adjust
            cornerToAdjust = i;
            break;
          }
        }
      }
    } else {
      // otherwise set the corner to adjust to -1
      cornerToAdjust = -1;
    }

    // adjust the corner
    cv::Point2f adjustment = mousePosLast - currMousePos;

    if (cornerToAdjust == 0) {
      preprocess_tl_x += adjustment.x;
      preprocess_tl_y += adjustment.y;
    } else if (cornerToAdjust == 1) {
      preprocess_tr_x += adjustment.x;
      preprocess_tr_y += adjustment.y;
    } else if (cornerToAdjust == 2) {
      preprocess_br_x += adjustment.x;
      preprocess_br_y += adjustment.y;
    } else if (cornerToAdjust == 3) {
      preprocess_bl_x += adjustment.x;
      preprocess_bl_y += adjustment.y;
    }
  }

  // save the last mouse position
  mousePosLast = currMousePos;
}

TrackingWidget* TrackingWidget::GetInstance() { return _instance; }

enum class MaskState { LOCKED = 0, WAITING_FOR_CLICK, DRAGGING };

/**
 * Allows the user to mask out regions of the image by drawing rectangles.
 * Future camera sources will have these regions blacked out.
 */
void TrackingWidget::_MaskOutRegions() {
  static MaskState state{MaskState::LOCKED};
  static cv::Point2f topLeft{0, 0};
  static cv::Point2f bottomRight{0, 0};

  // if user clicks checkbox to draw the mask, go to the waiting for click state
  if (CameraWidget::DrawMask) {
    if (state == MaskState::LOCKED) {
      state = MaskState::WAITING_FOR_CLICK;
    }
  }
  // otherwise go to locked
  else {
    state = MaskState::LOCKED;
  }

  // if waiting for a click
  if (state == MaskState::WAITING_FOR_CLICK) {
    // check if the left mouse button is down
    if (InputState::GetInstance().IsMouseDown(0) && IsMouseOver()) {
      state = MaskState::DRAGGING;
      topLeft = GetMousePos();
    }
  }

  // if dragging the rectangle
  if (state == MaskState::DRAGGING) {
    bottomRight = GetMousePos();

    // check if the left mouse button is released
    if (!InputState::GetInstance().IsMouseDown(0)) {
      // Normalize so width/height are positive regardless of drag direction
      int x1 = static_cast<int>((std::min)(topLeft.x, bottomRight.x));
      int y1 = static_cast<int>((std::min)(topLeft.y, bottomRight.y));
      int x2 = static_cast<int>((std::max)(topLeft.x, bottomRight.x));
      int y2 = static_cast<int>((std::max)(topLeft.y, bottomRight.y));
      cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
      cv::Rect maskBounds(0, 0, _fieldMask.cols, _fieldMask.rows);
      cv::Rect clipped(rect & maskBounds);

      if (clipped.width > 0 && clipped.height > 0) {
        cv::rectangle(_fieldMask, clipped, cv::Scalar(255), -1);
        cv::imwrite(MASK_PATH, _fieldMask);
      }

      state = MaskState::WAITING_FOR_CLICK;
    }
  }
}

/**
 * Clears the field mask to all visible (0 = visible, 255 = masked).
 */
void TrackingWidget::ClearMask() {
  _fieldMask = cv::Mat{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}};
  cv::imwrite(MASK_PATH, _fieldMask);
}

cv::Mat& TrackingWidget::GetMask() { return _fieldMask; }

void DrawX(cv::Mat& mat, cv::Point2f pos, cv::Scalar color, int size) {
  cv::line(mat, pos + cv::Point2f(-size, -size), pos + cv::Point2f(size, size),
           color, 2);
  cv::line(mat, pos + cv::Point2f(-size, size), pos + cv::Point2f(size, -size),
           color, 2);
}

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
  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  BlobDetection& _odometry_Blob = odometry.GetBlobOdometry();
  HeuristicOdometry& _odometry_Heuristic = odometry.GetHeuristicOdometry();
  CVPosition& _odometry_Neural = odometry.GetNeuralOdometry();
  LKFlowTracker& _odometry_LKFlow = odometry.GetLKFlowOdometry();

  _DrawAlgIfActive(_odometry_Blob, showBlob, blobColor, true);
  _DrawAlgIfActive(_odometry_Heuristic, showHeuristic, heuristicColor, true);
  _DrawAlgIfActive(_odometry_Neural, showNeural, neuralColor, false);
  _DrawAlgIfActive(_odometry_LKFlow, showLKFlow, lkFlowColor, true);
  if (showFusion) {
    OdometryData robotData = odometry.Robot();
    OdometryData opponentData = odometry.Opponent();
    _DrawPositions(robotData, opponentData, _trackingMat, fusionColor);
    _DrawAngles(robotData, opponentData, _trackingMat, fusionColor);
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
                                 cv::Mat& currMatt, cv::Scalar arrowColor) {
  constexpr int lineThickness = 3;
  constexpr double arrowLength = 50.0;
  for (OdometryData* data : {&robot, &opponent}) {
    if (!data->angle.has_value()) continue;
    double angle = data->angle.value().angle;
    cv::Point2f pos = data->GetPositionOrZero();
    cv::Point2f arrowEnd =
        pos + cv::Point2f(arrowLength * cos(angle), arrowLength * sin(angle));
    safe_arrow(currMatt, pos, arrowEnd, arrowColor, lineThickness);
  }
}

void TrackingWidget::_DrawPositions(OdometryData& robot, OdometryData& opponent,
                                    cv::Mat& currMatt, cv::Scalar arrowColor) {
  constexpr int sizeWithData = 20;
  if (robot.pos.has_value()) {
    OdometryData robot_ext =
        robot.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    DrawX(currMatt, robot_ext.pos.value().position, arrowColor, sizeWithData);
  }
  if (opponent.pos.has_value()) {
    OdometryData opponent_ext =
        opponent.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    safe_circle(currMatt, opponent_ext.pos.value().position, sizeWithData,
                arrowColor, 2);
  }
}

template <typename T>
void TrackingWidget::_DrawAlgIfActive(T& alg, bool show, cv::Scalar color,
                                    bool drawAngles) {
  if (!alg.IsRunning() || !show) return;
  OdometryData robotData = alg.GetData(false);
  OdometryData opponentData = alg.GetData(true);
  _DrawPositions(robotData, opponentData, _trackingMat, color);
  if (drawAngles) _DrawAngles(robotData, opponentData, _trackingMat, color);
}

void TrackingWidget::Update() {
  // Get the updated camera frame
  _GrabFrame();

  // Render all the frames together
  _RenderFrames();

  _AdjustFieldCrop();
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
  variantImages[static_cast<size_t>(variant)] = image.empty() ? image : image.clone();
}

cv::Mat& TrackingWidget::GetDebugImage(DebugVariant variant) {
  size_t i = static_cast<size_t>(variant);
  if (variantImages[i].empty()) {
    variantImages[i] = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0));
  }
  return variantImages[i];
}

cv::Point TrackingWidget::GetDebugOffset(DebugVariant variant) {
  return variantOffsets[static_cast<size_t>(variant)];
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
  // Initialize the output image as empty
  _trackingMat = cv::Mat();

  // List of variants and their visibility flags
  std::vector<std::pair<DebugVariant, bool>> variants = {
      {DebugVariant::Camera, showCamera},
      {DebugVariant::Blob, showBlob},
      {DebugVariant::Heuristic, showHeuristic},
      {DebugVariant::Neural, showNeural},
      {DebugVariant::Fusion, showFusion},
      {DebugVariant::NeuralRot, showNeuralRot},
#ifdef USE_OPENCV_TRACKER
      {DebugVariant::Opencv, showOpencv},
#endif
      {DebugVariant::LKFlow, showLKFlow},
  };

  cv::Size outputSize = GetDebugImage(DebugVariant::Camera).size();

  RobotOdometry& odometry = RobotController::GetInstance().odometry;
  if (showBlob) {
    odometry.GetBlobOdometry().GetDebugImage(
        GetDebugImage(DebugVariant::Blob), GetDebugOffset(DebugVariant::Blob));
  }
  if (showHeuristic) {
    odometry.GetHeuristicOdometry().GetDebugImage(
        GetDebugImage(DebugVariant::Heuristic),
        GetDebugOffset(DebugVariant::Heuristic));
  }
  if (showNeural) {
    odometry.GetNeuralOdometry().GetDebugImage(
        GetDebugImage(DebugVariant::Neural),
        GetDebugOffset(DebugVariant::Neural));
  }
  if (showFusion) {
    odometry.GetDebugImage(GetDebugImage(DebugVariant::Fusion),
                           GetDebugOffset(DebugVariant::Fusion));
  }
  if (showNeuralRot) {
    odometry.GetNeuralRotOdometry().GetDebugImage(
        GetDebugImage(DebugVariant::NeuralRot),
        GetDebugOffset(DebugVariant::NeuralRot));
  }
  if (showLKFlow) {
    odometry.GetLKFlowOdometry().GetDebugImage(
        GetDebugImage(DebugVariant::LKFlow),
        GetDebugOffset(DebugVariant::LKFlow));
  }
#ifdef USE_OPENCV_TRACKER
  if (showOpencv) {
    odometry.GetOpenCVOdometry().GetDebugImage(
        GetDebugImage(DebugVariant::Opencv),
        GetDebugOffset(DebugVariant::Opencv));
  }
#endif

  // Initialize output image as a black color image (CV_8UC3)
  _trackingMat = cv::Mat::zeros(outputSize, CV_8UC3);

  // Reusable buffers to avoid per-frame allocations
  cv::Mat colorized, mask, temp3channel, grayTemp, blended, maskedColorized,
      inverseMask;
  colorized.create(outputSize, CV_8UC3);
  mask.create(outputSize, CV_8UC1);
  temp3channel.create(outputSize, CV_8UC3);
  grayTemp.create(outputSize, CV_8UC1);
  blended.create(outputSize, CV_8UC3);
  maskedColorized.create(outputSize, CV_8UC3);
  inverseMask.create(outputSize, CV_8UC1);

  for (const auto& variant : variants) {
    DebugVariant v = variant.first;
    bool isVisible = variant.second;
    size_t vi = static_cast<size_t>(v);

    if (!isVisible || variantImages[vi].empty()) {
      continue;
    }

    const cv::Mat& srcImage = variantImages[vi];
    if (srcImage.cols != outputSize.width ||
        srcImage.rows != outputSize.height) {
      continue;
    }

    ImVec4 color = variantColors[vi];
    cv::Scalar bgrColor(color.z * 255, color.y * 255,
                        color.x * 255);  // Convert RGB to BGR for OpenCV
    float alpha = color.w;               // Use the alpha value from ImVec4

    if (srcImage.type() == CV_8UC1) {  // Grayscale or binary mask
      cv::cvtColor(srcImage, temp3channel, cv::COLOR_GRAY2BGR);

      std::vector<cv::Mat> channels(3);
      cv::split(temp3channel, channels);

      channels[0] *= (bgrColor[0] / 255.0);
      channels[1] *= (bgrColor[1] / 255.0);
      channels[2] *= (bgrColor[2] / 255.0);

      cv::merge(channels, colorized);

      cv::threshold(srcImage, mask, 0, 255, cv::THRESH_BINARY);

    } else if (srcImage.type() == CV_8UC3) {  // Already a color image
      srcImage.copyTo(colorized);

      std::vector<cv::Mat> channels(3);
      cv::split(colorized, channels);

      channels[0] *= (bgrColor[0] / 255.0);
      channels[1] *= (bgrColor[1] / 255.0);
      channels[2] *= (bgrColor[2] / 255.0);

      cv::merge(channels, colorized);

      cv::cvtColor(srcImage, grayTemp, cv::COLOR_BGR2GRAY);
      cv::threshold(grayTemp, mask, 0, 255, cv::THRESH_BINARY);

    } else {
      continue;  // Unsupported image type
    }

    if (alpha >= 0.99f) {
      colorized.copyTo(_trackingMat, mask);
    } else {
      maskedColorized.setTo(cv::Scalar(0, 0, 0));
      colorized.copyTo(maskedColorized, mask);

      cv::bitwise_not(mask, inverseMask);

      cv::addWeighted(_trackingMat, 1.0 - alpha, maskedColorized, alpha, 0,
                      blended);

      blended.copyTo(_trackingMat, mask);
    }
  }
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
#ifdef USE_OPENCV_TRACKER
  _DrawShowButton(DebugVariant::Opencv, showOpencv);
#endif
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

std::string TrackingWidget::SaveGUISettings() {
  std::stringstream ss;

  // Save boolean flags
  ss << "showCamera=" << (showCamera ? "1" : "0") << ";";
  ss << "showBlob=" << (showBlob ? "1" : "0") << ";";
  ss << "showHeuristic=" << (showHeuristic ? "1" : "0") << ";";
  ss << "showNeural=" << (showNeural ? "1" : "0") << ";";
  ss << "showNeuralRot=" << (showNeuralRot ? "1" : "0") << ";";
  ss << "showFusion=" << (showFusion ? "1" : "0") << ";";
  ss << "showOpencv=" << (showOpencv ? "1" : "0") << ";";
  ss << "showLKFlow=" << (showLKFlow ? "1" : "0") << ";";

  // Save outputVideoFile
  ss << "outputVideoFile=" << outputVideoFile << ";";

  ss << "variantColors=";
  for (size_t i = 0; i < kDebugVariantCount; ++i) {
    const ImVec4& c = variantColors[i];
    ss << DebugVariantToString(static_cast<DebugVariant>(i)) << ":" << c.x
       << "," << c.y << "," << c.z << "," << c.w << "|";
  }
  ss << ";";

  ss << "variantOffsets=";
  for (size_t i = 0; i < kDebugVariantCount; ++i) {
    const cv::Point& p = variantOffsets[i];
    ss << DebugVariantToString(static_cast<DebugVariant>(i)) << ":" << p.x
       << "," << p.y << "|";
  }
  ss << ";";

  return ss.str();
}

void TrackingWidget::RestoreGUISettings(const std::string& settings) {
  std::stringstream ss(settings);
  std::string token;

  // Helper function to split a string by delimiter
  auto split = [](const std::string& s,
                  char delim) -> std::vector<std::string> {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
      result.push_back(item);
    }
    return result;
  };

  while (std::getline(ss, token, ';')) {
    if (token.empty()) continue;
    auto keyValue = split(token, '=');
    if (keyValue.size() != 2) continue;
    const std::string& key = keyValue[0];
    const std::string& value = keyValue[1];

    // Restore boolean flags
    if (key == "showCamera")
      showCamera = (value == "1");
    else if (key == "showBlob")
      showBlob = (value == "1");
    else if (key == "showHeuristic")
      showHeuristic = (value == "1");
    else if (key == "showNeural")
      showNeural = (value == "1");
    else if (key == "showNeuralRot")
      showNeuralRot = (value == "1");
    else if (key == "showFusion")
      showFusion = (value == "1");
    else if (key == "showOpencv")
      showOpencv = (value == "1");
    else if (key == "showLKFlow")
      showLKFlow = (value == "1");

    // Restore outputVideoFile
    else if (key == "outputVideoFile") {
      outputVideoFile = value;
      strncpy(outputVideoFileBuffer, value.c_str(),
              sizeof(outputVideoFileBuffer) - 1);
      outputVideoFileBuffer[sizeof(outputVideoFileBuffer) - 1] =
          '\0';  // Ensure null-termination
    }

    else if (key == "variantColors") {
      auto colorEntries = split(value, '|');
      for (const auto& entry : colorEntries) {
        if (entry.empty()) continue;
        auto colorData = split(entry, ':');
        if (colorData.size() != 2) continue;
        auto colors = split(colorData[1], ',');
        if (colors.size() != 4) continue;
        size_t i = static_cast<size_t>(DebugVariantFromString(colorData[0]));
        if (i < kDebugVariantCount) {
          variantColors[i] = ImVec4(std::stof(colors[0]), std::stof(colors[1]),
                                    std::stof(colors[2]), std::stof(colors[3]));
        }
      }
    }

    else if (key == "variantOffsets") {
      auto offsetEntries = split(value, '|');
      for (const auto& entry : offsetEntries) {
        if (entry.empty()) continue;
        auto offsetData = split(entry, ':');
        if (offsetData.size() != 2) continue;
        auto offsets = split(offsetData[1], ',');
        if (offsets.size() != 2) continue;
        size_t i = static_cast<size_t>(DebugVariantFromString(offsetData[0]));
        if (i < kDebugVariantCount) {
          variantOffsets[i] =
              cv::Point(std::stoi(offsets[0]), std::stoi(offsets[1]));
        }
      }
    }
  }
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
