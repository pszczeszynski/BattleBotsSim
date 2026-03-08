#pragma once

#include <array>
#include <opencv2/opencv.hpp>
#include <string>

#include "../Globals.h"
#include "../RobotOdometry.h"
#include "DebugVariant.h"
#include "FrameCompositor.h"
#include "ImageWidget.h"
#include "TrackingFieldCropEditor.h"
#include "TrackingFieldMaskEditor.h"
#include "TrackingWidgetSettings.h"
#include "imgui.h"

/**
 * This is the widget for correcting + adjusting the odometry
 * tracking algorithms + the field warping
 */

class TrackingWidget : public ImageWidget {
 public:
  TrackingWidget();
  void ClearMask();
  cv::Mat& GetMask();
  void Update();
  void DrawGUI();
  void AutoMatchStart(bool left);

  static TrackingWidget* GetInstance();
  static cv::Point2f robotMouseClickPoint;
  static cv::Point2f opponentMouseClickPoint;
  static double robotMouseClickAngle;
  static double opponentMouseClickAngle;

  void UpdateDebugImage(DebugVariant variant, const cv::Mat& image);
  std::string SaveGUISettings();
  void RestoreGUISettings(const std::string& settings);

  std::string outputVideoFile = "Recordings/Tracking_dataDump.mp4";

 private:
  TrackingWidgetSettings _GetSettings() const;
  void _ApplySettings(const TrackingWidgetSettings& s);

  void _GrabFrame();
  void _DrawAlgorithmData();
  void _MaskOutRegions();
  void _RenderFrames();

  static TrackingWidget* _instance;

  int _lastFrameId = 0;

  cv::Mat _fieldMask;
  TrackingFieldCropEditor _cropEditor;
  TrackingFieldMaskEditor _maskEditor;
  cv::Mat _trackingMat{WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)};
  FrameCompositor _frameCompositor;

  bool showCamera = true;
  bool showBlob = false;
  bool showHeuristic = false;
  bool showNeural = false;
  bool showNeuralRot = false;
  bool showFusion = false;
  bool showOpencv = false;
  bool showLKFlow = false;

  // Store colors, images, and offsets per variant (indexed by DebugVariant)
  static constexpr size_t kDebugVariantCount =
      static_cast<size_t>(DebugVariant::kCount);
  std::array<ImVec4, kDebugVariantCount> variantColors{};
  std::array<cv::Mat, kDebugVariantCount> variantImages{};
  std::array<cv::Point, kDebugVariantCount> variantOffsets{};

  void _DrawShowButton(DebugVariant variant, bool& enabledFlag);
  void _DrawAngles(OdometryData& robot, OdometryData& opponent,
                   cv::Mat& currMatt, cv::Scalar robotColor,
                   cv::Scalar opponentColor);
  void _DrawPositions(OdometryData& robot, OdometryData& opponent,
                      cv::Mat& currMatt, cv::Scalar robotColor,
                      cv::Scalar opponentColor);
  void _DrawAlgorithmLabels(OdometryData& robot, OdometryData& opponent,
                            cv::Mat& currMatt, cv::Scalar robotColor,
                            cv::Scalar opponentColor);
  template <typename T>
  void _DrawAlgIfActive(T& alg, bool show, cv::Scalar color, bool drawAngles);
  void _DrawTimestamp();
  void _HandleMouseOverInput();

  // Dump to video functions
  char outputVideoFileBuffer[256] = "";  // Buffer for ImGui text input
  cv::VideoWriter video;

  // for edge detection on the save video button
  bool _save_camera_stream = false;
  bool _save_camera_stream_prev = false;

  cv::Mat _rawCameraFrame;
  void SaveToVideo();
};