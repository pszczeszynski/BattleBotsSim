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
  cv::Mat& GetDebugImage(DebugVariant variant);
  cv::Point GetDebugOffset(DebugVariant variant);
  std::string SaveGUISettings();
  void RestoreGUISettings(const std::string& settings);

  bool save_video_enabled = false;
  std::string outputVideoFile = "Recordings/Tracking_dataDump.mp4";

 private:
  void _GrabFrame();
  void _DrawAlgorithmData();
  void _MaskOutRegions();
  void _RenderFrames();

  static TrackingWidget* _instance;

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
  void _HandleMouseOverInput();

  // Dump to video functions
  char outputVideoFileBuffer[256] = "";  // Buffer for ImGui text input
  cv::VideoWriter video;
  bool save_video_enabled_old = false;
  void SaveToVideo();
};