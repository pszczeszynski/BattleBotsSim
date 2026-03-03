#pragma once

#include <GLFW/glfw3.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "CameraWidget.h"
#include "ConfigWidget.h"
#include "IMUWidget.h"
#include "ManualControlWidget.h"
#include "PlaybackWidget.h"
#include "PurePursuitRadiusWidget.h"
#include "RobotTelemetryWidget.h"
#include "StatusIndicatorWidget.h"
#include "VariantsWidget.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"

class RobotControllerGUI {
 public:
  RobotControllerGUI();
  bool Update();
  void Shutdown();

  // singleton
  static RobotControllerGUI &GetInstance();

 private:
  void Render();
  GLFWwindow *SetupWindow();
  void SetupIMGUI(GLFWwindow *window);
  void InitializeImGUIFrame();
  void Render(GLFWwindow *window, ImVec4 clearColor);

  GLFWwindow *window;

  IMUWidget _imuWidget;
  ConfigWidget _configWidget;
  RobotTelemetryWidget _robotTelemetryWidget;
  StatusIndicatorWidget _statusIndicatorWidget;
  VariantsWidget _variantsWidget;
  PlaybackWidget _playbackWidget;
  ManualControlWidget _manualControlWidget;
  CameraWidget _cameraWidget;
  PurePursuitRadiusWidget _purePursuitRadiusWidget;
};

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}
