#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "ConfigWidget.h"
#include "FieldWidget.h"
#include "IMUWidget.h"
#include "RobotTelemetryWidget.h"

#include <stdio.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <opencv2/opencv.hpp>
#include "ImageWidget.h"
#include "KillWidget.h"
#include "FieldWidget.h"
#include "VariantsWidget.h"
#include "PlaybackWidget.h"
#include "ManualControlWidget.h"

////////// GLOBALS //////////

#define IMGUI_WIDTH 1
#define IMGUI_HEIGHT 1
#define IMGUI_ENABLE_VSYNC 1

class RobotControllerGUI
{
public:
    RobotControllerGUI();
    bool Update();
    void Shutdown();

    // singleton
    static RobotControllerGUI& GetInstance();

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
    VariantsWidget _variantsWidget;
    PlaybackWidget _playbackWidget;
    ManualControlWidget _manualControlWidget;
};

static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}




