#pragma once

#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#define GL_BGRA 0x80E1
#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

bool SpinBox(const char* label, int* value, int step = 1, int min_value = INT_MIN, int max_value = INT_MAX);
ImTextureID MatToTexture(const cv::Mat& mat);
void ClearLastFrameTextures();