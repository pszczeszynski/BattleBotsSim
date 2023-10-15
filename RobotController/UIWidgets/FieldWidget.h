#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"

class FieldWidget
{
public:
    // singleton
    static FieldWidget& GetInstance();
    void Draw();

    cv::Point2f GetMousePosOnField();

private:
    FieldWidget();
    ImVec2 _windowPos;
};