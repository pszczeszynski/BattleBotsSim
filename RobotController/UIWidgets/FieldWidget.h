#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"

class FieldWidget : public ImageWidget
{
public:
    FieldWidget();
    static FieldWidget* GetInstance();
    static FieldWidget* _instance;
};