#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"

class FieldWidget : public ImageWidget
{
public:
    void Draw() override;
    FieldWidget();

private:
    void _AdjustFieldCrop();
};