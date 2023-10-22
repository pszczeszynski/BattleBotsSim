#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"

class FieldWidget : public ImageWidget
{
public:
    // singleton
    static FieldWidget& GetInstance();

    void Draw() override;

private:
    FieldWidget();
};