#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"


class FieldWidget : public ImageWidget
{
public:
    void StartDrawingBoundary();
    FieldWidget();
    void AdjustFieldCrop();

    static FieldWidget* GetInstance();
    static FieldWidget* _instance;
    static cv::Point2f leftClickPoint;
    static cv::Point2f rightClickPoint;

};