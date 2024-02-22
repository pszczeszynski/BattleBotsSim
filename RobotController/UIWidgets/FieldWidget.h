#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"

enum DrawingBoundaryState
{
    IDLE,
    WAIT_FOR_CLICK,
    WAIT_FOR_RELEASE,
    DRAGGING
};

class FieldWidget : public ImageWidget
{
public:
    void StartDrawingBoundary();
    FieldWidget();
    void AdjustFieldCrop();

    static FieldWidget* GetInstance();
    static FieldWidget* _instance;

private:

    DrawingBoundaryState _boundaryState = IDLE;

};