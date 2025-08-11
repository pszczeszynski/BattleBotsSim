#pragma once

// IMUWidget.h
// The IMU widget now uses ImGui native drawing to create a modern, visually stunning interface
// Features include: acceleration vector visualization, rotation indicators, health status,
// and comprehensive debug data display with our custom color scheme

#include <mutex>
#include "ImageWidget.h"

class IMUWidget : public ImageWidget
{
public:
    IMUWidget();
    void Draw() override;
};
