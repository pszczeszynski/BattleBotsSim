#pragma once

// IMUWiget.h
// The imu widget is an opencv mat that visualizes the imu data
// There is a circle outline (filling up the mat) and a crosshair in the middle
// There is a small dot that shows the current acceleration as a 2d vector

#include <opencv2/opencv.hpp>
#include <mutex>
#include "ImageWidget.h"

class IMUWidget : public ImageWidget
{
public:
    IMUWidget();
    void Draw() override;
};
