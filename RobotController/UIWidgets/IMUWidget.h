#pragma once

// IMUWiget.h
// The imu widget is an opencv mat that visualizes the imu data
// There is a circle outline (filling up the mat) and a crosshair in the middle
// There is a small dot that shows the current acceleration as a 2d vector

#include <opencv2/opencv.hpp>
#include <mutex>

class IMUWidget
{

public:
    IMUWidget();
    static IMUWidget& GetInstance();
    void SetMat(const cv::Mat& mat);

    void Update();
    cv::Mat& Draw();

private:
    static IMUWidget* _instance;

    cv::Mat _mat;
    std::mutex _matMutex;
};
