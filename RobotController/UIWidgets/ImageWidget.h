#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include <mutex>
#include <vector>

/**
 * A widget that displays an image in a window
 * This class is thread safe
*/
class ImageWidget
{
public:
    ImageWidget(std::string name, cv::Mat& image, bool moveable = false);
    ImageWidget(std::string name, bool moveable = false);
    ~ImageWidget();
    static std::vector<ImageWidget*>& Instances();
    virtual void Draw();
    cv::Point2f GetMousePos();
    bool IsMouseOver();
    void UpdateMat(cv::Mat& image);
    void AddAdditionalUI(std::function<void()> func);

protected:
    ImVec2 _windowPos;
    std::string _name;
    bool _moveable;

    std::mutex _imageMutex;
    cv::Mat _image;

    std::function<void()> _additionalUI;
};