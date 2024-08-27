#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"
#include "../Globals.h"


/**
 * This is the widget for correcting + adjusting the odometry
 * tracking algorithms + the field warping
 */

class TrackingWidget : public ImageWidget
{
public:
    TrackingWidget();
    void ClearMask();
    cv::Mat& GetMask();
    cv::Mat& GetTrackingMat();
    void Update();

    static TrackingWidget* GetInstance();
    static cv::Point2f leftClickPoint;
    static cv::Point2f rightClickPoint;

private:
    void _GrabFrame();
    void _DrawAlgorithmData();
    void _AdjustFieldCrop();
    void _MaskOutRegions();

    static TrackingWidget* _instance;

    cv::Mat _fieldMask;
    cv::Mat _trackingMat{WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)};
};