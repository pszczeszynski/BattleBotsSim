#pragma once
#include <opencv2/opencv.hpp>

/**
 * @brief A motion blob is a connected component in the image
 * It is only exists in one frame and has no history.
 */
struct MotionBlob
{
public:
    cv::Rect rect;
    cv::Point2f center; // weighted center (!= rect.center)
    cv::Mat *frame;
    double rotation = 0; // in radians
};
