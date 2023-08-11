/**
 * This class converts the image to a birds eye view just using a linear transformation
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "Globals.h"

class VisionPreprocessor
{
public:
    VisionPreprocessor();
    void Preprocess(cv::Mat& frame, cv::Mat& dst);
    void MoveSourcePoint(int index, cv::Point2f displacement);

private:
    cv::Point2f srcPoints[4];
    cv::Point2f dstPoints[4];
    cv::Point2f _mousePosLast;
    bool down[4];
};