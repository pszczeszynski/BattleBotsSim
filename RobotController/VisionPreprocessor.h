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
};