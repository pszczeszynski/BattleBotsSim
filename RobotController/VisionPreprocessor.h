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
    void Preprocess(cv::Mat &frame, cv::Mat &dst);
    cv::Point2f TransformPoint(const cv::Point2f &pt);

private:
    void _StabalizeImage(cv::Mat &frame, cv::Mat &dst);
    cv::Point2f _TrackFeature(cv::Mat &prevFrame, cv::Mat &frame, cv::Rect &roi);
    void ComputeTransformationMatrix();

    cv::Point2f _dstPoints[4];
    cv::Mat _prevFrame;
    cv::Mat _transformationMatrix;
};
