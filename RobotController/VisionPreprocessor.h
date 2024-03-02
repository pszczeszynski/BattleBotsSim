/**
 * This class converts the image to a birds eye view just using a linear transformation
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "Globals.h"
#include "ThreadPool.h"
#include "UIWidgets/ClockWidget.h"

class VisionPreprocessor
{
public:
    VisionPreprocessor();
    void Preprocess(cv::Mat &frame, cv::Mat &dst);
    cv::Point2f TransformPoint(const cv::Point2f &pt);
    float fisheyeScale = 0;


private:
    void _StabalizeImage(cv::Mat &frame, cv::Mat &dst);
    cv::Point2f _TrackFeature(cv::Mat &prevFrame, cv::Mat &frame, cv::Rect &roi);
    void ComputeTransformationMatrix();

    // Fisheye Complex Parameters
    void _generateCameraParameters(float scaler_fl, float scaler_intensity, float scaler_y, cv::Size imageSize, cv::Matx33d &K, cv::Vec4d &D) ;
    // Fisheye Parameters
    cv::Matx33d K;
    cv::Vec4d D;


    // PoorMans Fisheye
    std::vector<std::vector<cv::Point2f>> srcPointsList;
    std::vector<std::vector<cv::Point2f>> dstPointsList;
    float gridSize = 5; // 5x5 grid
    void ComputePMFisheyePointList(cv::Mat& image);
    cv::Point2f GetPMFisheyeStartPoint(cv::Point2f& point);
    void DoPMFishEye(cv::Mat& inImage, cv::Mat& outImage);
    void PMFishEyeCoreThread(size_t i, cv::Mat& inImage, cv::Mat& dst, int &doneInt, std::condition_variable_any &doneCV, std::mutex &mutex);
    std::mutex _mutexFisheye;
    std::mutex _mutexDrawImage;
    std::condition_variable_any _cvFisheye;
    bool _FIImageShown = false;


    std::vector<cv::Point> convertPoints(const std::vector<cv::Point2f>& points2f);

    cv::Point2f _dstPoints[4];
    cv::Mat _prevFrame;
    cv::Mat _transformationMatrix;

    ClockWidget preprocessClock{"Image Preprocess"};



};
