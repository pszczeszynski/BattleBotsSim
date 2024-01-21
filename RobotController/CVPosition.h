#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define MODEL_PATH "MachineLearning/rotation_model.onnx"

class CVRotation
{
public:
    CVRotation();
    double GetRobotRotation(cv::Mat& fieldImage, cv::Point2f robotPos);
    void _CropImage(cv::Mat& src, cv::Mat& dst, cv::Rect roi);
private:
    cv::dnn::Net _net;
};
