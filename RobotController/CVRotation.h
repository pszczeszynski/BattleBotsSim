#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define MODEL_PATH "MachineLearning/model.onnx"

class CVRotation
{
public:
    CVRotation();
    double GetRobotRotation(cv::Mat& fieldImage, cv::Point2f robotPos);
private:
    cv::dnn::Net _net;
};
