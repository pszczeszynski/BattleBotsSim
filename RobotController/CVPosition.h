#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define MODEL_PATH "MachineLearning/position_model.onnx"

class CVPosition
{
public:
    CVPosition();
    static CVPosition& GetInstance();

    cv::Point2f ComputeRobotPosition(cv::Mat& fieldImage);
private:
    cv::dnn::Net _net;
    cv::Point2f _lastPosition;
};
