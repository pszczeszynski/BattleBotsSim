#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define MODEL_PATH "MachineLearning/rotation_model.onnx"

class CVRotation
{
public:
    CVRotation();
    static CVRotation& GetInstance();

    double ComputeRobotRotation(cv::Mat& fieldImage, cv::Point2f robotPos);
    double GetLastComputedRotation();
    void _CropImage(cv::Mat& src, cv::Mat& dst, cv::Rect roi);
    double GetLastConfidence();
private:
    double _lastRotation = 0;
    cv::dnn::Net _net;

    cv::Point2f _netXY1;
    cv::Point2f _netXY2;
    double _lastDisagreementRad;
};
