#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "Odometry/OdometryBase.h"
#include "CameraReceiver.h"


// MachineLearning/rotation_model_hoop.onnx
#define MODEL_PATH "MachineLearning/rotation_model_hoop.onnx"

class CVRotation : public OdometryBase
{
public:
    CVRotation(ICameraReceiver *videoSource);
    void _ProcessNewFrame(cv::Mat frame, double frameTime) override; // Run every time a new frame is available

    static CVRotation* GetInstance();

    double ComputeRobotRotation(cv::Mat& fieldImage, cv::Point2f robotPos);
    double GetLastComputedRotation();
    bool _CropImage(cv::Mat& src, cv::Mat& dst, cv::Rect roi);
    double GetLastConfidence();
private:
    static CVRotation* _instance;

    double _lastRotation = 0;
    unsigned int _frameID = 0;
    cv::dnn::Net _net;

    cv::Point2f _netXY1;
    cv::Point2f _netXY2;
    double _lastDisagreementRad;
};
