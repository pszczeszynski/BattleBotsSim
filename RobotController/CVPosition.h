#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>

#define MODEL_PATH "MachineLearning/position_model.onnx"

class CVPosition
{
public:
    CVPosition();
    static CVPosition& GetInstance();
    std::vector<int> GetBoundingBox();
private:
    cv::dnn::Net _net;
    cv::Point2f _lastPosition;

    std::vector<std::string> classes{"orbitron"};

    cv::Size2f modelShape{};

    float modelConfidenceThreshold {0.25};
    float modelScoreThreshold      {0.45};
    float modelNMSThreshold        {0.50};

    bool letterBoxForSquare = true;

    std::thread workerThread;

    std::vector<int> _boundingBox;
    std::mutex boundingBoxMutex;

    std::vector<int> ComputeRobotPosition(cv::Mat& fieldImage);
};