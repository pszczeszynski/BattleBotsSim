#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>
#include "ServerSocket.h"

#define MODEL_PATH "MachineLearning/train_yolo/model2_orbitron_unpreprocessed.onnx"

class CVPosition
{
public:
    CVPosition();
    static CVPosition& GetInstance();

    std::vector<int> GetBoundingBox(int* outFrameID = nullptr);
    cv::Point2f GetCenter(int* outFrameID = nullptr);

private:
    cv::dnn::Net _net;
    cv::Point2f _lastPosition;

    std::vector<std::string> classes{"orbitron"};

    cv::Size modelShape{};

    std::thread workerThread;

    std::vector<int> _boundingBox;
    std::mutex _boundingBoxMutex;
    std::mutex _frameIdMutex;

    std::vector<int> _ComputeRobotPosition();



    // shared memory
    std::string memName = "cv_pos_img";
    int width = 720; // Example dimensions and type
    int height = 720;
    
    cv::Mat sharedImage;

    void _InitSharedImage();

    ServerSocket _pythonSocket;

    int _frameId;
};
