#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>
#include "../../ServerSocket.h"
#include "../Odometrybase.h"


struct CVPositionData
{
    std::vector<int> boundingBox;
    cv::Point2f center;
    uint32_t frameID;
    uint32_t time_millis;
};

class CVPosition : public OdometryBase
{
public:
    CVPosition();

    std::vector<int> GetBoundingBox(int* outFrameID = nullptr);
    cv::Point2f GetCenter(int* outFrameID = nullptr);

    void SwitchRobots(void) override{}; // Dont do anything for SwitchRobots
    bool Run(void) override; // Starts the thread(s) to decode data. Returns true if succesful

private:
    cv::Point2f _lastPosition;
    std::vector<std::string> classes{"orbitron"};

    cv::Size modelShape{};

    std::thread workerThread;

    CVPositionData _lastData;
    std::mutex _lastDataMutex;

    CVPositionData _GetDataFromPython();

    void _UpdateData(CVPositionData data, cv::Point2f velocity);


    // shared memory
    std::string memName = "cv_pos_img";
    int width = 720; // Example dimensions and type
    int height = 720;
    
    cv::Mat sharedImage;

    void _InitSharedImage();

    ServerSocket _pythonSocket;

};
