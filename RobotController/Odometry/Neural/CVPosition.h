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
    bool valid;
};

class CVPosition : public OdometryBase
{
public:
    CVPosition(ICameraReceiver *videoSource);
    ~CVPosition();

    std::vector<int> GetBoundingBox(int* outFrameID = nullptr);
    cv::Point2f GetCenter(int* outFrameID = nullptr);

    void SwitchRobots(void) override{}; // Dont do anything for SwitchRobots

private:
    void _ProcessNewFrame(cv::Mat frame, double frameTime) override; // Run every time a new frame is available
    cv::Point2f _lastPos = cv::Point2f(-1,-1);
    std::vector<std::string> classes{"orbitron"};

    cv::Size modelShape{};

    CVPositionData _lastData;
    cv::Point2f _lastVelocity;
    std::mutex _lastDataMutex;

    CVPositionData _GetDataFromPython();

    void _UpdateData(CVPositionData data, cv::Point2f velocity);

    // shared memory
    std::string memName = "cv_pos_img";
    int width = 720; // Example dimensions and type
    int height = 720;
    
    cv::Mat sharedImage;

    void _InitSharedImage();
    void _StartPython();
    std::thread _pythonThread;

    ServerSocket _pythonSocket;

    int _valid_frames_counter = 0;
    int _max_distance_thresh = 100;
};
