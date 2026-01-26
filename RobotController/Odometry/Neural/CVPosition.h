#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <windows.h>
#include "../../ServerSocket.h"
#include "../OdometryBase.h"

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
    std::vector<std::string> classes{"orbitron"};

    cv::Size modelShape{};

    // Latest data from Python (protected by _lastDataMutex)
    CVPositionData _lastData;
    std::mutex _lastDataMutex;

    // State for velocity computation and gating
    cv::Point2f _lastValidCenter;
    double _lastValidTime = 0.0;
    uint32_t _lastValidFrameID = 0;
    int _validStreakCounter = 0;  // Consecutive valid frames
    int _max_distance_thresh = 100;
    bool _hasPrevValidState = false;  // Whether we have previous valid state for velocity

    CVPositionData _GetDataFromPython(bool& outPythonResponded);

    // Shared memory management
    std::string memName = "cv_pos_img";
    int width = 720;
    int height = 720;
    HANDLE _hMapFile = nullptr;  // Shared memory mapping handle
    void* _pSharedMemory = nullptr;  // Mapped memory pointer
    cv::Mat sharedImage;

    void _InitSharedImage();
    void _StartPython();
    std::thread _pythonThread;
    std::atomic<bool> _pythonThreadRunning{false};

    ServerSocket _pythonSocket;

    // Helper to compute center from bounding box
    // Assumes bbox format is [x1, y1, x2, y2] and computes midpoint
    cv::Point2f _ComputeCenterFromBBox(const std::vector<int>& bbox);
    
    // Helper to compute velocity from successive positions
    cv::Point2f _ComputeVelocity(cv::Point2f currentCenter, double currentTime,
                                  cv::Point2f lastCenter, double lastTime);
};
