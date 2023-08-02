#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>
#include <thread>

class ICameraReceiver
{
public:
    virtual bool GetFrame(cv::Mat &output) = 0;
};

class CameraReceiverSim : public ICameraReceiver
{
public:
    CameraReceiverSim(std::string fileName, int width = 1280, int height = 720);
    bool GetFrame(cv::Mat &output);
    ~CameraReceiverSim();

private:
    cv::Mat image;
    HANDLE hMapFile;
    HANDLE hMutex;
    LPVOID lpMapAddress;

    cv::Mat _prevFrame;
};

class CameraReceiver : public ICameraReceiver
{
public:
    CameraReceiver(int cameraIndex);
    bool GetFrame(cv::Mat &output);
    ~CameraReceiver();

private:
    std::thread _captureThread;
    std::mutex _frameMutex;
    cv::Mat _frame;
    long int _framesReady;

    cv::VideoCapture *_cap;
};
