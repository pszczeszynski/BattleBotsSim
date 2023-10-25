#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <windows.h>
#include <thread>
#include "Clock.h"

class ICameraReceiver
{
public:
    virtual bool GetFrame(cv::Mat &output) = 0;
};

// class CameraReceiverSim : public ICameraReceiver
// {
// public:
//     CameraReceiverSim(std::string fileName, int width = 1280, int height = 720);
//     bool GetFrame(cv::Mat &output);
//     ~CameraReceiverSim();

// private:
//     bool _InitializeCamera();
//     void _CaptureFrame();


//     std::string _sharedFileName;
//     int _width;
//     int _height;

//     cv::Mat _image; // the memory mapped image
//     HANDLE _hMapFile;
//     HANDLE _hMutex;
//     LPVOID _lpMapAddress;
//     std::thread _captureThread;

//     std::mutex _frameMutex;
//     cv::Mat _frame; // the last frame captured
//     cv::Mat _prevFrame;

//     Clock _prevFrameTimer;

//     long int _framesReady;
// };

class CameraReceiver : public ICameraReceiver
{
public:
    CameraReceiver(int cameraIndex);
    bool GetFrame(cv::Mat &output);
    ~CameraReceiver();

private:
    bool _InitializeCamera();
    void _CaptureFrame();

    std::thread _captureThread;
    std::mutex _frameMutex;
    cv::Mat _frame;
    long int _framesReady;
    int _cameraIndex;

    cv::VideoCapture *_cap = nullptr;
};
