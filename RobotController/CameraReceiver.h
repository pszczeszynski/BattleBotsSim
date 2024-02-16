#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>
#include <thread>
#include <condition_variable>
#include "Clock.h"
#include <conio.h>
#include <sstream>
#include <chrono>

// #define INCLUDE_SPINNAKER

#ifdef INCLUDE_SPINNAKER
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#endif


class ICameraReceiver
{
public:
    virtual long GetFrame(cv::Mat &output, long old_id);
protected:
    void _StartCaptureThread();

    ICameraReceiver();
    std::thread _captureThread;
    cv::Mat _frame; // the last frame captured
    std::mutex _frameMutex;
    std::condition_variable _frameCV;

    long int _frameID = 0;

    virtual bool _InitializeCamera() = 0;
    virtual bool _CaptureFrame() = 0;
};

class CameraReceiverSim : public ICameraReceiver
{
public:
    CameraReceiverSim(std::string fileName, int width = 1280, int height = 720);
    ~CameraReceiverSim();

private:
    bool _InitializeCamera() override;
    bool _CaptureFrame() override;

    std::string _sharedFileName;
    int _width;
    int _height;

    cv::Mat _image; // the memory mapped image
    HANDLE _hMapFile;
    HANDLE _hMutex;
    LPVOID _lpMapAddress;

    std::mutex _frameMutex;
    cv::Mat _prevFrame;

    Clock _prevFrameTimer;
};

#ifdef INCLUDE_SPINNAKER
class CameraReceiver : public ICameraReceiver
{
public:
    CameraReceiver(int cameraIndex);
    long GetFrame(cv::Mat &output, old_id);
    ~CameraReceiver();

private:
    virtual bool _InitializeCamera() override;
    virtual bool _CaptureFrame() override;

    int _cameraIndex;
    int pcam_image_width;
    int pcam_image_height;

    Spinnaker::CameraPtr pCam = nullptr;
    Spinnaker::SystemPtr _system = nullptr;
};

#endif

class CameraReceiverVideo : public ICameraReceiver
{
public:
    CameraReceiverVideo(std::string fileName);
private:
    cv::VideoCapture _cap;
    virtual bool _InitializeCamera() override;
    virtual bool _CaptureFrame() override;
    std::string _fileName;
    long _videoFrame = 0;

    Clock _prevFrameTimer;

};