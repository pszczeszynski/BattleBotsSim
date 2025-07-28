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
#include "VisionPreprocessor.h"

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"


enum class CameraType
{
    BASE_CLASS = 0,
    SIM_CAMERA,
    VIDEO_CAMERA,
    REAL_CAMERA,
    USB_CAMERA
};

class ICameraReceiver
{
public:
    virtual long GetFrame(cv::Mat &output, long old_id, double* frameTime = NULL, bool blockUntilReady = true);
    virtual bool NewFrameReady(long old_id);
    static ICameraReceiver* GetInstance();
    virtual CameraType GetType();

    // destructor
    virtual ~ICameraReceiver();


protected:
    void _StartCaptureThread();

    ICameraReceiver();
    std::thread _captureThread;
    cv::Mat _frame; // the last frame captured
    std::mutex _frameMutex;
    std::condition_variable _frameCV;

    long int _frameID = 0;
    double _frameTime = 0;

    virtual bool _InitializeCamera() = 0;
    virtual bool _CaptureFrame() = 0;

    VisionPreprocessor birdsEyePreprocessor;
};

class CameraReceiverSim : public ICameraReceiver
{
public:
    CameraReceiverSim(std::string fileName);
    ~CameraReceiverSim();
    CameraType GetType() override;

private:
    bool _InitializeCamera() override;
    bool _CaptureFrame() override;
    bool UpdateImageFromSharedMemory();

    std::string _sharedFileName;
    int _width;
    int _height;
    int _lastFrameNumber; // Track the last frame number

    cv::Mat _image; // the memory mapped image
    HANDLE _hMapFile;
    HANDLE _hMutex;
    LPVOID _lpMapAddress;

    std::mutex _frameMutex;
    cv::Mat _prevFrame;

    Clock _prevFrameTimer;
};

class CameraReceiver : public ICameraReceiver
{
public:
    CameraReceiver();
    ~CameraReceiver();
    void SetCameraGain(float gain);
    CameraType GetType() override;

private:
    virtual bool _InitializeCamera() override;
    virtual bool _CaptureFrame() override;

    int pcam_image_width;
    int pcam_image_height;

    Spinnaker::CameraPtr pCam = nullptr;
    Spinnaker::SystemPtr _system = nullptr;
};

class CameraReceiverUSB : public ICameraReceiver
{
public:
    CameraReceiverUSB();
    ~CameraReceiverUSB();
    CameraType GetType() override;

private:
    virtual bool _InitializeCamera() override;
    virtual bool _CaptureFrame() override;

    cv::VideoCapture _cap;
};

class CameraReceiverVideo : public ICameraReceiver
{
public:
    CameraReceiverVideo();
    CameraType GetType() override;
private:
    cv::VideoCapture _cap;
    virtual bool _InitializeCamera() override;
    virtual bool _CaptureFrame() override;
    long _videoFrame = 0;
    long _maxFrameCount = 1;

    Clock _prevFrameTimer;
};