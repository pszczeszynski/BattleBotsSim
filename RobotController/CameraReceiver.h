#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>

class ICameraReceiver
{
public:
    virtual void getFrame(cv::Mat &output) = 0;
};

class CameraReceiverSim : public ICameraReceiver {
public:
    CameraReceiverSim(std::string fileName, int width = 1280, int height = 720);
    void getFrame(cv::Mat& output);
    ~CameraReceiverSim();

private:
    cv::Mat image;
    HANDLE hMapFile;
    HANDLE hMutex;
    LPVOID lpMapAddress;
};

class CameraReceiver : public ICameraReceiver
{
public:
    CameraReceiver(int cameraIndex);
    void getFrame(cv::Mat& output);
    ~CameraReceiver();
private:
    cv::VideoCapture* cap = nullptr;
};
