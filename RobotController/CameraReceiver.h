#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>

class CameraReceiver
{
public:
    CameraReceiver(std::string fileName, int width = 1280, int height = 960);

    cv::Mat getFrame();
    ~CameraReceiver();

private:
    cv::Mat image;
    HANDLE hMapFile;
    HANDLE hMutex;
    LPVOID lpMapAddress;
};
