#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>

class CameraReceiver
{
public:
    CameraReceiver(std::string fileName, int width = 640, int height = 480);

    cv::Mat getFrame();
    ~CameraReceiver();

private:
    cv::Mat image;
    HANDLE hMapFile;
    LPVOID lpMapAddress;
};
