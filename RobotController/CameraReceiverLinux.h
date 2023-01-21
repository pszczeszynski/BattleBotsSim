#pragma once

#include <opencv2/core.hpp>
#include <string>

class CameraReceiver
{
public:
    CameraReceiver(std::string sharedFileName, int width, int height);
    cv::Mat getFrame();
    ~CameraReceiver();

private:
    cv::Mat image;
    void* lpMapAddress;
    int hMapFile;
};