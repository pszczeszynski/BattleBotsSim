#include "CameraReceiver.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>
#include <iostream>

CameraReceiver::CameraReceiver(std::string sharedFileName, int width, int height)
{
    // 1. Create shared file
    // Open a handle to the memory-mapped file
    hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, sharedFileName.c_str());

    if (hMapFile == NULL)
    {
        std::cerr << "Could not open memory-mapped file" << std::endl;
        return;
    }

    // Map the memory-mapped file to a memory address
    lpMapAddress = MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (lpMapAddress == NULL)
    {
        std::cerr << "Could not map memory-mapped file" << std::endl;
        CloseHandle(hMapFile);
        return;
    }

    // Create an OpenCV Mat to hold the image data (this points to the shared memory region)
    image = cv::Mat(cv::Size(width, height), CV_8UC4, lpMapAddress);
}

void CameraReceiver::getFrame(cv::Mat& output)
{
    cv::cvtColor(image, output, cv::COLOR_RGBA2BGR);
    // Flip the image vertically
    cv::flip(output, output, 0);
}

CameraReceiver::~CameraReceiver()
{
    // Unmap the memory-mapped file and close the handle
    UnmapViewOfFile(lpMapAddress);
    CloseHandle(hMapFile);
}
