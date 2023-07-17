#include "CameraReceiver.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>
#include <iostream>
#include "Globals.h"
#include "Clock.h"

////////////////////////////////////////// REAL VERSION //////////////////////////////////////////


CameraReceiver::CameraReceiver(int cameraIndex)
{
    // Create a VideoCapture object with the DirectShow string
    cap = new cv::VideoCapture(cameraIndex, cv::CAP_DSHOW);

    // enable 60 fps
    cap->set(cv::CAP_PROP_FPS, 60);
    // disable auto exposure
    cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    // disable auto white balance
    cap->set(cv::CAP_PROP_AUTO_WB, 0.25);
    // set exposure
    cap->set(cv::CAP_PROP_EXPOSURE, 0.1);
    // set white balance
    cap->set(cv::CAP_PROP_WB_TEMPERATURE, 4500);

    // set resolution
    cap->set(cv::CAP_PROP_FRAME_WIDTH, WIDTH * 1.7777778);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT);
    

    if (!cap->isOpened())
    {
        std::cout << "Failed to open VideoCapture " << cameraIndex << std::endl;
    }
}

void CameraReceiver::getFrame(cv::Mat& output)
{
TIMER_INIT
TIMER_START
    *cap >> output;
TIMER_PRINT("CameraReceiver::getFrame")

    // print size
    std::cout << "CameraReceiver::getFrame: " << output.size() << std::endl;
}

CameraReceiver::~CameraReceiver()
{
    delete cap;
}


////////////////////////////////////////// SIMULATION //////////////////////////////////////////
CameraReceiverSim::CameraReceiverSim(std::string sharedFileName, int width, int height)
{
    LPCWSTR sharedFileNameLPCWSTR = std::wstring(sharedFileName.begin(), sharedFileName.end()).c_str();
    // 1. Create shared file
    // Open a handle to the memory-mapped file
    hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, sharedFileNameLPCWSTR);

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

void CameraReceiverSim::getFrame(cv::Mat& output)
{
    // remove alpha
    cv::cvtColor(image, output, cv::COLOR_BGRA2BGR);
    // Flip the image vertically
    cv::flip(output, output, 0);
}

CameraReceiverSim::~CameraReceiverSim()
{
    // Unmap the memory-mapped file and close the handle
    UnmapViewOfFile(lpMapAddress);
    CloseHandle(hMapFile);
}
