#include "CameraReceiverLinux.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

CameraReceiver::CameraReceiver(std::string sharedFileName, int width, int height)
{
    // Open a file descriptor to the shared memory file
    int fd = shm_open(sharedFileName.c_str(), O_RDONLY, 0);
    if (fd < 0)
    {
        std::cerr << "Could not open shared memory file" << std::endl;
        return;
    }

    // Map the shared memory file to a memory address
    lpMapAddress = mmap(NULL, width * height * 4, PROT_READ, MAP_SHARED, fd, 0);
    if (lpMapAddress == MAP_FAILED)
    {
        std::cerr << "Could not map shared memory file" << std::endl;
        close(fd);
        return;
    }

    // Create an OpenCV Mat to hold the image data (this points to the shared memory region)
    image = cv::Mat(cv::Size(width, height), CV_8UC4, lpMapAddress);
}

cv::Mat CameraReceiver::getFrame()
{
    // Create a new matrix to store the output image
    cv::Mat output = image.clone();
    // Flip the image vertically
    cv::flip(output, output, 0);
    cv::cvtColor(output, output, cv::COLOR_BGR2RGB);

    return output;
}

CameraReceiver::~CameraReceiver()
{
// Unmap the memory-mapped file and close the handle
UnmapViewOfFile(lpMapAddress);
CloseHandle(hMapFile);
}

