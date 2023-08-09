#include "CameraReceiver.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <windows.h>
#include <iostream>
#include "Globals.h"
#include "Clock.h"
#include <stdlib.h>

////////////////////////////////////////// REAL VERSION //////////////////////////////////////////
#define BAD_READ_TIME_THRESH_SECONDS 0.4
#define NUMBER_LONG_READS_THRESH 2
int NUMBER_OF_LONG_READS = 0;

CameraReceiver::CameraReceiver(int cameraIndex) :
    _cameraIndex(cameraIndex)
{
    // Disable hardware transforms so it takes less time to initialize
    putenv("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0");

    // create a thread to capture frames
    _captureThread = std::thread([this, &cameraIndex]()
                                 {
        // try to initialize camera
        while (!_InitializeCamera())
        {
            std::cerr << "ERROR: failed to initialize camera!" << std::endl;
            Sleep(1000); // wait 1 second
        }

        // start capturing frames
        while (true)
        {
            _CaptureFrame();
        } });
}

void CameraReceiver::_CaptureFrame()
{
    // if camera disconnected, try to reconnect
    if (!_cap->isOpened())
    {
        std::cerr << "ERROR: camera disconnected!" << std::endl;
        _InitializeCamera();
        return;
    }

    // temporary frame
    cv::Mat frame = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
    Clock readTimer;
    readTimer.markStart();
    // wait for a frame
    bool result = _cap->read(frame);

    if (!result)
    {
        std::cerr << "ERROR: failed to read frame!" << std::endl;
        _InitializeCamera();
        return;
    }

    // if read took too long, warn
    if (readTimer.getElapsedTime() > BAD_READ_TIME_THRESH_SECONDS)
    {
        std::cout << "WARNING: read took too long!" << std::endl;
        NUMBER_OF_LONG_READS++;
    }
    else
    {
        // reset counter
        NUMBER_OF_LONG_READS = 0;
    }

    // check if too many long reads
    if (NUMBER_OF_LONG_READS >= NUMBER_LONG_READS_THRESH)
    {
        std::cout << "ERROR: too many long reads!" << std::endl;
        NUMBER_OF_LONG_READS = 0;
        _InitializeCamera();
        return;
    }

    // check if frame is empty
    if (frame.empty())
    {
        std::cout << "ERROR: frame is empty!" << std::endl;
        _InitializeCamera();
        return;
    }

    // check for black frame
    if (frame.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 0))
    {
        std::cout << "ERROR: black frame!" << std::endl;
        _InitializeCamera();
        return;
    }

    // convert to RGB
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    // lock the mutex
    _frameMutex.lock();
    // flip image
    cv::flip(frame, _frame, -1);
    // increase _framesReady
    _framesReady++;
    // unlock the mutex
    _frameMutex.unlock();
}

bool CameraReceiver::_InitializeCamera()
{
    std::cout << "Initializing Camera..." << std::endl;
    // Now we can create the video capture
    _cap = new cv::VideoCapture(_cameraIndex, cv::CAP_ANY);

    // check if camera opened successfully
    if (!_cap->isOpened())
    {
        std::cout << "ERROR: failed to open camera!" << std::endl;
        return false;
    }

    // set frame size
    _cap->set(cv::CAP_PROP_FRAME_WIDTH, WIDTH);
    _cap->set(cv::CAP_PROP_FRAME_HEIGHT, (int)(HEIGHT * 0.6666666667));
    // set to 60 fps
    _cap->set(cv::CAP_PROP_FPS, 100000);
    // disable buffering
    _cap->set(cv::CAP_PROP_BUFFERSIZE, 1);
    // disable auto exposure
    _cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    // disable auto white balance
    _cap->set(cv::CAP_PROP_AUTO_WB, 0.25);
    // set exposure
    _cap->set(cv::CAP_PROP_EXPOSURE, 0.1);
    // set white balance
    _cap->set(cv::CAP_PROP_WB_TEMPERATURE, 4500);

    // make camera have no automatic anything
    _cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    _cap->set(cv::CAP_PROP_AUTO_WB, 0.25);
    _cap->set(cv::CAP_PROP_AUTOFOCUS, 0.25);

    std::cout << "Successfully initialized camera!" << std::endl;

    return _cap->isOpened();
}

/**
 * Attempts to get a frame from the camera
 * @param output the output frame
 * @return true if frame was ready, false otherwise
 */
bool CameraReceiver::GetFrame(cv::Mat &output)
{
    TIMER_INIT
    TIMER_START

    _frameMutex.lock();
    // if no frames are ready, return false
    if (_framesReady <= 0)
    {
        _frameMutex.unlock();
        return false;
    }

    // otherwise copy the frame
    _frame.copyTo(output);
    _framesReady = 0;
    _frameMutex.unlock();

    TIMER_PRINT("CameraReceiver::getFrame")

    // return SUCCESS
    return true;
}

CameraReceiver::~CameraReceiver()
{
    _cap->release();
    delete _cap;
}

////////////////////////////////////////// SIMULATION //////////////////////////////////////////
CameraReceiverSim::CameraReceiverSim(std::string sharedFileName, int width, int height)
{
    std::wstring sharedFileNameW(sharedFileName.begin(), sharedFileName.end());
    LPCWSTR sharedFileNameLPCWSTR = sharedFileNameW.c_str();
    // 1. Create shared file
    // Open a handle to the memory-mapped file
    _hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, sharedFileNameLPCWSTR);

    if (_hMapFile == NULL)
    {
        std::cerr << "Could not open memory-mapped file" << std::endl;
        // get error
        DWORD err = GetLastError();
        if (err == 6) // ERROR_INVALID_HANDLE
        {
            std::cerr << "ERROR_INVALID_HANDLE" << std::endl;
        }
        std::cerr << "Error: " << err << std::endl;
        return;
    }

    // Map the memory-mapped file to a memory address
    _lpMapAddress = MapViewOfFile(_hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (_lpMapAddress == NULL)
    {
        std::cerr << "Could not map memory-mapped file" << std::endl;
        CloseHandle(_hMapFile);
        return;
    }

    // Create an OpenCV Mat to hold the image data (this points to the shared memory region)
    _image = cv::Mat(cv::Size(width, height), CV_8UC4, _lpMapAddress);
    std::cout << "done openning shared file: " << sharedFileName << std::endl;
}

static bool AreMatsEqual(const cv::Mat &mat1, const cv::Mat &mat2)
{
    if (mat1.size() != mat2.size() || mat1.type() != mat2.type())
    {
        // Mats have different sizes or types
        return false;
    }

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(mat1, mat2, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    return cv::countNonZero(grayDiff) == 0;
}

bool CameraReceiverSim::GetFrame(cv::Mat &output)
{
    // remove alpha
    cv::cvtColor(_image, output, cv::COLOR_BGRA2BGR);
    // Flip the image vertically
    cv::flip(output, output, 0);

    // Skip the first frame or if the current frame is the same as the previous frame
    if (AreMatsEqual(output, _prevFrame))
    {
        // set the previous frame to the current frame
        _prevFrame = output.clone();
        // return no classification
        return false;
    }

    // set the previous frame to the current frame
    _prevFrame = output.clone();
}

CameraReceiverSim::~CameraReceiverSim()
{
    // Unmap the memory-mapped file and close the handle
    UnmapViewOfFile(_lpMapAddress);
    CloseHandle(_hMapFile);
}
