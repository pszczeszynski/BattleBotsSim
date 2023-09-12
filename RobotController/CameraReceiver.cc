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
#include "Input/Input.h"


#define VIDEO_READ
// #define SAVE_VIDEO

////////////////////////////////////////// REAL VERSION //////////////////////////////////////////
#define BAD_READ_TIME_THRESH_SECONDS 0.4
#define NUMBER_LONG_READS_THRESH 2
int NUMBER_OF_LONG_READS = 0;


bool saveVideo = false;
CameraReceiver::CameraReceiver(int cameraIndex) : _cameraIndex(cameraIndex)
{
    // Disable hardware transforms so it takes less time to initialize
    putenv("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0");

    // create a thread to capture frames
    _captureThread = std::thread([this]()
                                 {

#ifdef SAVE_VIDEO
        // Initialize VideoWriter0
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // or use another codec
        double fps = 60.0;                                        // you can adjust this according to your needs
        cv::Size frameSize(1280, 720);
        cv::VideoWriter video("Recordings/outputVideo.avi", fourcc, fps, frameSize, true); // 'true' for color video
#endif

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

#ifdef SAVE_VIDEO
            if (Input::GetInstance().IsKeyPressed(Qt::Key_Enter))
            {
                saveVideo = true;
            }
            if (Input::GetInstance().IsKeyPressed(Qt::Key_Backspace))
            {
                saveVideo = false;
            }
            if (saveVideo)
            {
                _frameMutex.lock();
                video.write(_frame);
                _frameMutex.unlock();
            }
#endif

#ifdef SIMULATION
            // sleep for 15 ms
            Sleep(15);
#endif
            
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

    // cv::equalizeHist(frame, frame);
    // convert to RGB
    // lock the mutex
    _frameMutex.lock();
    cv::cvtColor(frame, _frame, cv::COLOR_BGR2RGB);

    // flip image
    // cv::flip(frame, _frame, -1);
    // increase _framesReady
    _framesReady++;
    // unlock the mutex
    _frameMutex.unlock();
}

bool CameraReceiver::_InitializeCamera()
{
    std::cout << "Initializing Camera..." << std::endl;

    // check if camera is already initialized
    if (_cap != nullptr)
    {
        // release the camera
        _cap->release();
        // delete the camera
        delete _cap;
        _cap = nullptr;
    }


#ifndef VIDEO_READ
    // Now we can create the video capture
    _cap = new cv::VideoCapture(_cameraIndex, cv::CAP_DSHOW);

    // Set core properties first
    _cap->set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    _cap->set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    _cap->set(cv::CAP_PROP_FPS, 60); // Assuming you meant 60 fps based on your comment. Adjust this value to the highest supported fps if you have a specific requirement.

    _cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // Disable buffering to minimize latency
    _cap->set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Disable any automatic settings for more predictable performance
    _cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0); // 0.25 usually means "manual mode" in OpenCV
    _cap->set(cv::CAP_PROP_AUTO_WB, 1.0);
#else
    // init video reader
    _cap = new cv::VideoCapture("Recordings/terribleFight.avi");
    _cap->set(cv::CAP_PROP_FPS, 60); // Assuming you meant 60 fps based on your comment. Adjust this value to the highest supported fps if you have a specific requirement.

#endif

    // check if camera opened successfully
    if (!_cap->isOpened())
    {
        std::cout << "ERROR: failed to open camera!" << std::endl;
        return false;
    }

    // _cap->set(cv::CAP_PROP_AUTOFOCUS, 0.25);

    // // Set exposure and white balance after disabling automatic modes
    // _cap->set(cv::CAP_PROP_EXPOSURE, 0.1);
    // _cap->set(cv::CAP_PROP_WB_TEMPERATURE, 2500);

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
CameraReceiverSim::CameraReceiverSim(std::string sharedFileName, int width, int height) : _sharedFileName(sharedFileName),
                                                                                          _width(width),
                                                                                          _height(height)
{
    // create a thread to capture frames
    _captureThread = std::thread([this]()
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

bool CameraReceiverSim::_InitializeCamera()
{
    std::wstring sharedFileNameW(_sharedFileName.begin(), _sharedFileName.end());
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
        return false;
    }

    // Map the memory-mapped file to a memory address
    _lpMapAddress = MapViewOfFile(_hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (_lpMapAddress == NULL)
    {
        std::cerr << "Could not map memory-mapped file" << std::endl;
        CloseHandle(_hMapFile);
        return false;
    }

    // Create an OpenCV Mat to hold the image data (this points to the shared memory region)
    _image = cv::Mat(cv::Size(_width, _height), CV_8UC4, _lpMapAddress);
    std::cout << "done openning shared file: " << _sharedFileName << std::endl;

    // return success
    return true;
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

void CameraReceiverSim::_CaptureFrame()
{
    // don't receive at more than 60 fps
    if (_prevFrameTimer.getElapsedTime() < 0.015)
    {
        return;
    }
    _prevFrameTimer.markStart();

    cv::Mat captured;
    // remove alpha
    cv::cvtColor(_image, captured, cv::COLOR_BGRA2BGR);
    // Flip the image vertically
    cv::flip(captured, captured, 0);

    // lock the mutex
    _frameMutex.lock();
    // copy the frame to the previous frame
    _frame.copyTo(_prevFrame);
    // copy the frame
    captured.copyTo(_frame);
    // increase _framesReady
    _framesReady++;
    // unlock the mutex
    _frameMutex.unlock();
}

bool CameraReceiverSim::GetFrame(cv::Mat &output)
{
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

    // return SUCCESS
    return true;
}

CameraReceiverSim::~CameraReceiverSim()
{
    // Unmap the memory-mapped file and close the handle
    UnmapViewOfFile(_lpMapAddress);
    CloseHandle(_hMapFile);
}
