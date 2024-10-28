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
#include "imgui.h"
#include "Input/InputState.h"
#include "VisionPreprocessor.h"
#include "UIWidgets/FieldWidget.h"
#include "UIWidgets/TrackingWidget.h"
#include "UIWidgets/ClockWidget.h"

#define GET_FRAME_TIMEOUT_MS 500
#define GET_FRAME_MUTEX_TIMEOUT std::chrono::milliseconds(150)

double MAX_CAP_FPS = 100.0;

// TODO: add a way to save video with the UI
ICameraReceiver* _instance = nullptr;

ICameraReceiver& ICameraReceiver::GetInstance()
{
    if (!_instance)
    {
        std::cerr << "ERROR: CameraReceiver not initialized!" << std::endl;
        exit(1);
    }
    return *_instance;
}

ICameraReceiver::ICameraReceiver()
{
    // Disable hardware transforms so it takes less time to initialize
    putenv("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0");
    _instance = this;
}

void ICameraReceiver::_StartCaptureThread()
{
    // create a thread to capture frames
    _captureThread = std::thread([this]()
                                 {
        ClockWidget captureTimer{"Camera Grab"};

        // try to initialize camera
        while (!_InitializeCamera())
        {
            std::cerr << "ERROR: failed to initialize camera!" << std::endl;
            Sleep(10000); // wait 1 second
        }

        std::cout << "Camera initialized" << std::endl;

        // start capturing frames
        while (true)
        {
            captureTimer.markEnd();
            captureTimer.markStart();

            _CaptureFrame();  
        } });
}

/**
 * Attempts to get a frame from the camera
 * @param output the output frame
 * @param old_id the previous id of the frame. If a new frame is not ready it will block
 * If old_id is 0, it will return the latest frame
 * @param frameTime (Optional) pointer to a double that will store the frame elapsed time when it was acquired
 * @return the id of the new frame
 */
long ICameraReceiver::GetFrame(cv::Mat &output, long old_id, double* frameTime, bool blockUntilReady)
{
    // Using scoped mutex locker because conditional variable integrates with it
    // This creates and locks the mutex
    std::unique_lock<std::mutex> locker(_frameMutex);

    while ((_frameID <= 0) || (_frameID <= old_id))
    {
        if( blockUntilReady )
        {
            _frameCV.wait(locker);
        }
        else       // Unlock mutex, waits until conditional varables is notifed, then it locks mutex again
        if( _frameCV.wait_for(locker, GET_FRAME_MUTEX_TIMEOUT) ==  std::cv_status::timeout )
        {
            // Unable to get a new frame, exit
            return -1;
        }
    }

    // At this point our mutex is locked and a frame is ready
    _frame.copyTo(output);
    old_id = _frameID;

    if (frameTime != NULL)
    {
        *frameTime = _frameTime;
    }

    locker.unlock();
    // return ID of new frame
    return old_id;
}

// Returns true if a newer frame is ready
bool ICameraReceiver::NewFrameReady(long old_id)
{
    return _frameID > old_id;
}

 CameraType ICameraReceiver::GetType()
 {
    return CameraType::BASE_CLASS;
 }

////////////////////////////////////////// REAL VERSION //////////////////////////////////////////

CameraType CameraReceiver::GetType() 
 {
    return CameraType::REAL_CAMERA;
 }

CameraReceiver::CameraReceiver() : ICameraReceiver()
{
    _StartCaptureThread();
}

int ConfigureCamera(Spinnaker::CameraPtr pCam)
{

    std::cout << "Configuring camera" << std::endl;
    //
    // NOTE: In general, a fault will occur if you disable a function and try to set its options
    //
    try
    {

        std::cout << "Initing camera" << std::endl;
        pCam->Init();

        std::cout << "Setting up camera" << std::endl;

        // Disable heartbeat (a GiGe standard requiring program to reconfirm periodically its alive)
        pCam->GevGVCPHeartbeatDisable();

        // ******** General Settings ***************
        // Set exposure mode
        // pCam->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);  // turn off auto
        pCam->ExposureMode.SetValue(Spinnaker::ExposureMode_Timed); // set it to fixed time
        // pCam->ExposureTime.SetValue(10002.0); // Set to 2ms. Longer is less noisy

        // Set Gain Mode
        pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Once); // Turn off auto gain
        // pCam->Gain.SetValue(8.0); // Set gain in dB (between 0 and 47.99)

        // Turn off Gama correction
        pCam->GammaEnable.SetValue(false);
        // pCam->Gamma.SetValue(1.0f, false); // Gamma correction if enabled. 0 to 4 nominal range

        // Turn off white balance
        // pCam->BalanceWhiteAuto.SetValue(Spinnaker::BalanceWhiteAuto_Off);

        // Allocate all bandwidth to this one camera 125000000 is max, reduce to 122000000 for margin
        pCam->DeviceLinkThroughputLimit.SetValue(122000000);

        // Turn off trigger
        pCam->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);

        // Turn off sequencer
        // pCam->SequencerConfigurationMode.SetValue(Spinnaker::SequencerConfigurationMode_Off);
        // pCam->SequencerFeatureEnable.SetValue(false);
        // pCam->SequencerMode.SetValue( SequencerModeEnums::SequencerMode_Off);

        // ******** Image Settings ***************
        // Reducing image size saves bandwidth for faster refresh rate
        // We need to reset offsets first before changing this
        // pCam->OffsetX.SetValue(0);
        // pCam->OffsetY.SetValue(0);

        // Now setting width/height should cause issues
        int my_width = 1440;
        int my_height = 600;

        // try
        // {
        //     pCam->Width.SetValue(my_width);   // Max 1440 for this camera
        //     pCam->Height.SetValue(my_height); // Max 1080 for this camera
        // }
        // catch (Spinnaker::Exception &e)
        // {
        //     std::cout << "Error: " << e.what() << std::endl;
        //     return -1;
        // }

        // pCam->OffsetX.SetValue((1440 - my_width) / 2);  // Set to center of ccd
        // pCam->OffsetY.SetValue((1080 - my_height) / 2); // Set to center of ccd

        // pCam->IspEnable.SetValue(false);                         // Turn off image processing
        // pCam->AdcBitDepth.SetValue(Spinnaker::AdcBitDepth_Bit8); // Set to 8-bit color resolution

        // ******* Output Data Settings *******

        // pCam->PixelFormat.SetValue(Spinnaker::PixelFormat_Mono8); // Only some of the formats work, this is one of them and is fast.

        // Compression may be useful, but not tested for delay
        // pCam->ImageCompressionMode.SetValue(Spinnaker::ImageCompressionModeEnums::ImageCompressionMode_Off);

        // Set Acquisition to continouse
        // pCam->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

        //
        // Make it always return latest image (This option is not available via the easy access mode)

        Spinnaker::GenApi::INodeMap &sNodeMap = pCam->GetTLStreamNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        ptrHandlingMode->SetIntValue(Spinnaker::StreamBufferHandlingMode_NewestOnly);

        // Save settings to user register 0
        // pCam->UserSetSelector = UserSetSelectorEnums::UserSetSelector_UserSet0;
        // pCam->UserSetSave();
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "********* CONFIG ERROR *********" << std::endl;
        std::cout << "Error: " << e.what() << std::endl
                  << std::endl;
        return -1;
    }

    return 0;
}


bool CameraReceiver::_InitializeCamera()
{
    // Retrieve singleton reference to system object
    _system = Spinnaker::System::GetInstance();

    // Retrieve list of cameras from the system
    Spinnaker::CameraList camList = _system->GetCameras();

    const unsigned int numCameras = camList.GetSize();
    std::cerr << "Number of cameras detected: " << numCameras << std::endl
              << std::endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        _system->ReleaseInstance();

        return false;
    }

    // Get the first camera
    pCam = camList.GetByIndex(0);

    int configRet = -1;
    try
    {
        configRet = ConfigureCamera(pCam);
    }
    // any exception
    catch (Spinnaker::Exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        configRet = -1;
    }

    // Initialize
    if (configRet < 0)
    {
        std::cerr << "Failed to configure camera" << std::endl;

        // Failed, exit
        pCam = nullptr;

        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        _system->ReleaseInstance();
        // return failure
        return false;
    }

    try
    {
        // Start acquisition
        pCam->BeginAcquisition();

        pcam_image_width = (int)pCam->Width.GetValue();   // Max 1440 for this camera
        pcam_image_height = (int)pCam->Height.GetValue(); // Max 1080 for this camera
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        // Failed, exit
        pCam = nullptr;

        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        _system->ReleaseInstance();
        // return failure
        return false;
    }

    // return success
    return true;
}

void CameraReceiver::SetCameraGain(float gain)
{
    if (pCam == nullptr)
    {
        return;
    }
    pCam->Gain.SetValue(gain); // Set gain in dB (between 0 and 47.99)
}


bool CameraReceiver::_CaptureFrame()
{
    if (pCam == nullptr)
    {
        std::cerr << "ERROR: camera not initialized!" << std::endl;
        return false;
    }
    // Get Next Image
    Spinnaker::ImagePtr pResultImage = pCam->GetNextImage(GET_FRAME_TIMEOUT_MS);

    if (!pResultImage || pResultImage->IsIncomplete())
    {
        std::cerr << "ERROR: Failed to capture frame!" << std::endl;
        return false;
    }

    unsigned char *img_data = (unsigned char *)pResultImage->GetData();

    // Create a Mat with the data
    cv::Mat bayerImage(pcam_image_height, pcam_image_width, CV_8UC1, img_data);

    // Apply processing to it
    cv::Mat finalImage;
    birdsEyePreprocessor.Preprocess(bayerImage, finalImage);

    TrackingWidget* trackingWidget = TrackingWidget::GetInstance();
    if (trackingWidget != nullptr)
    {
        // apply mask
        cv::Mat& mask = trackingWidget->GetMask();
        finalImage.setTo(cv::Scalar(0, 0, 0), mask);
    }

    // lock the mutex
    std::unique_lock<std::mutex> locker(_frameMutex);

    // deep copy over the frame
    finalImage.copyTo(_frame);

    // increase _frameID
    _frameID++;

    // Update time
    _frameTime = Clock::programClock.getElapsedTime();

    // unlock the mutex
    locker.unlock();

    // Notify new frame is ready
    _frameCV.notify_all();

    // return success
    return true;
}

CameraReceiver::~CameraReceiver()
{
}


///////////////////////////////////////// SIMULATION //////////////////////////////////////////
CameraReceiverSim::CameraReceiverSim(std::string sharedFileName, int width, int height)
    : ICameraReceiver(), _sharedFileName(sharedFileName), _width(width), _height(height)
{
    _StartCaptureThread();
}

CameraType CameraReceiverSim::GetType() 
 {
    return CameraType::SIM_CAMERA;
 }


bool CameraReceiverSim::_InitializeCamera()
{
    std::wstring sharedFileNameW(_sharedFileName.begin(), _sharedFileName.end());
    // LPCWSTR sharedFileNameLPCWSTR = sharedFileNameW.c_str();

    // 1. Create shared file
    // Open a handle to the memory-mapped file
    _hMapFile = OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, sharedFileNameW.c_str());

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

bool CameraReceiverSim::_CaptureFrame()
{
    // wait for the previous frame to be at 1/MAX_CAP_FPS long
    while (_prevFrameTimer.getElapsedTime() < 1.0 / MAX_CAP_FPS )
    {
        double currelapsedtime = _prevFrameTimer.getElapsedTime();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    _prevFrameTimer.markStart(_prevFrameTimer.getElapsedTime() - 1.0 / MAX_CAP_FPS);

    cv::Mat captured;

    // remove alpha and convert to B&W
    if (_image.channels() == 1) {
        captured = _image;
    }
    else if (_image.channels() == 3) {
        cv::cvtColor(_image, captured, cv::COLOR_BGR2GRAY);
    }
    else if (_image.channels() == 4)
    {
        cv::cvtColor(_image, captured, cv::COLOR_BGRA2GRAY);
    }

    // Flip the image vertically
    cv::flip(captured, captured, 0);

    // Apply processing to it
    cv::Mat finalImage;
    birdsEyePreprocessor.Preprocess(captured, finalImage);

    TrackingWidget* trackingWidget = TrackingWidget::GetInstance();
    if (trackingWidget != nullptr)
    {
        // apply mask
        cv::Mat& mask = trackingWidget->GetMask();
        finalImage.setTo(cv::Scalar(0, 0, 0), mask);
    }

    // lock the mutex
    std::unique_lock<std::mutex> locker(_frameMutex);

    // copy the frame to the previous frame
    _frame.copyTo(_prevFrame);
    // copy the frame
    finalImage.copyTo(_frame);
    // increase _frameID
    _frameID++;

    // Update time
    _frameTime = Clock::programClock.getElapsedTime();

    // unlock the mutex
    locker.unlock();

    // Notify all frame is ready
    _frameCV.notify_all();

    // return success
    return true;
}

CameraReceiverSim::~CameraReceiverSim()
{
    // Unmap the memory-mapped file and close the handle
    UnmapViewOfFile(_lpMapAddress);
    CloseHandle(_hMapFile);
}


////////////////////////////////////////// VIDEO PLAYBACK //////////////////////////////////////////

CameraReceiverVideo::CameraReceiverVideo() : ICameraReceiver()
{
    _StartCaptureThread();
}

CameraType CameraReceiverVideo::GetType() 
 {
    return CameraType::VIDEO_CAMERA;
 }


bool CameraReceiverVideo::_CaptureFrame()
{
    if (playback_file_changed)
    {
        _InitializeCamera();
        playback_file_changed = false;
    }

    // if the video is not open, return false
    if (!_cap.isOpened())
    {
        std::cerr << "ERROR: video not open!" << std::endl;
        return false;
    }

    if (!playback_play)
    {
        _prevFrameTimer.markStart();
        return false;
    }


    // wait for the previous frame to be at 1/AX_CAP_FPS long
    float videoFramePeriod = 1.0 / MAX_CAP_FPS / playback_speed;

    while (_prevFrameTimer.getElapsedTime() < videoFramePeriod)
    {
        double currelapsedtime = _prevFrameTimer.getElapsedTime();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    float deltaTimeLeft = min(_prevFrameTimer.getElapsedTime() - videoFramePeriod,videoFramePeriod); 

    _prevFrameTimer.markStart(deltaTimeLeft);

    // read the next frame
    if (playback_goback || playback_restart || playback_pause)
    {
        // Go backwards
        if(playback_goback && !playback_pause)
        {
            _videoFrame -= 1;
        }
        // Otherwise we didn't increment it so its stays on the same image


        if (playback_restart)
        {
            _videoFrame = 0;
            playback_restart = false;
        }

        if (_videoFrame < 0)
        {
            _videoFrame = 0;
        }

        // Move to the previous frame
        _cap.set(cv::CAP_PROP_POS_FRAMES, _videoFrame);
    }
    else
    { 
            _videoFrame++;
    }

   
    // read in the frame
    cv::Mat _rawFrame;
    _cap.read(_rawFrame);

    // Convert to gray scale 
    if (_rawFrame.channels() == 1) {
        // Do nothing
    }
    else if (_rawFrame.channels() == 3) {
        cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGR2GRAY);
    }
    else if (_rawFrame.channels() == 4)
    {
        cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGRA2GRAY);
    }

    // Apply processing to it
    cv::Mat finalImage;
// #define PREPROCESS
#ifdef PREPROCESS
    birdsEyePreprocessor.Preprocess(_rawFrame, finalImage);
#else
    finalImage = _rawFrame;
#endif
    std::cout << "size: " << finalImage.size() << std::endl;

    // convert to gray if it is not
    if (_rawFrame.channels() == 3)
    {
        cv::cvtColor(finalImage, finalImage, cv::COLOR_BGR2GRAY);
    }

    // apply mask
    // cv::Mat& mask = TrackingWidget::GetInstance()->GetMask();
    // finalImage.setTo(cv::Scalar(0, 0, 0), mask);

    std::unique_lock<std::mutex> locker(_frameMutex);

    // Copy it over
    finalImage.copyTo(_frame);
    std::cout << "frame size: " << _frame.size() << std::endl;

    // increase _frameID
    _frameID++;
    bool empty = _frame.empty();

    // Update time
    _frameTime = Clock::programClock.getElapsedTime();

    locker.unlock();

    // Notify all frame is ready
    _frameCV.notify_all();

    // if the frame is empty, return false
    if (empty)
    {
        std::cerr << "ERROR: frame empty!" << std::endl;
        return false;
    }

    // return SUCCESS
    return true;
}

bool CameraReceiverVideo::_InitializeCamera()
{
    std::cout << "Initializing playback for " << playback_file << std::endl;
    _cap = cv::VideoCapture(playback_file);
    _videoFrame = 0;

    // if the video is not open, return false
    if (!_cap.isOpened())
    {
        std::cout << "Failed to initialize playback" << std::endl;
        return false;
    }

    // Set the frame rate
    MAX_CAP_FPS = _cap.get(cv::CAP_PROP_FPS);

    // If the property didn't exit properly, set MAX_CAP_FPS to default value
    if (MAX_CAP_FPS > 500.0)
    {
        MAX_CAP_FPS = 100.0;
    }
    std::cout << "Playback initialized with FPS = " << MAX_CAP_FPS << std::endl;
    // return SUCCESS
    return true;
}

////////////////////////////////////////// USB CAMERA //////////////////////////////////////////

CameraReceiverUSB::CameraReceiverUSB() : ICameraReceiver()
{
    _StartCaptureThread();
}

CameraType CameraReceiverUSB::GetType() 
 {
    return CameraType::USB_CAMERA;
 }

bool CameraReceiverUSB::_InitializeCamera()
{
    // Open the default camera
    _cap.open(0);
    // set the frame rate
    _cap.set(cv::CAP_PROP_FPS, MAX_CAP_FPS);

    // if the video is not open, return false
    if (!_cap.isOpened())
    {
        std::cerr << "ERROR: video not open!" << std::endl;
        return false;
    }

    // return SUCCESS
    return true;
}

bool CameraReceiverUSB::_CaptureFrame()
{
    // if the video is not open, return false
    if (!_cap.isOpened())
    {
        std::cerr << "ERROR: video not open!" << std::endl;
        return false;
    }


    // read in the frame
    cv::Mat _rawFrame;
    _cap.read(_rawFrame);

    // Convert to gray scale
    if (_rawFrame.channels() == 1)
    {
        // Do nothing
    }
    else if (_rawFrame.channels() == 3)
    {
        cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGR2GRAY);
    }
    else if (_rawFrame.channels() == 4)
    {
        cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGRA2GRAY);
    }

    // Apply processing to it
    cv::Mat finalImage;
    birdsEyePreprocessor.Preprocess(_rawFrame, finalImage);


    // apply mask
    TrackingWidget* trackingWidget = TrackingWidget::GetInstance();
    if (trackingWidget != nullptr)
    {
        cv::Mat& mask = trackingWidget->GetMask();
        finalImage.setTo(cv::Scalar(0, 0, 0), mask);
    }

    std::unique_lock<std::mutex> locker(_frameMutex);

    // Copy it over
    finalImage.copyTo(_frame);

    // increase _frameID
    _frameID++;

    // Update time
    _frameTime = Clock::programClock.getElapsedTime();

    locker.unlock();

    // Notify all frame is ready
    _frameCV.notify_all();

    // return SUCCESS
    return true;
}

CameraReceiverUSB::~CameraReceiverUSB()
{
    // Release the camera
    _cap.release();
}