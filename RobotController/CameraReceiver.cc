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
        std::cout << "Camera initialized" << std::endl;
        // start capturing frames
        while (true)
        {
            _CaptureFrame();

#ifdef SAVE_VIDEO
            if (InputState::GetInstance().IsKeyDown(ImGuiKey_Enter))
            {
                saveVideo = true;
            }
            if (InputState::GetInstance().IsKeyDown(ImGuiKey_Backspace))
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
        pCam->BalanceWhiteAuto.SetValue(Spinnaker::BalanceWhiteAuto_Off);

        // Allocate all bandwidth to this one camera 125000000 is max, reduce to 122000000 for margin
        pCam->DeviceLinkThroughputLimit.SetValue(122000000);

        // Turn off trigger
        pCam->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);

        // Turn off sequencer
        pCam->SequencerConfigurationMode.SetValue(Spinnaker::SequencerConfigurationMode_Off);
        pCam->SequencerFeatureEnable.SetValue(false);
        // pCam->SequencerMode.SetValue( SequencerModeEnums::SequencerMode_Off);

        // ******** Image Settings ***************
        // Reducing image size saves bandwidth for faster refresh rate
        // We need to reset offsets first before changing this
        pCam->OffsetX.SetValue(0);
        pCam->OffsetY.SetValue(0);

        // Now setting width/height should cause issues
        int my_width = 1440;
        int my_height = 600;

        try
        {
            pCam->Width.SetValue(my_width);   // Max 1440 for this camera
            pCam->Height.SetValue(my_height); // Max 1080 for this camera
        }
        catch (Spinnaker::Exception &e)
        {
            std::cout << "Error: " << e.what() << std::endl;
            return -1;
        }

        pCam->OffsetX.SetValue((1440 - my_width) / 2);  // Set to center of ccd
        pCam->OffsetY.SetValue((1080 - my_height) / 2); // Set to center of ccd

        pCam->IspEnable.SetValue(false);                         // Turn off image processing
        pCam->AdcBitDepth.SetValue(Spinnaker::AdcBitDepth_Bit8); // Set to 8-bit color resolution

        // ******* Output Data Settings *******

        pCam->PixelFormat.SetValue(Spinnaker::PixelFormat_BayerRG8); // Only some of the formats work, this is one of them and is fast.

        // Compression may be useful, but not tested for delay
        pCam->ImageCompressionMode.SetValue(Spinnaker::ImageCompressionModeEnums::ImageCompressionMode_Off);

        // Set Acquisition to continouse
        pCam->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

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

#define GET_FRAME_TIMEOUT_MS 500
void CameraReceiver::_CaptureFrame()
{
    std::cout << "Capturing frame" << std::endl;
    // Get Next Image
    Spinnaker::ImagePtr pResultImage = pCam->GetNextImage(GET_FRAME_TIMEOUT_MS);
    // Get char data
    unsigned char *img_data = (unsigned char *)pResultImage->GetData();
    // Create a Mat with the data
    cv::Mat bayerImage(pcam_image_height, pcam_image_width, CV_8UC1, img_data);

    // lock the mutex
    _frameMutex.lock();
    cv::cvtColor(bayerImage, _frame, cv::COLOR_BayerRGGB2BGR);
    // increase _framesReady
    _framesReady++;
    // unlock the mutex
    _frameMutex.unlock();

    std::cout << "Frame captured" << std::endl;
}

bool CameraReceiver::_InitializeCamera()
{
    // Retrieve singleton reference to system object
    _system = Spinnaker::System::GetInstance();

    // Retrieve list of cameras from the system
    Spinnaker::CameraList camList = _system->GetCameras();

    const unsigned int numCameras = camList.GetSize();
    std::cout << "Number of cameras detected: " << numCameras << std::endl
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

    std::cout << "about to get by index" << std::endl;
    // Get the first camera
    pCam = camList.GetByIndex(0);

    std::cout << "got by index" << std::endl;

    // Initialize
    if (ConfigureCamera(pCam) < 0)
    {

        std::cout << "Failed to configure camera" << std::endl;
        // Failed, exit
        pCam = nullptr;

        // Release system
        _system->ReleaseInstance();
        getchar();

        // return failure
        return false;
    }

    std::cout << "about to begin acquisition" << std::endl;
    // Start acquisition
    pCam->BeginAcquisition();

    pcam_image_width = (int)pCam->Width.GetValue();   // Max 1440 for this camera
    pcam_image_height = (int)pCam->Height.GetValue(); // Max 1080 for this camera

    std::cout << "acquisition started" << std::endl;

    // return success
    return true;
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
    cv::cvtColor(_image, captured, cv::COLOR_BGRA2RGB);
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
