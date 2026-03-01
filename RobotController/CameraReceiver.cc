#include "CameraReceiver.h"

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#define NOMINMAX
#include <stdlib.h>
#include <windows.h>

#include <iostream>

#include "Clock.h"
#include "Globals.h"
#include "PlaybackController.h"
#include "RobotConfig.h"
#include "UIWidgets/ClockWidget.h"
#include "UIWidgets/TrackingWidget.h"
#include "VisionPreprocessor.h"

#define GET_FRAME_TIMEOUT_MS 500
#define GET_FRAME_MUTEX_TIMEOUT std::chrono::milliseconds(150)

double MAX_CAP_FPS = 160.0;

// TODO: add a way to save video with the UI
ICameraReceiver* _instance = nullptr;

ICameraReceiver* ICameraReceiver::GetInstance() {
  if (!_instance) {
    std::cerr << "ERROR: CameraReceiver not initialized!" << std::endl;
    return nullptr;
  }
  return _instance;
}

ICameraReceiver::ICameraReceiver() {
  // Disable hardware transforms so it takes less time to initialize
  putenv("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0");
  _instance = this;
}

ICameraReceiver::~ICameraReceiver() { _instance = nullptr; }

void ICameraReceiver::_StartCaptureThread() {
  // create a thread to capture frames
  _captureThread = std::thread([this]() {
    ClockWidget captureTimer{"Camera Grab"};

    // try to initialize camera
    while (!_InitializeCamera()) {
      std::cerr << "ERROR: failed to initialize camera!" << std::endl;
      Sleep(3000);  // wait 1 second
    }

    std::cout << "Camera initialized" << std::endl;

    // start capturing frames
    while (true) {
      captureTimer.markEnd();
      captureTimer.markStart();

      int failedCount = 0;
      bool success = false;
      while (failedCount < 5) {
        success = _CaptureFrame();
        if (success) {
          break;
        } else {
          failedCount++;
          Sleep(100);
        }
      }

      if (!success) {
        std::cerr
            << "[CameraReceiver] _CaptureFrame failed 5x, reinitializing camera"
            << std::endl;
        Sleep(1000);
        // try to initialize camera
        while (!_InitializeCamera()) {
          std::cerr << "ERROR: failed to initialize camera!" << std::endl;
          Sleep(3000);  // wait 1 second
        }
      }
    }
  });
}

/**
 * Attempts to get a frame from the camera
 * @param output the output frame
 * @param old_id the previous id of the frame. If a new frame is not ready it
 * will block If old_id is 0, it will return the latest frame
 * @param frameTime (Optional) pointer to a double that will store the frame
 * elapsed time when it was acquired
 * @return the id of the new frame
 */
long ICameraReceiver::GetFrame(cv::Mat& output, long old_id, double* frameTime,
                               bool blockUntilReady) {
  // Using scoped mutex locker because conditional variable integrates with it
  // This creates and locks the mutex
  std::unique_lock<std::mutex> locker(_frameMutex);

  while ((_frameID <= 0) || (_frameID <= old_id)) {
    if (blockUntilReady) {
      _frameCV.wait(locker);
    } else {
      // Unlock mutex, waits until conditional varables is notifed, then
      // it locks mutex again
      if (_frameCV.wait_for(locker, GET_FRAME_MUTEX_TIMEOUT) ==
          std::cv_status::timeout) {
        // Unable to get a new frame, exit
        return -1;
      }
    }
  }

  // At this point our mutex is locked and a frame is ready
  _frame.copyTo(output);
  old_id = _frameID;

  if (frameTime != NULL) {
    *frameTime = _frameTime;
  }

  locker.unlock();
  // return ID of new frame
  return old_id;
}

// Returns true if a newer frame is ready
bool ICameraReceiver::NewFrameReady(long old_id) { return _frameID > old_id; }

CameraType ICameraReceiver::GetType() { return CameraType::BASE_CLASS; }

////////////////////////////////////////// REAL VERSION
/////////////////////////////////////////////

CameraType CameraReceiver::GetType() { return CameraType::REAL_CAMERA; }

CameraReceiver::CameraReceiver() : ICameraReceiver() { _StartCaptureThread(); }

int ConfigureCamera(Spinnaker::CameraPtr pCam) {
  if (pCam == nullptr) {
    std::cout << "error attempt to configure null camera. Returning"
              << std::endl;
    return -1;
  }

  std::cout << "Configuring camera" << std::endl;
  //
  // NOTE: In general, a fault will occur if you disable a function and try to
  // set its options
  //
  try {
    std::cout << "Initing camera" << std::endl;
    pCam->Init();

    std::cout << "Setting up camera" << std::endl;

    // Disable heartbeat (a GiGe standard requiring program to reconfirm
    // periodically its alive)
    pCam->GevGVCPHeartbeatDisable();

    // ******** General Settings ***************
    // Set exposure mode
    // pCam->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);  //
    // turn off auto pCam->ExposureMode.SetValue(Spinnaker::ExposureMode_Timed);
    // // set it to fixed time pCam->ExposureTime.SetValue(10002.0); // Set to
    // 2ms. Longer is less noisy

    // Set Gain Mode
    // pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Once); // Turn
    // off auto gain pCam->Gain.SetValue(8.0); // Set gain in dB (between 0
    // and 47.99)

    // Turn off Gama correction
    pCam->GammaEnable.SetValue(false);
    // pCam->Gamma.SetValue(1.0f, false); // Gamma correction if enabled. 0 to 4
    // nominal range

    // Turn off white balance
    // pCam->BalanceWhiteAuto.SetValue(Spinnaker::BalanceWhiteAuto_Off);

    // Allocate all bandwidth to this one camera 125000000 is max, reduce to
    // 122000000 for margin
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

    // pCam->IspEnable.SetValue(false);                         // Turn off
    // image processing pCam->AdcBitDepth.SetValue(Spinnaker::AdcBitDepth_Bit8);
    // // Set to 8-bit color resolution

    // ******* Output Data Settings *******

    // pCam->PixelFormat.SetValue(Spinnaker::PixelFormat_Mono8); // Only some of
    // the formats work, this is one of them and is fast.

    // Compression may be useful, but not tested for delay
    // pCam->ImageCompressionMode.SetValue(Spinnaker::ImageCompressionModeEnums::ImageCompressionMode_Off);

    // Set Acquisition to continouse
    // pCam->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

    //
    // Make it always return latest image (This option is not available via the
    // easy access mode)

    Spinnaker::GenApi::INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();
    Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode =
        sNodeMap.GetNode("StreamBufferHandlingMode");
    ptrHandlingMode->SetIntValue(
        Spinnaker::StreamBufferHandlingMode_NewestOnly);

    // Save settings to user register 0
    // pCam->UserSetSelector = UserSetSelectorEnums::UserSetSelector_UserSet0;
    // pCam->UserSetSave();
  } catch (Spinnaker::Exception& e) {
    std::cout << "********* CONFIG ERROR *********" << std::endl;
    std::cout << "Error: " << e.what() << std::endl << std::endl;
    return -1;
  }

  return 0;
}

bool CameraReceiver::_InitializeCamera() {
  std::cout << "about to get system" << std::endl;
  // Retrieve singleton reference to system object
  _system = Spinnaker::System::GetInstance();
  std::cout << "got system" << std::endl;

  // Retrieve list of cameras from the system
  Spinnaker::CameraList camList = _system->GetCameras();

  std::cout << "got camera list" << std::endl;

  const unsigned int numCameras = camList.GetSize();
  std::cerr << "Number of cameras detected: " << numCameras << std::endl
            << std::endl;

  // Finish if there are no cameras
  if (numCameras == 0) {
    if (pCam != nullptr) {
      try {
        // Check if the camera is initialized before de-initializing
        if (pCam->IsInitialized()) {
          pCam->DeInit();
        }
      } catch (Spinnaker::Exception& e) {
        std::cerr << "Error during DeInit: " << e.what() << std::endl;
      }
      pCam = nullptr;
    }

    // Clear camera list before releasing system
    camList.Clear();

    if (_system != nullptr) {
      // Release system
      _system->ReleaseInstance();
      _system = nullptr;
    }

    return false;
  }

  // Get the first camera
  pCam = camList.GetByIndex(0);

  int configRet = -1;
  try {
    configRet = ConfigureCamera(pCam);
  }
  // any exception
  catch (Spinnaker::Exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    configRet = -1;
  }

  // Initialize
  if (configRet < 0) {
    std::cerr << "Failed to configure camera" << std::endl;

    if (pCam != nullptr) {
      try {
        // Check if the camera is initialized before de-initializing
        if (pCam->IsInitialized()) {
          pCam->DeInit();
        }
      } catch (Spinnaker::Exception& e) {
        std::cerr << "Error during DeInit: " << e.what() << std::endl;
      }

      pCam = nullptr;
    }

    std::cout << "about to clear camlist" << std::endl;
    // Clear camera list before releasing system
    camList.Clear();

    std::cout << "about to release system" << std::endl;

    if (_system != nullptr) {
      try {
        // Release system
        _system->ReleaseInstance();
      } catch (Spinnaker::Exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
      }
      _system = nullptr;
    }

    std::cout << "returning false" << std::endl;
    // return failure
    return false;
  }

  try {
    // Start acquisition
    pCam->BeginAcquisition();

    pcam_image_width = (int)pCam->Width.GetValue();  // Max 1440 for this camera
    pcam_image_height =
        (int)pCam->Height.GetValue();  // Max 1080 for this camera
  } catch (Spinnaker::Exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    if (pCam != nullptr) {
      try {
        // Check if the camera is initialized before de-initializing
        if (pCam->IsInitialized()) {
          pCam->DeInit();
        }
      } catch (Spinnaker::Exception& e) {
        std::cerr << "Error during DeInit: " << e.what() << std::endl;
      }
      pCam = nullptr;
    }

    // Clear camera list before releasing system
    camList.Clear();

    if (_system != nullptr) {
      // Release system
      _system->ReleaseInstance();
      _system = nullptr;
    }

    // return failure
    return false;
  }

  // return success
  return true;
}

void CameraReceiver::SetCameraGain(float gain) {
  if (pCam == nullptr) {
    return;
  }
  pCam->Gain.SetValue(gain);  // Set gain in dB (between 0 and 47.99)
}

bool CameraReceiver::_CaptureFrame() {
  if (pCam == nullptr) {
    std::cerr << "ERROR: camera not initialized!" << std::endl;
    return false;
  }
  // Get Next Image
  Spinnaker::ImagePtr pResultImage = nullptr;

  try {
    // Attempt to get the next image
    pResultImage = pCam->GetNextImage(GET_FRAME_TIMEOUT_MS);
  } catch (Spinnaker::Exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    // Handle camera disconnection
    pCam = nullptr;

    return false;
  }

  if (!pResultImage || pResultImage->IsIncomplete()) {
    std::cerr << "ERROR: Failed to capture frame!" << std::endl;
    return false;
  }

  unsigned char* img_data = (unsigned char*)pResultImage->GetData();

  // if (InputState::GetInstance().IsKeyDown(ImGuiKey_U)) {
  //     // randomly flip bits in the image data
  //     for (int i = 0; i < pcam_image_width * pcam_image_height; i++) {
  //         img_data[i] = img_data[i] ^ (rand() % 255);
  //     }
  // }

  // Create a Mat with the data
  cv::Mat bayerImage(pcam_image_height, pcam_image_width, CV_8UC1, img_data);

  // if (InputState::GetInstance().IsKeyDown(ImGuiKey_U)) {
  //     // make image fully white
  //     bayerImage.setTo(cv::Scalar(255, 255, 255));
  // }

  // Apply processing to it (including field mask if available)
  cv::Mat finalImage;
  TrackingWidget* trackingWidget = TrackingWidget::GetInstance();
  const cv::Mat* fieldMask =
      (trackingWidget != nullptr) ? &trackingWidget->GetMask() : nullptr;
  birdsEyePreprocessor.Preprocess(bayerImage, finalImage, fieldMask);

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

CameraReceiver::~CameraReceiver() {}

///////////////////////////////////////// SIMULATION
/////////////////////////////////////////////
CameraReceiverSim::CameraReceiverSim(std::string sharedFileName)
    : ICameraReceiver(), _sharedFileName(sharedFileName) {
  _StartCaptureThread();
}

CameraType CameraReceiverSim::GetType() { return CameraType::SIM_CAMERA; }

bool CameraReceiverSim::_InitializeCamera() {
  std::wstring sharedFileNameW(_sharedFileName.begin(), _sharedFileName.end());
  // LPCWSTR sharedFileNameLPCWSTR = sharedFileNameW.c_str();

  // 1. Create shared file
  // Open a handle to the memory-mapped file
  _hMapFile =
      OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, sharedFileNameW.c_str());

  if (_hMapFile == NULL) {
    std::cerr << "Could not open memory-mapped file" << std::endl;
    // get error
    DWORD err = GetLastError();
    if (err == 6)  // ERROR_INVALID_HANDLE
    {
      std::cerr << "ERROR_INVALID_HANDLE" << std::endl;
    }
    std::cerr << "Error: " << err << std::endl;
    return false;
  }

  // Map the memory-mapped file to a memory address
  _lpMapAddress = MapViewOfFile(_hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);
  if (_lpMapAddress == NULL) {
    std::cerr << "Could not map memory-mapped file" << std::endl;
    CloseHandle(_hMapFile);
    return false;
  }

  // Extract width and height from the start of the shared memory
  int* dimensions = static_cast<int*>(_lpMapAddress);
  _width = dimensions[0];   // First integer is width
  _height = dimensions[1];  // Second integer is height

  void* imageData = static_cast<char*>(_lpMapAddress) + 2 * sizeof(int);
  _image = cv::Mat(cv::Size(_width, _height), CV_8UC4, imageData);
  std::cout << "Opened shared file: " << _sharedFileName << " with size "
            << _width << "x" << _height << std::endl;

  // return success
  return true;
}

bool CameraReceiverSim::_CaptureFrame() {
  // wait for the previous frame to be at 1/MAX_CAP_FPS long
  while (_prevFrameTimer.getElapsedTime() < 1.0 / MAX_CAP_FPS) {
    double currelapsedtime = _prevFrameTimer.getElapsedTime();

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Update image
  while (!UpdateImageFromSharedMemory()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  _prevFrameTimer.markStart(_prevFrameTimer.getElapsedTime() -
                            1.0 / MAX_CAP_FPS);

  cv::Mat captured;

  // remove alpha and convert to B&W
  if (_image.channels() == 1) {
    captured = _image;
  } else if (_image.channels() == 3) {
    cv::cvtColor(_image, captured, cv::COLOR_BGR2GRAY);
  } else if (_image.channels() == 4) {
    cv::cvtColor(_image, captured, cv::COLOR_BGRA2GRAY);
  }

  // Flip the image vertically
  cv::flip(captured, captured, 0);

  // Apply processing to it (including field mask if available)
  cv::Mat finalImage;
  TrackingWidget* trackingWidget = TrackingWidget::GetInstance();
  const cv::Mat* fieldMask =
      (trackingWidget != nullptr) ? &trackingWidget->GetMask() : nullptr;
  birdsEyePreprocessor.Preprocess(captured, finalImage, fieldMask);

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

// Ensure UpdateImageFromSharedMemory accounts for the third integer
bool CameraReceiverSim::UpdateImageFromSharedMemory() {
  // Open the named event
  HANDLE frameReadyEvent =
      OpenEventW(EVENT_ALL_ACCESS, FALSE, L"FrameReadyEvent");
  if (frameReadyEvent == NULL) {
    std::cerr << "Could not open FrameReadyEvent: " << GetLastError()
              << std::endl;
    return false;
  }

  // Wait for the event to be signaled (new frame available)
  DWORD waitResult =
      WaitForSingleObject(frameReadyEvent, 1000);  // Timeout after 1 second
  CloseHandle(frameReadyEvent);                    // Close handle after use
  if (waitResult != WAIT_OBJECT_0) {
    std::cerr << "Wait for FrameReadyEvent failed or timed out: " << waitResult
              << std::endl;
    return false;
  }

  if (_lpMapAddress == NULL) {
    std::cerr << "Shared memory not mapped" << std::endl;
    return false;
  }

  // Extract width, height, and frame number from the start of the shared memory
  int* dimensions = static_cast<int*>(_lpMapAddress);
  int newWidth = dimensions[0];   // First integer is width
  int newHeight = dimensions[1];  // Second integer is height

  // Validate dimensions
  if (newWidth <= 0 || newHeight <= 0) {
    std::cerr << "Invalid dimensions in shared memory: " << newWidth << "x"
              << newHeight << std::endl;
    return false;
  }

  // Update only if dimensions have changed
  if (newWidth != _width || newHeight != _height) {
    _width = newWidth;
    _height = newHeight;
    // Update cv::Mat with new dimensions and data pointer (skip three integers)
    void* imageData = static_cast<char*>(_lpMapAddress) + 3 * sizeof(int);
    _image = cv::Mat(cv::Size(_width, _height), CV_8UC4, imageData);
    std::cout << "Updated image dimensions to: " << _width << "x" << _height
              << std::endl;
  }

  // Check if the frame number has changed (indicating a new frame)
  int currentFrameNumber = dimensions[2];  // Third integer is frame number
  if (currentFrameNumber == _lastFrameNumber) {
    // No new frame available
    return false;
  }
  _lastFrameNumber = currentFrameNumber;

  return true;
}

CameraReceiverSim::~CameraReceiverSim() {
  // Unmap the memory-mapped file and close the handle
  UnmapViewOfFile(_lpMapAddress);
  CloseHandle(_hMapFile);
}

////////////////////////////////////////// VIDEO PLAYBACK
/////////////////////////////////////////////

CameraReceiverVideo::CameraReceiverVideo() : ICameraReceiver() {
  _StartCaptureThread();
}

CameraType CameraReceiverVideo::GetType() { return CameraType::VIDEO_CAMERA; }

bool CameraReceiverVideo::_CaptureFrame() {
  PlaybackController& playback = PlaybackController::GetInstance();

  // 1. File change: reinitialize and skip this iteration
  if (playback.HasFileChanged()) {
    std::cout << "[CameraReceiverVideo] HasFileChanged=true, calling "
                 "_InitializeCamera"
              << std::endl;
    _InitializeCamera();
    return false;
  }

  if (!_cap.isOpened()) {
    std::cerr << "ERROR: video not open!" << std::endl;
    return false;
  }

  bool needSeek = false;

  // 2. Paused: produce no frames unless seek or step-forward/back
  if (!playback.IsPlaying() || playback.IsPaused()) {
    int64_t seekFrame;
    if (playback.ConsumeSeekFrame(seekFrame)) {
      _videoFrame = seekFrame;
      needSeek = true;
    } else {
      StepRequest step = playback.ConsumeStepRequest();
      if (step == StepRequest::StepForward) {
        _videoFrame = (std::min)(_videoFrame + 1, _maxFrameCount - 1);
        if (_videoFrame < 0) _videoFrame = 0;
        needSeek = true;
      } else if (step == StepRequest::StepBackward) {
        _videoFrame = (std::max)(_videoFrame - 1, 0L);
        needSeek = true;
      } else {
        _prevFrameTimer.markStart();
        return true;  // Paused, no step/seek: no frame produced, but not a
                      // failure (avoids reinit)
      }
    }
  } else {
    // 3. Seek frame (user scrubber) - when not paused
    int64_t seekFrame;
    if (playback.ConsumeSeekFrame(seekFrame)) {
      _videoFrame = seekFrame;
      needSeek = true;
    }

    // 4. Restart
    if (playback.ShouldRestart()) {
      _videoFrame = 0;
      needSeek = true;
    }

    // 5. Reversing (seek every frame when going backwards)
    if (playback.IsReversing()) {
      _videoFrame = (std::max)(_videoFrame - 1, 0L);
      needSeek = true;
    }

    // 6. Normal play: no seek, read next frame
    if (!needSeek) {
      _videoFrame = playback.GetFrame();
      if (_videoFrame >= _maxFrameCount - 1) {
        _videoFrame = _maxFrameCount - 1;
        needSeek = true;  // Stay on last frame
      } else {
        _videoFrame++;
      }
    }
  }

  // Clamp _videoFrame
  _videoFrame = (std::max)(0L, (std::min)(_videoFrame, _maxFrameCount - 1));

  // Rate limiting for play/reverse (not when paused/stepping)
  if (playback.IsPlaying() && !playback.IsPaused()) {
    float videoFramePeriod = 1.0f / MAX_CAP_FPS / playback.GetSpeed();
    while (_prevFrameTimer.getElapsedTime() < videoFramePeriod) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    float deltaTimeLeft =
        (std::min)((float)(_prevFrameTimer.getElapsedTime() - videoFramePeriod),
                   videoFramePeriod);
    _prevFrameTimer.markStart(deltaTimeLeft);
  }

  // Seek ONLY when needed (discontinuity)
  if (needSeek) {
    _cap.set(cv::CAP_PROP_POS_FRAMES, _videoFrame);
  }

  // Read exactly one frame
  cv::Mat _rawFrame;
  if (!_cap.read(_rawFrame)) {
    return false;
  }

  // Determine frame we just read: when we sought, use _videoFrame; else use cap
  // position
  int64_t frameRead;
  if (needSeek) {
    frameRead = _videoFrame;
  } else {
    double pos = _cap.get(cv::CAP_PROP_POS_FRAMES);
    frameRead = static_cast<int64_t>(pos) - 1;
    if (frameRead < 0) frameRead = 0;
    frameRead = (std::min)((int64_t)frameRead, (int64_t)_maxFrameCount - 1);
    _videoFrame = frameRead;
  }
  playback.SetFrame(frameRead);

  // Convert to gray scale
  if (_rawFrame.channels() == 1) {
    // Do nothing
  } else if (_rawFrame.channels() == 3) {
    cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGR2GRAY);
  } else if (_rawFrame.channels() == 4) {
    cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGRA2GRAY);
  }

  // Apply processing
  cv::Mat finalImage;
  bool isImageCorrectSize = false;
  if (!PLAYBACK_PREPROCESS) {
    if (_rawFrame.size().width <= WIDTH && _rawFrame.size().height <= HEIGHT) {
      isImageCorrectSize = true;
      finalImage = cv::Mat::zeros(HEIGHT, WIDTH, _rawFrame.type());
      int xOffset = (WIDTH - _rawFrame.size().width) / 2;
      int yOffset = (HEIGHT - _rawFrame.size().height) / 2;
      cv::Rect roi(xOffset, yOffset, _rawFrame.size().width,
                   _rawFrame.size().height);
      _rawFrame.copyTo(finalImage(roi));
    }
  }
  if (!isImageCorrectSize || PLAYBACK_PREPROCESS) {
    PLAYBACK_PREPROCESS = true;
    TrackingWidget* tw = TrackingWidget::GetInstance();
    const cv::Mat* fieldMask = (tw != nullptr) ? &tw->GetMask() : nullptr;
    birdsEyePreprocessor.Preprocess(_rawFrame, finalImage, fieldMask);
  }
  if (_rawFrame.channels() == 3) {
    cv::cvtColor(finalImage, finalImage, cv::COLOR_BGR2GRAY);
  }

  std::unique_lock<std::mutex> locker(_frameMutex);
  finalImage.copyTo(_frame);
  _frameID++;
  bool empty = _frame.empty();
  _frameTime = Clock::programClock.getElapsedTime();
  locker.unlock();
  _frameCV.notify_all();

  if (empty) {
    std::cerr << "ERROR: frame empty!" << std::endl;
    return false;
  }
  return true;
}

bool CameraReceiverVideo::_InitializeCamera() {
  PlaybackController& playback = PlaybackController::GetInstance();

  std::string filename = playback.GetFile();
  std::cout << "[CameraReceiverVideo] _InitializeCamera called for " << filename
            << std::endl;
  _cap = cv::VideoCapture(filename);
  _videoFrame = 0;

  // if the video is not open, return false
  if (!_cap.isOpened()) {
    std::cout << "Failed to initialize playback" << std::endl;
    return false;
  }

  // Read first frame to initialize backend
  cv::Mat frame;
  _cap >> frame;

  // Reset to start if needed
  _cap.set(cv::CAP_PROP_POS_FRAMES, 0);

  // Set the frame rate
  MAX_CAP_FPS = _cap.get(cv::CAP_PROP_FPS);
  _maxFrameCount = static_cast<long>(_cap.get(cv::CAP_PROP_FRAME_COUNT));
  if (_maxFrameCount <= 0) {
    // Alternative: Seek to end to estimate duration (backend-dependent)
    _cap.set(cv::CAP_PROP_POS_MSEC, 1e9);  // Seek to far future
    double durationMs = _cap.get(cv::CAP_PROP_POS_MSEC);
    _cap.set(cv::CAP_PROP_POS_FRAMES, 0);  // Reset
    _maxFrameCount = static_cast<long>(durationMs / 1000.0 * MAX_CAP_FPS);
    if (_maxFrameCount <= 0) _maxFrameCount = 1;
  }
  playback.SetFrameCount(static_cast<int64_t>(_maxFrameCount));
  playback.SetFrame(0);

  // If the property didn't exit properly, set MAX_CAP_FPS to default value
  if (MAX_CAP_FPS > 500.0) {
    MAX_CAP_FPS = 100.0;
  }
  std::cout << "Playback initialized with FPS = " << MAX_CAP_FPS << std::endl;
  // return SUCCESS
  return true;
}

////////////////////////////////////////// USB CAMERA
/////////////////////////////////////////////

CameraReceiverUSB::CameraReceiverUSB() : ICameraReceiver() {
  _StartCaptureThread();
}

CameraType CameraReceiverUSB::GetType() { return CameraType::USB_CAMERA; }

bool CameraReceiverUSB::_InitializeCamera() {
  // Open the default camera
  _cap.open(0);
  // set the frame rate
  _cap.set(cv::CAP_PROP_FPS, MAX_CAP_FPS);

  // if the video is not open, return false
  if (!_cap.isOpened()) {
    std::cerr << "ERROR: video not open!" << std::endl;
    return false;
  }

  // return SUCCESS
  return true;
}

bool CameraReceiverUSB::_CaptureFrame() {
  // if the video is not open, return false
  if (!_cap.isOpened()) {
    std::cerr << "ERROR: video not open!" << std::endl;
    return false;
  }

  // read in the frame
  cv::Mat _rawFrame;
  _cap.read(_rawFrame);

  // Convert to gray scale
  if (_rawFrame.channels() == 1) {
    // Do nothing
  } else if (_rawFrame.channels() == 3) {
    cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGR2GRAY);
  } else if (_rawFrame.channels() == 4) {
    cv::cvtColor(_rawFrame, _rawFrame, cv::COLOR_BGRA2GRAY);
  }

  // Apply processing to it (including field mask if available)
  cv::Mat finalImage;
  TrackingWidget* trackingWidget = TrackingWidget::GetInstance();
  const cv::Mat* fieldMask =
      (trackingWidget != nullptr) ? &trackingWidget->GetMask() : nullptr;
  birdsEyePreprocessor.Preprocess(_rawFrame, finalImage, fieldMask);

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

CameraReceiverUSB::~CameraReceiverUSB() {
  // Release the camera
  _cap.release();
}