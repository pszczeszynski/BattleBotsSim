#include "CVPosition.h"

#include <cstdlib>  // For std::system
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"
#include "../../UIWidgets/ClockWidget.h"


// Maximum time gap (seconds) before resetting velocity
constexpr double MAX_VELOCITY_TIME_GAP = 0.5;
// Minimum consecutive valid frames required before publishing
constexpr int MIN_VALID_STREAK = 3;

// Function to create shared memory and return handle and pointer
bool CreateSharedMemory(const std::string& name, int size, HANDLE& outHandle,
                        void*& outPointer) {
  LPCSTR sharedFileNameLPCSTR = name.c_str();

  HANDLE hMapFile = CreateFileMappingA(
      INVALID_HANDLE_VALUE,   // Use paging file - shared memory
      NULL,                   // Default security attributes
      PAGE_READWRITE,         // Read/write access
      0,                      // Maximum object size (high-order DWORD)
      size,                   // Maximum object size (low-order DWORD)
      sharedFileNameLPCSTR);  // Name of the mapping object

  if (hMapFile == NULL) {
    std::cerr << "Could not create file mapping object: " << GetLastError()
              << std::endl;
    return false;
  }

  void* pBuf = MapViewOfFile(hMapFile,             // Handle to mapping object
                             FILE_MAP_ALL_ACCESS,  // Read/write permission
                             0, 0, size);

  if (pBuf == nullptr) {
    std::cerr << "Could not map view of file: " << GetLastError() << std::endl;
    CloseHandle(hMapFile);
    return false;
  }

  outHandle = hMapFile;
  outPointer = pBuf;
  return true;
}

void CVPosition::_InitSharedImage() {
  int type = CV_8UC1;
  int elementSize = CV_ELEM_SIZE(type);
  int imgSize = width * height * elementSize;

  // Create shared memory and store handle/pointer
  if (!CreateSharedMemory(memName, imgSize, _hMapFile, _pSharedMemory)) {
    std::cerr << "Failed to create shared memory for CVPosition" << std::endl;
    return;
  }

  // Initialize cv::Mat with the shared memory data
  sharedImage = cv::Mat(height, width, type, _pSharedMemory);
}

void CVPosition::_StartPython() {
  return;  // Disabled for now
  const std::string venv_path = "venv";
  const std::string script_path = "CVPosition.py";
  std::string command = "cmd.exe /C \"cd MachineLearning && " + venv_path +
                        "\\Scripts\\activate && python " + script_path +
                        " > NUL 2>&1\"";

  _pythonThreadRunning = true;
  _pythonThread = std::thread([command, this]() {
    int result = std::system(command.c_str());
    if (result != 0) {
      std::cerr << "Failed to run CVPosition.py" << std::endl;
    }
    _pythonThreadRunning = false;
  });
}

CVPosition::CVPosition(ICameraReceiver* videoSource)
    : _pythonSocket("11116"),
      _lastData(CVPositionData()),
      OdometryBase(videoSource),
      _hMapFile(nullptr),
      _pSharedMemory(nullptr) {
  _InitSharedImage();
  _StartPython();
}

CVPosition::~CVPosition() {
  // Stop Python thread if running
  if (_pythonThreadRunning) {
    // Thread will exit naturally when Python script stops
    // We could add a signal mechanism if needed
  }
  if (_pythonThread.joinable()) {
    _pythonThread.join();
  }

  // Clean up shared memory
  if (_pSharedMemory != nullptr) {
    UnmapViewOfFile(_pSharedMemory);
    _pSharedMemory = nullptr;
  }
  if (_hMapFile != nullptr) {
    CloseHandle(_hMapFile);
    _hMapFile = nullptr;
  }
}

cv::Point2f CVPosition::_ComputeCenterFromBBox(const std::vector<int>& bbox) {
  // Assumes bbox format is [x1, y1, x2, y2]
  // Compute center as midpoint
  if (bbox.size() >= 4) {
    float centerX = (bbox[0] + bbox[2]) / 2.0f;
    float centerY = (bbox[1] + bbox[3]) / 2.0f;
    return cv::Point2f(centerX, centerY);
  }
  // Fallback: if bbox[0], bbox[1] are already center (legacy behavior)
  if (bbox.size() >= 2) {
    return cv::Point2f(static_cast<float>(bbox[0]),
                       static_cast<float>(bbox[1]));
  }
  return cv::Point2f(0, 0);
}

cv::Point2f CVPosition::_ComputeVelocity(cv::Point2f currentCenter,
                                         double currentTime,
                                         cv::Point2f lastCenter,
                                         double lastTime) {
  double deltaTime = currentTime - lastTime;

  // Reset velocity if time gap is too large
  if (deltaTime <= 0 || deltaTime > MAX_VELOCITY_TIME_GAP) {
    return cv::Point2f(0, 0);
  }

  cv::Point2f deltaPos = currentCenter - lastCenter;
  return cv::Point2f(static_cast<float>(deltaPos.x / deltaTime),
                     static_cast<float>(deltaPos.y / deltaTime));
}

void CVPosition::_ProcessNewFrame(cv::Mat frame, double frameTime) {
  // Write frameID to the first 4 pixels (allowed image mutation)
  for (int i = 0; i < 4; i++) {
    frame.at<uchar>(0, i) = (frameID >> (8 * i)) & 0xFF;
  }

  // Write time in milliseconds to next 4 pixels
  uint32_t timeMillis = (uint32_t)(frameTime * 1000.0);
  for (int i = 0; i < 4; i++) {
    frame.at<uchar>(0, i + 4) = (timeMillis >> (8 * i)) & 0xFF;
  }

  // Apply brightness adjustment if configured
  if (NEURAL_BRIGHTNESS_ADJUST != 0) {
    cv::Mat adjusted;
    frame.convertTo(adjusted, CV_16U, 1 + NEURAL_BRIGHTNESS_ADJUST / 10.0, 0);
    cv::threshold(adjusted, adjusted, 255, 255, cv::THRESH_TRUNC);
    adjusted.convertTo(frame, CV_8U);
  }

  // Copy to shared memory
  if (!sharedImage.empty()) {
    frame.copyTo(sharedImage);
  }

  // Get data from Python (non-blocking receive)
  bool pythonResponded = false;
  CVPositionData data = _GetDataFromPython(pythonResponded);

  // If Python didn't respond, don't publish (id increments only on Publish)
  if (!pythonResponded) {
    return;
  }

  // Validate bounding box and compute center
  if (data.valid && data.boundingBox.size() >= 4) {
    // Compute center from bounding box midpoint
    data.center = _ComputeCenterFromBBox(data.boundingBox);

    // Validate center is within frame bounds
    if (data.center.x <= 0 || data.center.y <= 0 ||
        data.center.x >= frame.cols || data.center.y >= frame.rows ||
        std::isnan(data.center.x) || std::isnan(data.center.y) ||
        std::isinf(data.center.x) || std::isinf(data.center.y)) {
      data.valid = false;
    }
  } else {
    data.valid = false;
  }

  // Apply gating logic and compute velocity with RAII lock
  cv::Point2f velocity(0, 0);
  bool shouldPublish = false;

  {
    std::lock_guard<std::mutex> lock(_lastDataMutex);

    // Save previous valid state for velocity computation (before updating)
    cv::Point2f prevCenter = _lastValidCenter;
    double prevTime = _lastValidTime;
    bool hadPrevState = _hasPrevValidState;

    if (data.valid) {
      // Check distance jump and frame gap
      double distance = (_lastValidFrameID == 0)
                            ? 0.0
                            : cv::norm(data.center - _lastValidCenter);
      uint32_t frameGap = (_lastValidFrameID == 0)
                              ? 0
                              : (data.frameID > _lastValidFrameID
                                     ? data.frameID - _lastValidFrameID
                                     : 0);

      if (distance > _max_distance_thresh || frameGap > 100) {
        // Reset on large jump or gap
        data.valid = false;
        _validStreakCounter = 0;
        _hasPrevValidState = false;
      } else {
        // Increment valid streak
        _validStreakCounter++;

        // Compute velocity from previous valid state
        if (hadPrevState && prevTime > 0) {
          double currentTime = data.time_millis / 1000.0;
          velocity =
              _ComputeVelocity(data.center, currentTime, prevCenter, prevTime);
        }
      }
    } else {
      // Reset streak on invalid data
      _validStreakCounter = 0;
    }

    // Update last valid state if data is valid
    if (data.valid) {
      _lastData = data;

      // Update current state (previous state was already read into
      // prevCenter/prevTime above)
      _lastValidCenter = data.center;
      _lastValidTime = data.time_millis / 1000.0;
      _lastValidFrameID = data.frameID;
      _hasPrevValidState = true;

      // Check if we should publish (streak sufficient)
      shouldPublish = (_validStreakCounter >= MIN_VALID_STREAK);
    } else {
      _hasPrevValidState = false;
    }
  }

  // Only publish if data is valid and streak is sufficient
  // (id increments only on Publish; no publish on invalid/no-response)
  if (shouldPublish) {
    // Construct OdometryData sample for publishing
    OdometryData sample;
    sample.Clear();

    double posTime = data.time_millis / 1000.0;

    // Create bounding box rect
    cv::Rect bboxRect;
    if (data.boundingBox.size() >= 4) {
      bboxRect = cv::Rect(data.boundingBox[0], data.boundingBox[1],
                          data.boundingBox[2] - data.boundingBox[0],
                          data.boundingBox[3] - data.boundingBox[1]);
    } else {
      // Fallback: create default rect around center
      bboxRect = cv::Rect(static_cast<int>(data.center.x - 10),
                          static_cast<int>(data.center.y - 10), 20, 20);
    }

    // Set position data with velocity and rect
    sample.pos = PositionData(data.center, velocity, bboxRect, posTime);

    // Angle remains invalid (not set)

    // Publish the sample (base class increments id automatically)
    OdometryBase::Publish(sample, /*isOpponent=*/false);
  }
}

std::vector<int> CVPosition::GetBoundingBox(int* outFrameID) {
  std::lock_guard<std::mutex> lock(_lastDataMutex);
  CVPositionData actualData = _lastData;

  if (outFrameID != nullptr) {
    *outFrameID = actualData.frameID;
  }

  return actualData.boundingBox;
}

cv::Point2f CVPosition::GetCenter(int* outFrameID) {
  std::lock_guard<std::mutex> lock(_lastDataMutex);
  CVPositionData actualData = _lastData;

  if (outFrameID != nullptr) {
    *outFrameID = actualData.frameID;
  }

  return actualData.center;
}

CVPositionData CVPosition::_GetDataFromPython(bool& outPythonResponded) {
  // Receive from socket (non-blocking - ServerSocket::receive() uses FIONBIO)
  std::string data = _pythonSocket.receive();

  if (data.empty()) {
    CVPositionData ret;
    {
      std::lock_guard<std::mutex> lock(_lastDataMutex);
      ret = _lastData;  // Return last known data
    }
    ret.valid = false;
    outPythonResponded = false;
    return ret;
  }
  outPythonResponded = true;

  // Parse JSON
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(data);
  } catch (...) {
    CVPositionData ret;
    {
      std::lock_guard<std::mutex> lock(_lastDataMutex);
      ret = _lastData;
    }
    ret.valid = false;
    return ret;
  }

  // Get "bounding_box"
  nlohmann::json boundingBox = j["bounding_box"];

  // Check if it's a string and equal to "invalid"
  if (boundingBox.is_string()) {
    CVPositionData ret;
    {
      std::lock_guard<std::mutex> lock(_lastDataMutex);
      ret = _lastData;
    }
    ret.valid = false;
    return ret;
  }

  // It's an array of floats, convert to ints
  std::vector<float> boundingBoxVec = boundingBox.get<std::vector<float>>();
  std::vector<int> intBoundingBox;
  for (size_t i = 0; i < boundingBoxVec.size(); i++) {
    intBoundingBox.push_back(static_cast<int>(boundingBoxVec[i]));
  }

  // Get confidence and frame_id
  float conf = j["conf"].get<float>();
  uint32_t id = j["frame_id"].get<uint32_t>();
  uint32_t time_milliseconds = j["time_milliseconds"].get<uint32_t>();

  CVPositionData ret;
  ret.boundingBox = intBoundingBox;
  ret.frameID = id;
  ret.time_millis = time_milliseconds;
  ret.valid = conf >= NN_MIN_CONFIDENCE;

  // Center will be computed in _ProcessNewFrame from bbox

  return ret;
}
