#include "CVPosition.h"

#include <cstdint>
#include <cstdlib>
#include <new>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "../../CameraReceiver.h"
#include "../../Clock.h"
#include "../../RobotConfig.h"
#include "../../UIWidgets/GraphWidget.h"

namespace {

constexpr double MAX_VELOCITY_TIME_GAP = 0.5;
constexpr int MIN_VALID_STREAK = 3;

std::optional<BBoxCxCyWh> BBoxFromData(const CVPositionData& d) {
  if (d.boundingBox.size() != 4) return std::nullopt;
  float cx = static_cast<float>(d.boundingBox[0]);
  float cy = static_cast<float>(d.boundingBox[1]);
  float w = static_cast<float>(d.boundingBox[2]);
  float h = static_cast<float>(d.boundingBox[3]);
  if (w < 0 || h < 0) return std::nullopt;
  return BBoxCxCyWh{cx, cy, w, h};
}

cv::Point2f CenterFromBBox(const BBoxCxCyWh& b) {
  return cv::Point2f(b.cx, b.cy);
}

void ClampRectToFrameImpl(float cx, float cy, float w, float h, int frameWidth,
                          int frameHeight, int& outX, int& outY, int& outW,
                          int& outH) {
  float hw = w * 0.5f, hh = h * 0.5f;
  int x1 = static_cast<int>(cx - hw);
  int y1 = static_cast<int>(cy - hh);
  int x2 = static_cast<int>(cx + hw);
  int y2 = static_cast<int>(cy + hh);
  x1 = std::max(0, std::min(x1, frameWidth - 1));
  y1 = std::max(0, std::min(y1, frameHeight - 1));
  x2 = std::max(0, std::min(x2, frameWidth));
  y2 = std::max(0, std::min(y2, frameHeight));
  outW = std::max(1, x2 - x1);
  outH = std::max(1, y2 - y1);
  outX = x1;
  outY = y1;
}

cv::Rect BBoxToRectClamped(const BBoxCxCyWh& b, int frameWidth,
                           int frameHeight) {
  int x, y, w, h;
  ClampRectToFrameImpl(b.cx, b.cy, b.w, b.h, frameWidth, frameHeight, x, y, w,
                       h);
  return cv::Rect(x, y, w, h);
}

}  // namespace

// Debug graphs for publishing (valid streak, distance from last, frameGap,
// valid)
static GraphWidget s_validStreakGraph("CVPosition valid streak", 0, 20, "");
static GraphWidget s_distanceFromLastGraph("CVPosition distance from last", 0,
                                           200, "px");
static GraphWidget s_frameGapGraph("CVPosition frame gap", 0, 100, "");
static GraphWidget s_validGraph("CVPosition valid", 0, 1, "");
static GraphWidget s_publishedPosXGraph("CVPosition published pos X", 0, 720,
                                        "px");
static GraphWidget s_publishedPosYGraph("CVPosition published pos Y", 0, 720,
                                        "px");
static GraphWidget s_timeMillisGraph("CVPosition time_millis", 0, 120, "s");
static GraphWidget s_programClockGraph("CVPosition program clock", 0, 120, "s");

std::optional<BBoxCxCyWh> ParseBBoxCxCyWh(float cx, float cy, float w,
                                          float h) {
  if (w < 0 || h < 0) return std::nullopt;
  return BBoxCxCyWh{cx, cy, w, h};
}

void ClampRectToFrame(float cx, float cy, float w, float h, int frameWidth,
                      int frameHeight, int& outX, int& outY, int& outW,
                      int& outH) {
  ClampRectToFrameImpl(cx, cy, w, h, frameWidth, frameHeight, outX, outY, outW,
                       outH);
}

static bool CreateSharedMemory(const std::string& name, int size,
                               HANDLE& outHandle, void*& outPointer) {
  LPCSTR sharedFileNameLPCSTR = name.c_str();
  HANDLE hMapFile =
      CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size,
                         sharedFileNameLPCSTR);
  if (hMapFile == NULL) {
    std::cerr << "CVPosition: CreateFileMapping failed: " << GetLastError()
              << std::endl;
    return false;
  }
  void* pBuf = MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, size);
  if (pBuf == nullptr) {
    std::cerr << "CVPosition: MapViewOfFile failed: " << GetLastError()
              << std::endl;
    CloseHandle(hMapFile);
    return false;
  }
  outHandle = hMapFile;
  outPointer = pBuf;
  return true;
}

void CVPosition::_InitSharedImage() {
  const int type = CV_8UC1;
  const int elementSize = CV_ELEM_SIZE(type);
  const int imgSize = width * height * elementSize;
  const int totalSize = static_cast<int>(sizeof(SharedImageHeader)) + imgSize;
  if (!CreateSharedMemory(memName, totalSize, _hMapFile, _pSharedMemory)) {
    std::cerr << "CVPosition: Failed to create shared memory" << std::endl;
    return;
  }
  // Placement-new the header so seq is a valid atomic; image follows immediately.
  auto* header = new (_pSharedMemory) SharedImageHeader{};
  header->seq.store(0, std::memory_order_relaxed);  // init only; no reader yet
  sharedImage = cv::Mat(height, width, type,
                        static_cast<char*>(_pSharedMemory) + sizeof(SharedImageHeader));
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
    if (result != 0)
      std::cerr << "CVPosition: Failed to run CVPosition.py" << std::endl;
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
  if (_pythonThread.joinable()) _pythonThread.join();
  if (_pSharedMemory != nullptr) {
    UnmapViewOfFile(_pSharedMemory);
    _pSharedMemory = nullptr;
  }
  if (_hMapFile != nullptr) {
    CloseHandle(_hMapFile);
    _hMapFile = nullptr;
  }
}

cv::Point2f CVPosition::_ComputeVelocity(cv::Point2f currentCenter,
                                         double currentTime,
                                         const LastValidSample& last) const {
  double dt = currentTime - last.t;
  if (dt <= 0 || dt > MAX_VELOCITY_TIME_GAP) return cv::Point2f(0, 0);
  cv::Point2f delta = currentCenter - last.center;
  return cv::Point2f(static_cast<float>(delta.x / dt),
                     static_cast<float>(delta.y / dt));
}

void CVPosition::_ProcessNewFrame(cv::Mat frame, double frameTime) {
  if (frame.type() != CV_8UC1 || frame.rows < 1 || frame.cols < 8) {
    std::cerr << "invalid frame type or size" << std::endl;
    return;
  }
  const uint32_t id = _inputFrameId++;
  // Clamp negative frameTime to avoid uint32_t wrap when casting.
  const double tMs = frameTime >= 0 ? frameTime * 1000.0 : 0.0;
  const uint32_t timeMillis = static_cast<uint32_t>(tMs);

  if (NEURAL_BRIGHTNESS_ADJUST != 0) {
    cv::Mat adjusted;
    frame.convertTo(adjusted, CV_16U, 1 + NEURAL_BRIGHTNESS_ADJUST / 10.0, 0);
    cv::threshold(adjusted, adjusted, 255, 255, cv::THRESH_TRUNC);
    adjusted.convertTo(frame, CV_8U);
  }

  // Seqlock write: seq odd => "writing"; then publish header + image; then seq even => "done".
  // Release on seq stores ensures a reader that sees the same even seq before/after sees consistent header+image.
  if (!sharedImage.empty() && _pSharedMemory != nullptr) {
    auto* header = static_cast<SharedImageHeader*>(_pSharedMemory);
    uint32_t prev = header->seq.load(std::memory_order_relaxed);
    uint32_t odd = (prev + 1) | 1u;
    header->seq.store(odd, std::memory_order_release);

    header->frame_id = id;
    header->time_ms = timeMillis;
    frame.copyTo(sharedImage);

    header->seq.store(odd + 1, std::memory_order_release);
  }

  bool pythonResponded = false;
  CVPositionData data = _GetDataFromPython(pythonResponded);

  if (!pythonResponded) {
    return;
  }

  auto bboxOpt = BBoxFromData(data);
  if (!bboxOpt) {
    data.valid = false;
  } else {
    data.center = CenterFromBBox(*bboxOpt);
    if (data.center.x < 0 || data.center.y < 0 || data.center.x >= frame.cols ||
        data.center.y >= frame.rows || std::isnan(data.center.x) ||
        std::isnan(data.center.y) || std::isinf(data.center.x) ||
        std::isinf(data.center.y)) {
      data.valid = false;
      std::cerr << "center out of frame or non-finite" << std::endl;
    }
  }

  cv::Point2f velocity(0, 0);
  bool shouldPublish = false;
  double distanceFromLast = 0.0;
  uint32_t frameGap = 0;
  int validStreakForGraph = 0;

  {
    std::lock_guard<std::mutex> lock(_lastDataMutex);

    if (data.valid) {
      double currentTime = data.time_millis / 1000.0;

      if (_lastValid) {
        distanceFromLast = cv::norm(data.center - _lastValid->center);
        frameGap = data.frameID - _lastValid->frameId;
        velocity = _ComputeVelocity(data.center, currentTime, *_lastValid);
      }

      if (distanceFromLast < _max_distance_thresh && frameGap < 100) {
        _validStreakCounter++;
        _lastValid = LastValidSample{data.center, currentTime, data.frameID};
      } else {
        std::cerr << "disqualified: distance or frame gap too large"
                  << std::endl;
        data.valid = false;
        _validStreakCounter = 0;
        _lastValid = std::nullopt;
      }
    } else {
      _validStreakCounter = 0;
    }

    validStreakForGraph = _validStreakCounter;
    _lastData = data;
    shouldPublish = data.valid && (_validStreakCounter >= MIN_VALID_STREAK);
  }

  s_validStreakGraph.AddData(static_cast<float>(validStreakForGraph));
  s_distanceFromLastGraph.AddData(static_cast<float>(distanceFromLast));
  s_frameGapGraph.AddData(static_cast<float>(frameGap));
  s_validGraph.AddData(data.valid ? 1.0f : 0.0f);
  s_timeMillisGraph.AddData(static_cast<double>(data.time_millis / 1000.0));
  s_programClockGraph.AddData(
      static_cast<float>(Clock::programClock.getElapsedTime()));


  if (true) {
    auto bbox = BBoxFromData(data);
    if (!bbox) {
      std::cerr << "CVPosition: publish path with no bbox (should not happen)"
                << std::endl;
      return;
    }
    cv::Rect bboxRect = BBoxToRectClamped(*bbox, frame.cols, frame.rows);
    OdometryData sample{};

    std::cout << "received time milliseconds: " << data.time_millis << std::endl;
    sample.pos = PositionData{data.center, velocity, bboxRect,
                              static_cast<double>(data.time_millis / 1000.0)};
    OdometryBase::Publish(sample, /*isOpponent=*/false, OdometryAlg::Neural);
    s_publishedPosXGraph.AddData(data.center.x);
    s_publishedPosYGraph.AddData(data.center.y);
  } else {
    std::cout << "not publishing" << std::endl;
  }
}

CVPositionData CVPosition::_GetDataFromPython(bool& outPythonResponded) {
  std::string data = _pythonSocket.receive();

  auto invalidFromLast = [this]() -> CVPositionData {
    CVPositionData ret;
    {
      std::lock_guard<std::mutex> lock(_lastDataMutex);
      ret = _lastData;
    }
    ret.valid = false;
    return ret;
  };

  if (data.empty()) {
    outPythonResponded = false;
    return invalidFromLast();
  }
  outPythonResponded = true;

  nlohmann::json j;
  try {
    j = nlohmann::json::parse(data);
  } catch (const std::exception& e) {
    std::cerr << "CVPosition: JSON parse error: " << e.what() << std::endl;
    return invalidFromLast();
  }

  if (!j.contains("bounding_box")) {
    std::cerr << "CVPosition: JSON missing bounding_box" << std::endl;
    return invalidFromLast();
  }
  const auto& boundingBox = j["bounding_box"];
  if (boundingBox.is_string()) {
    // "invalid" from Python: last data with valid=false, optional frame/time
    CVPositionData ret = invalidFromLast();
    ret.boundingBox = {};
    ret.frameID = 0;
    ret.time_millis = 0;
    if (j.contains("frame_id") && j["frame_id"].is_number_unsigned()) {
      ret.frameID = j["frame_id"].get<uint32_t>();
    }
    if (j.contains("time_milliseconds") &&
        j["time_milliseconds"].is_number_unsigned()) {
      ret.time_millis = j["time_milliseconds"].get<uint32_t>();
    }
    return ret;
  }
  if (!boundingBox.is_array() || boundingBox.size() != 4) {
    std::cerr << "CVPosition: bounding_box must be array of 4 numbers"
              << std::endl;
    return invalidFromLast();
  }
  for (size_t i = 0; i < 4; i++) {
    if (!boundingBox[i].is_number()) {
      std::cerr << "CVPosition: bounding_box[" << i << "] not a number"
                << std::endl;
      return invalidFromLast();
    }
  }

  if (!j.contains("conf") || !j["conf"].is_number()) {
    std::cerr << "CVPosition: JSON missing or invalid conf" << std::endl;
    return invalidFromLast();
  }
  if (!j.contains("frame_id") || !j["frame_id"].is_number_unsigned()) {
    std::cerr << "CVPosition: JSON missing or invalid frame_id" << std::endl;
    return invalidFromLast();
  }
  if (!j.contains("time_milliseconds") ||
      !j["time_milliseconds"].is_number_unsigned()) {
    std::cerr << "CVPosition: JSON missing or invalid time_milliseconds"
              << std::endl;
    return invalidFromLast();
  }

  float cx = boundingBox[0].get<float>();
  float cy = boundingBox[1].get<float>();
  float w = boundingBox[2].get<float>();
  float h = boundingBox[3].get<float>();
  CVPositionData ret;
  ret.boundingBox = {static_cast<int>(cx), static_cast<int>(cy),
                     static_cast<int>(w), static_cast<int>(h)};
  ret.frameID = j["frame_id"].get<uint32_t>();
  ret.time_millis = j["time_milliseconds"].get<uint32_t>();
  ret.valid = (j["conf"].get<float>() >= NN_MIN_CONFIDENCE);
  return ret;
}