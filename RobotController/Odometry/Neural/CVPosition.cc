#include "CVPosition.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/video/tracking.hpp>

#include "../../CameraReceiver.h"
#include "../../RobotConfig.h"

namespace {

constexpr double MAX_VELOCITY_TIME_GAP = 0.5;
constexpr int MIN_VALID_STREAK = 3;

std::optional<BBoxCxCyWh> BBoxFromData(const CVPositionData& d) {
  float cx = d.boundingBox[0], cy = d.boundingBox[1];
  float w = d.boundingBox[2], h = d.boundingBox[3];
  if (w < 0 || h < 0 || !std::isfinite(cx) || !std::isfinite(cy) ||
      !std::isfinite(w) || !std::isfinite(h))
    return std::nullopt;
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

namespace {

PyMsg ParsePyMsg(const std::string& raw) {
  PyMsg out{};
  out.type = PyMsgType::Unknown;
  try {
    auto j = nlohmann::json::parse(raw);
    if (!j.contains("type") || !j["type"].is_string()) return out;
    const std::string t = j["type"].get<std::string>();

    if (t == "ready") {
      out.type = PyMsgType::Ready;
      return out;
    }
    if (t == "frame_copied") {
      out.type = PyMsgType::FrameCopied;
      if (j.contains("frame_id") && j["frame_id"].is_number_unsigned())
        out.frame_id = j["frame_id"].get<uint32_t>();
      if (j.contains("time_milliseconds") &&
          j["time_milliseconds"].is_number_unsigned())
        out.time_ms = j["time_milliseconds"].get<uint32_t>();
      return out;
    }
    if (t == "result") {
      out.type = PyMsgType::Result;
      if (!j.contains("frame_id") || !j["frame_id"].is_number_unsigned())
        return out;
      if (!j.contains("time_milliseconds") ||
          !j["time_milliseconds"].is_number_unsigned())
        return out;
      if (!j.contains("conf") || !j["conf"].is_number()) return out;
      out.frame_id = j["frame_id"].get<uint32_t>();
      out.time_ms = j["time_milliseconds"].get<uint32_t>();

      CVPositionData data{};
      data.frameID = out.frame_id;
      data.time_millis = out.time_ms;
      data.valid = false;

      const auto& bb = j["bounding_box"];
      if (bb.is_string() && bb.get<std::string>() == "invalid") {
        data.boundingBox = {-1, -1, -1, -1};
        out.result = data;
        return out;
      }
      if (!bb.is_array() || bb.size() != 4) return out;
      for (size_t i = 0; i < 4; i++) {
        if (!bb[i].is_number()) return out;
      }
      float cx = bb[0].get<float>(), cy = bb[1].get<float>();
      float w = bb[2].get<float>(), h = bb[3].get<float>();
      if (w < 0 || h < 0) return out;
      data.boundingBox = {cx, cy, w, h};
      data.center = cv::Point2f(cx, cy);
      data.valid = (j["conf"].get<float>() >= NN_MIN_CONFIDENCE);
      out.result = data;
      return out;
    }
  } catch (const std::exception& e) {
    std::cerr << "CVPosition: parse error: " << e.what() << std::endl;
  }
  return out;
}

}  // namespace

void CVPosition::_PumpMessages(int timeout_ms) {
  std::string raw = _pythonSocket.receive_one(timeout_ms, nullptr);
  while (!raw.empty()) {
    PyMsg msg = ParsePyMsg(raw);
    if (msg.type == PyMsgType::Ready) _pythonReadySeen = true;
    if (msg.type != PyMsgType::Unknown) {
      std::lock_guard<std::mutex> lock(_mailboxMutex);
      _pendingMessages.push_back(std::move(msg));
    }
    raw = _pythonSocket.receive_one_nonblock(nullptr);
  }
}

bool CVPosition::_PopPendingFrameCopied(uint32_t frameId, PyMsg& out) {
  std::lock_guard<std::mutex> lock(_mailboxMutex);
  for (auto it = _pendingMessages.begin(); it != _pendingMessages.end(); ++it) {
    if (it->type == PyMsgType::FrameCopied && it->frame_id == frameId) {
      out = std::move(*it);
      _pendingMessages.erase(it);
      return true;
    }
  }
  return false;
}

bool CVPosition::_PopPendingResult(uint32_t frameId, CVPositionData& out) {
  std::lock_guard<std::mutex> lock(_mailboxMutex);
  for (auto it = _pendingMessages.begin(); it != _pendingMessages.end(); ++it) {
    if (it->type == PyMsgType::Result && it->result &&
        it->result->frameID == frameId) {
      out = *it->result;
      _pendingMessages.erase(it);
      return true;
    }
  }
  return false;
}

bool CVPosition::_WaitForPythonReady(int timeout_ms) {
  if (_pythonReadySeen) return true;
  const int deadline = timeout_ms;
  int elapsed = 0;
  while (elapsed < deadline) {
    _PumpMessages(std::min(100, deadline - elapsed));
    if (_pythonReadySeen) return true;
    elapsed += 100;
  }
  std::cerr << "CVPosition: timeout waiting for ready" << std::endl;
  return false;
}

bool CVPosition::_WaitForFrameCopied(uint32_t frameId, int timeout_ms) {
  const int deadline = timeout_ms;
  int elapsed = 0;
  while (elapsed < deadline) {
    PyMsg msg;
    if (_PopPendingFrameCopied(frameId, msg)) return true;
    _PumpMessages(std::min(100, deadline - elapsed));
    elapsed += 100;
  }
  std::cerr << "CVPosition: timeout waiting for frame_copied frame_id=" << frameId
            << std::endl;
  return false;
}

bool CVPosition::_GetResultForFrame(uint32_t frameId, CVPositionData& out,
                                    int timeout_ms) {
  const int deadline = timeout_ms;
  int elapsed = 0;
  while (elapsed < deadline) {
    if (_PopPendingResult(frameId, out)) return true;
    _PumpMessages(std::min(100, deadline - elapsed));
    elapsed += 100;
  }
  std::cerr << "CVPosition: timeout waiting for result frame_id=" << frameId
            << std::endl;
  return false;
}

void CVPosition::_SendFrameReady(uint32_t frameId, uint32_t timeMs) {
  nlohmann::json j;
  j["type"] = "frame_ready";
  j["frame_id"] = frameId;
  j["time_milliseconds"] = timeMs;
  _SendToPython(j.dump());
}

void CVPosition::_SendToPython(const std::string& msg) {
  if (!_pythonReadySeen) return;
  _pythonSocket.reply_to_last_sender(msg);
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
  // Placement-new the header so seq is a valid atomic; image follows
  // immediately.
  auto* header = new (_pSharedMemory) SharedImageHeader{};
  header->seq.store(0, std::memory_order_relaxed);  // init only; no reader yet
  sharedImage =
      cv::Mat(height, width, type,
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
    std::cerr << "CVPosition: invalid frame type or size" << std::endl;
    return;
  }
  const uint32_t id = _inputFrameId++;
  const double tMs = frameTime >= 0 ? frameTime * 1000.0 : 0.0;
  const uint32_t timeMillis = static_cast<uint32_t>(tMs);

  if (NEURAL_BRIGHTNESS_ADJUST != 0) {
    cv::Mat adjusted;
    frame.convertTo(adjusted, CV_16U, 1 + NEURAL_BRIGHTNESS_ADJUST / 10.0, 0);
    cv::threshold(adjusted, adjusted, 255, 255, cv::THRESH_TRUNC);
    adjusted.convertTo(frame, CV_8U);
  }

  if (!_pythonReadySeen && !_WaitForPythonReady(RECV_TIMEOUT_MS)) return;

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

  _SendFrameReady(id, timeMillis);

  if (!_WaitForFrameCopied(id, RECV_TIMEOUT_MS)) return;

  CVPositionData data{};
  if (!_GetResultForFrame(id, data, RECV_TIMEOUT_MS)) return;

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
    }
  }

  cv::Point2f velocity(0, 0);
  bool shouldPublish = false;
  double distanceFromLast = 0.0;
  uint32_t frameGap = 0;

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
        data.valid = false;
        _validStreakCounter = 0;
        _lastValid = std::nullopt;
      }
    } else {
      _validStreakCounter = 0;
    }

    _lastData = data;
    shouldPublish = data.valid && (_validStreakCounter >= MIN_VALID_STREAK);
  }

  if (!shouldPublish) return;

  auto bbox = BBoxFromData(data);
  if (!bbox) return;
  cv::Rect bboxRect = BBoxToRectClamped(*bbox, frame.cols, frame.rows);
  OdometryData sample{};
  sample.pos = PositionData{data.center, velocity, bboxRect,
                            static_cast<double>(data.time_millis / 1000.0)};
  OdometryBase::Publish(sample, /*isOpponent=*/false, OdometryAlg::Neural);
}
