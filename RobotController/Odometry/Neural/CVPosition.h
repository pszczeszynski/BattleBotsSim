#pragma once
#include <windows.h>

#include <cstdint>
#include <atomic>
#include <mutex>
#include <optional>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "../../ServerSocket.h"
#include "../OdometryBase.h"

// Shared-memory header for C++ writer / Python reader. Layout: [seq:4][frame_id:4][time_ms:4].
// Seqlock invariant: seq even => snapshot is complete; seq odd => writer is updating.
// Only seq is accessed atomically (release when publishing); fixes torn reads and backward timestamps.
struct alignas(4) SharedImageHeader {
  std::atomic<uint32_t> seq;
  uint32_t frame_id;
  uint32_t time_ms;
};
static_assert(sizeof(SharedImageHeader) == 12, "SharedImageHeader must be 12 bytes");
static_assert(alignof(SharedImageHeader) >= 4, "SharedImageHeader must be 4-byte aligned");

struct CVPositionData {
  std::vector<int> boundingBox;  // (cx, cy, w, h) in pixels when valid
  cv::Point2f center;
  uint32_t frameID;
  uint32_t time_millis;
  bool valid;
};

// Bbox format from neural net: [center_x, center_y, width, height] in pixels.
struct BBoxCxCyWh {
  float cx, cy, w, h;
};

// Last valid sample used for velocity and gating (only from valid frames).
struct LastValidSample {
  cv::Point2f center;
  double t;
  uint32_t frameId;
};

// --- Unit-testable helpers (declared here for tests; defined in .cc) ---
// Parse bbox from 4 floats; returns nullopt if w or h < 0.
std::optional<BBoxCxCyWh> ParseBBoxCxCyWh(float cx, float cy, float w, float h);

// Clamp rect (cx,cy,w,h) to frame [0,frameW) x [0,frameH); width/height >= 1.
// Returns (x, y, width, height) for OpenCV rect (x,y = top-left).
void ClampRectToFrame(float cx, float cy, float w, float h, int frameWidth,
                      int frameHeight, int& outX, int& outY, int& outW,
                      int& outH);

class CVPosition : public OdometryBase {
 public:
  CVPosition(ICameraReceiver* videoSource);
  ~CVPosition();

  void SwitchRobots(void) override {};  // Dont do anything for SwitchRobots

 private:
  void _ProcessNewFrame(cv::Mat frame, double frameTime)
      override;  // Run every time a new frame is available
  std::vector<std::string> classes{"orbitron"};

  cv::Size modelShape{};

  // Latest data from Python (protected by _lastDataMutex)
  CVPositionData _lastData;
  std::mutex _lastDataMutex;
  std::optional<LastValidSample> _lastValid;
  int _validStreakCounter = 0;  // Consecutive valid frames
  int _max_distance_thresh = 100;

  CVPositionData _GetDataFromPython(bool& outPythonResponded);

  // Shared memory management
  std::string memName = "cv_pos_img";
  int width = 720;
  int height = 720;
  HANDLE _hMapFile = nullptr;      // Shared memory mapping handle
  void* _pSharedMemory = nullptr;  // Mapped memory pointer
  cv::Mat sharedImage;

  void _InitSharedImage();
  void _StartPython();
  std::thread _pythonThread;
  std::atomic<bool> _pythonThreadRunning{false};

  ServerSocket _pythonSocket;

  uint32_t _inputFrameId = 0;

  // Compute velocity from previous valid state only; (0,0) if deltaTime <= 0
  // or > MAX_VELOCITY_TIME_GAP.
  cv::Point2f _ComputeVelocity(cv::Point2f currentCenter, double currentTime,
                               const LastValidSample& last) const;
};
