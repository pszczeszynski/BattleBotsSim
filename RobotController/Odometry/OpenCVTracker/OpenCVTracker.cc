#include "OpenCVTracker.h"

#include "../../Globals.h"
#include "../../RobotConfig.h"

constexpr double MAX_VELOCITY_TIME_GAP = 0.5;
constexpr int INVALID_RECOVERY_THRESHOLD = 10;

OpenCVTracker::OpenCVTracker(ICameraReceiver* videoSource)
    : OdometryBase(videoSource) {
  _trackerParams.model = "dasiamrpn_model.onnx";
  _trackerParams.kernel_cls1 = "dasiamrpn_kernel_cls1.onnx";
  _trackerParams.kernel_r1 = "dasiamrpn_kernel_r1.onnx";
  _trackerParams.backend = cv::dnn::DNN_BACKEND_CUDA;
  _trackerParams.target = cv::dnn::DNN_TARGET_CUDA;

  _robotTracker = cv::TrackerCSRT::create();
  _opponentTracker = cv::TrackerCSRT::create();
}

void OpenCVTracker::SwitchRobots(void) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  _robotTracker.swap(_opponentTracker);

  cv::Rect tempRect = _robotBBox;
  _robotBBox = _opponentBBox;
  _opponentBBox = tempRect;

  TrackerState tempState = _robotTrackerState;
  _robotTrackerState = _opponentTrackerState;
  _opponentTrackerState = tempState;

  cv::Point2f tempCenter = _robotLastCenter;
  _robotLastCenter = _opponentLastCenter;
  _opponentLastCenter = tempCenter;

  double tempTime = _robotLastTime;
  _robotLastTime = _opponentLastTime;
  _opponentLastTime = tempTime;

  bool tempHasLast = _robotHasLast;
  _robotHasLast = _opponentHasLast;
  _opponentHasLast = tempHasLast;

  int tempInvalid = _robotInvalidCount;
  _robotInvalidCount = _opponentInvalidCount;
  _opponentInvalidCount = tempInvalid;

  // Swap epochs, then increment both to invalidate in-flight updates
  uint64_t tempEpoch = _robotEpoch;
  _robotEpoch = _oppEpoch;
  _oppEpoch = tempEpoch;
  _robotEpoch++;
  _oppEpoch++;

  // Reset velocity state for any slot that became INIT_STARTED (prevents spikes)
  if (_robotTrackerState == INIT_STARTED) {
    _robotHasLast = false;
    _robotLastTime = 0.0;
    _robotInvalidCount = 0;
  }
  if (_opponentTrackerState == INIT_STARTED) {
    _opponentHasLast = false;
    _opponentLastTime = 0.0;
    _opponentInvalidCount = 0;
  }
}

void OpenCVTracker::SetPosition(const PositionData& newPos, bool opponentRobot) {
  if (newPos.algorithm == OdometryAlg::OpenCV) {
    return;
  }

  std::unique_lock<std::mutex> locker(_updateMutex);

  cv::Point2f newPosPt = newPos.position;
  cv::Rect& bbox = opponentRobot ? _opponentBBox : _robotBBox;
  bbox.width = MIN_ROBOT_BLOB_SIZE;
  bbox.height = MIN_ROBOT_BLOB_SIZE;
  bbox.x = int(newPosPt.x - MIN_ROBOT_BLOB_SIZE / 2);
  bbox.y = int(newPosPt.y - MIN_ROBOT_BLOB_SIZE / 2);

  if (bbox.x < 0) bbox.x = 0;
  if (bbox.y < 0) bbox.y = 0;
  if (bbox.x + bbox.width > WIDTH) bbox.width = WIDTH - bbox.x;
  if (bbox.y + bbox.height > HEIGHT) bbox.height = HEIGHT - bbox.y;

  if (opponentRobot) {
    _opponentTrackerState = INIT_STARTED;
    _oppEpoch++;
    _opponentHasLast = false;
    _opponentLastTime = 0.0;
    _opponentInvalidCount = 0;
  } else {
    _robotTrackerState = INIT_STARTED;
    _robotEpoch++;
    _robotHasLast = false;
    _robotLastTime = 0.0;
    _robotInvalidCount = 0;
  }
}

void OpenCVTracker::_ProcessSlot(cv::Ptr<cv::Tracker>& tracker, cv::Rect& bbox,
                                 TrackerState& state, uint64_t& epoch,
                                 cv::Point2f& lastCenter, double& lastTime,
                                 bool& hasLast, int& invalidCount,
                                 bool isOpponent, double frameTime) {
  {
    std::unique_lock<std::mutex> locker(_updateMutex);
    if (state == INIT_STARTED) {
      cv::Rect bboxLocal = bbox;
      cv::Ptr<cv::Tracker> trackerLocal = tracker;
      uint64_t epochLocal = epoch;
      state = INITIALIZED;
      hasLast = false;
      lastTime = 0.0;
      invalidCount = 0;
      locker.unlock();

      trackerLocal->init(_previousImage, bboxLocal);

      locker.lock();
      if (epochLocal == epoch && state == INITIALIZED) {
        bbox = bboxLocal;
      }
      return;
    }
    if (state != INITIALIZED) {
      return;
    }
  }

  uint64_t epochLocal;
  cv::Rect bboxLocal;
  cv::Ptr<cv::Tracker> trackerLocal;
  {
    std::unique_lock<std::mutex> locker(_updateMutex);
    epochLocal = epoch;
    bboxLocal = bbox;
    trackerLocal = tracker;
  }

  bool valid = trackerLocal->update(_previousImage, bboxLocal);

  if (!valid) {
    invalidCount++;
    if (invalidCount >= INVALID_RECOVERY_THRESHOLD) {
      std::unique_lock<std::mutex> locker(_updateMutex);
      if (state == INITIALIZED && epochLocal == epoch) {
        state = INIT_STARTED;
        bbox = bboxLocal;
        hasLast = false;
        lastTime = 0.0;
        invalidCount = 0;
      }
    }
    return;
  }

  invalidCount = 0;
  cv::Point2f center(bboxLocal.x + bboxLocal.width / 2.0f,
                     bboxLocal.y + bboxLocal.height / 2.0f);

  cv::Point2f velocity(0, 0);
  {
    std::unique_lock<std::mutex> locker(_updateMutex);
    if (epochLocal != epoch || state != INITIALIZED) {
      return;  // Skip publish for this slot only; continue to other tracker
    }
    if (hasLast) {
      double deltaTime = frameTime - lastTime;
      if (deltaTime > 0 && deltaTime <= MAX_VELOCITY_TIME_GAP) {
        velocity = (center - lastCenter) / static_cast<float>(deltaTime);
      }
    }
    bbox = bboxLocal;
    lastCenter = center;
    lastTime = frameTime;
    hasLast = true;
  }

  OdometryData sample;
  sample.Clear();
  sample.pos = PositionData(center, velocity, bboxLocal, frameTime);
  OdometryBase::Publish(sample, isOpponent, OdometryAlg::OpenCV);
}

void OpenCVTracker::_ProcessNewFrame(cv::Mat currFrame, double frameTime) {
  if (currFrame.channels() == 1) {
    cv::cvtColor(currFrame, _previousImage, cv::COLOR_GRAY2RGB);
  } else if (currFrame.channels() == 3) {
    currFrame.copyTo(_previousImage);
  } else if (currFrame.channels() == 4) {
    cv::cvtColor(currFrame, _previousImage, cv::COLOR_RGBA2RGB);
  }

  _ProcessSlot(_robotTracker, _robotBBox, _robotTrackerState, _robotEpoch,
              _robotLastCenter, _robotLastTime, _robotHasLast,
              _robotInvalidCount, false, frameTime);

  _ProcessSlot(_opponentTracker, _opponentBBox, _opponentTrackerState,
              _oppEpoch, _opponentLastCenter, _opponentLastTime,
              _opponentHasLast, _opponentInvalidCount, true, frameTime);
}
