#include "OpenCVTracker.h"

#include "../../RobotConfig.h"
#include "../../Globals.h"

#ifdef _OPENCV_TRACKING
// Maximum time gap (seconds) before resetting velocity
constexpr double MAX_VELOCITY_TIME_GAP = 0.5;

OpenCVTracker::OpenCVTracker(ICameraReceiver *videoSource)
    : OdometryBase(videoSource) {
  _trackerParams.model = "dasiamrpn_model.onnx";
  _trackerParams.kernel_cls1 = "dasiamrpn_kernel_cls1.onnx";
  _trackerParams.kernel_r1 = "dasiamrpn_kernel_r1.onnx";
  _trackerParams.backend = cv::dnn::DNN_BACKEND_CUDA;
  _trackerParams.target = cv::dnn::DNN_TARGET_CUDA;

  //_robotTracker = cv::TrackerDaSiamRPN::create(_trackerParams);
  //_opponentTracker = cv::TrackerDaSiamRPN::create(_trackerParams);

  _robotTracker = cv::TrackerCSRT::create();
  _opponentTracker = cv::TrackerCSRT::create();
}

void OpenCVTracker::SwitchRobots(void) {
  // Switch who's who
  std::unique_lock<std::mutex> locker(_updateMutex);

  // Swap trackers
  _robotTracker.swap(_opponentTracker);

  // Swap bounding boxes
  cv::Rect tempRect = _robotBBox;
  _robotBBox = _opponentBBox;
  _opponentBBox = tempRect;

  // Swap tracker states
  enum TrackerState tempState = _robotTrackerState;
  _robotTrackerState = _opponentTrackerState;
  _opponentTrackerState = tempState;

  // Swap last publish state
  cv::Point2f tempCenter = _robotLastCenter;
  _robotLastCenter = _opponentLastCenter;
  _opponentLastCenter = tempCenter;

  double tempTime = _robotLastTime;
  _robotLastTime = _opponentLastTime;
  _opponentLastTime = tempTime;

  bool tempHasLast = _robotHasLast;
  _robotHasLast = _opponentHasLast;
  _opponentHasLast = tempHasLast;

  // Swap epochs and increment to invalidate in-flight updates
  uint64_t tempEpoch = _robotEpoch;
  _robotEpoch = _oppEpoch + 1;  // Increment to invalidate
  _oppEpoch = tempEpoch + 1;    // Increment to invalidate
}

void OpenCVTracker::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  // tracker appears to like growing to entire robot more than shrinking to only
  // encompass it
  cv::Rect &bbox = (opponentRobot) ? _opponentBBox : _robotBBox;
  bbox.width = MIN_ROBOT_BLOB_SIZE;
  bbox.height = MIN_ROBOT_BLOB_SIZE;
  bbox.x = int(newPos.x - MIN_ROBOT_BLOB_SIZE / 2);
  bbox.y = int(newPos.y - MIN_ROBOT_BLOB_SIZE / 2);

  if (bbox.x < 0) bbox.x = 0;
  if (bbox.y < 0) bbox.y = 0;

  if (bbox.x + bbox.width > WIDTH) bbox.width = WIDTH - bbox.x;
  if (bbox.y + bbox.height > HEIGHT) bbox.height = HEIGHT - bbox.y;

  // Set tracker state to INIT_STARTED so it will be initialized on next frame
  // Increment epoch to invalidate any in-flight updates
  if (opponentRobot) {
    _opponentTrackerState = INIT_STARTED;
    _oppEpoch++;
  } else {
    _robotTrackerState = INIT_STARTED;
    _robotEpoch++;
  }
}

void OpenCVTracker::_ProcessNewFrame(cv::Mat currFrame, double frameTime) {
  // Convert frame to RGB if needed (use copyTo to avoid shallow alias issues)
  if (currFrame.channels() == 1) {
    cv::cvtColor(currFrame, _previousImage, cv::COLOR_GRAY2RGB);
  } else if (currFrame.channels() == 3) {
    currFrame.copyTo(_previousImage);
  } else if (currFrame.channels() == 4) {
    cv::cvtColor(currFrame, _previousImage, cv::COLOR_RGBA2RGB);
  }

  // Process robot tracker
  {
    std::unique_lock<std::mutex> locker(_updateMutex);

    if (_robotTrackerState == INIT_STARTED) {
      // Copy bbox and tracker pointer under lock before unlocking
      cv::Rect robotBBoxLocal = _robotBBox;
      cv::Ptr<cv::Tracker> trackerLocal = _robotTracker;
      uint64_t epochLocal = _robotEpoch;
      _robotTrackerState = INITIALIZED;
      locker.unlock();

      // Call init() outside lock (can be heavy)
      trackerLocal->init(_previousImage, robotBBoxLocal);

      // Update bbox back under lock (init may have adjusted it)
      // Verify epoch unchanged to ensure no SwitchRobots()/SetPosition()
      // occurred
      {
        std::unique_lock<std::mutex> locker2(_updateMutex);
        if (epochLocal == _robotEpoch && _robotTrackerState == INITIALIZED) {
          _robotBBox = robotBBoxLocal;
        }
      }
      // Return without publish for this frame
    } else if (_robotTrackerState == INITIALIZED) {
      // Capture epoch, bbox, and tracker pointer under lock before unlocking
      uint64_t epochLocal = _robotEpoch;
      cv::Rect robotBBoxLocal = _robotBBox;
      cv::Ptr<cv::Tracker> trackerLocal = _robotTracker;
      locker.unlock();

      // Update tracker outside lock (OpenCV operations)
      bool valid = trackerLocal->update(_previousImage, robotBBoxLocal);

      if (valid) {
        cv::Point2f center(robotBBoxLocal.x + robotBBoxLocal.width / 2.0f,
                           robotBBoxLocal.y + robotBBoxLocal.height / 2.0f);

        // Re-lock to verify epoch unchanged and compute velocity + commit state
        {
          std::unique_lock<std::mutex> locker2(_updateMutex);

          // Verify epoch unchanged and state still INITIALIZED (TOCTOU check)
          if (epochLocal != _robotEpoch || _robotTrackerState != INITIALIZED) {
            // Epoch changed or state changed - discard this update
            return;
          }

          // Compute velocity from last published position/time
          cv::Point2f velocity(0, 0);
          if (_robotHasLast) {
            double deltaTime = frameTime - _robotLastTime;
            if (deltaTime > 0 && deltaTime <= MAX_VELOCITY_TIME_GAP) {
              velocity =
                  (center - _robotLastCenter) / static_cast<float>(deltaTime);
            }
          }

          // Commit bbox and last published state
          _robotBBox = robotBBoxLocal;
          _robotLastCenter = center;
          _robotLastTime = frameTime;
          _robotHasLast = true;

          // Build sample (still under lock for consistency, but publish
          // outside)
          OdometryData sample;
          sample.Clear();
          sample.pos =
              PositionData(center, velocity, robotBBoxLocal, frameTime);

          locker2.unlock();

          // Publish outside lock (OdometryBase::Publish is internally
          // thread-safe)
          OdometryBase::Publish(sample, /*isOpponent=*/false);
        }
      }
      // If invalid, do nothing (no publish)
    }
  }

  // Process opponent tracker
  {
    std::unique_lock<std::mutex> locker(_updateMutex);

    if (_opponentTrackerState == INIT_STARTED) {
      // Copy bbox and tracker pointer under lock before unlocking
      cv::Rect opponentBBoxLocal = _opponentBBox;
      cv::Ptr<cv::Tracker> trackerLocal = _opponentTracker;
      uint64_t epochLocal = _oppEpoch;
      _opponentTrackerState = INITIALIZED;
      locker.unlock();

      // Call init() outside lock (can be heavy)
      trackerLocal->init(_previousImage, opponentBBoxLocal);

      // Update bbox back under lock (init may have adjusted it)
      // Verify epoch unchanged to ensure no SwitchRobots()/SetPosition()
      // occurred
      {
        std::unique_lock<std::mutex> locker2(_updateMutex);
        if (epochLocal == _oppEpoch && _opponentTrackerState == INITIALIZED) {
          _opponentBBox = opponentBBoxLocal;
        }
      }
      // Return without publish for this frame
    } else if (_opponentTrackerState == INITIALIZED) {
      // Capture epoch, bbox, and tracker pointer under lock before unlocking
      uint64_t epochLocal = _oppEpoch;
      cv::Rect opponentBBoxLocal = _opponentBBox;
      cv::Ptr<cv::Tracker> trackerLocal = _opponentTracker;
      locker.unlock();

      // Update tracker outside lock (OpenCV operations)
      bool valid = trackerLocal->update(_previousImage, opponentBBoxLocal);

      if (valid) {
        cv::Point2f center(
            opponentBBoxLocal.x + opponentBBoxLocal.width / 2.0f,
            opponentBBoxLocal.y + opponentBBoxLocal.height / 2.0f);

        // Re-lock to verify epoch unchanged and compute velocity + commit state
        {
          std::unique_lock<std::mutex> locker2(_updateMutex);

          // Verify epoch unchanged and state still INITIALIZED (TOCTOU check)
          if (epochLocal != _oppEpoch || _opponentTrackerState != INITIALIZED) {
            // Epoch changed or state changed - discard this update
            return;
          }

          // Compute velocity from last published position/time
          cv::Point2f velocity(0, 0);
          if (_opponentHasLast) {
            double deltaTime = frameTime - _opponentLastTime;
            if (deltaTime > 0 && deltaTime <= MAX_VELOCITY_TIME_GAP) {
              velocity = (center - _opponentLastCenter) /
                         static_cast<float>(deltaTime);
            }
          }

          // Commit bbox and last published state
          _opponentBBox = opponentBBoxLocal;
          _opponentLastCenter = center;
          _opponentLastTime = frameTime;
          _opponentHasLast = true;

          // Build sample (still under lock for consistency, but publish
          // outside)
          OdometryData sample;
          sample.Clear();
          sample.pos =
              PositionData(center, velocity, opponentBBoxLocal, frameTime);

          locker2.unlock();

          // Publish outside lock (OdometryBase::Publish is internally
          // thread-safe)
          OdometryBase::Publish(sample, /*isOpponent=*/true);
        }
      }
      // If invalid, do nothing (no publish)
    }
  }
}
#endif // _OPENCV_TRACKING