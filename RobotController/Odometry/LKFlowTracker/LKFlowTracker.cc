#include "LKFlowTracker.h"

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <cmath>

#include "../../Clock.h"
#include "../../MathUtils.h"


// Static member initialization
const cv::Size LKFlowTracker::LK_WIN_SIZE(21, 21);

LKFlowTracker::LKFlowTracker(ICameraReceiver* videoSource)
    : OdometryBase(videoSource),
      _roi(0, 0, 0, 0),
      _angle(Angle(0)),
      _targetPointCount(0),
      _initialized(false),
      _rng(std::random_device{}()) {
  // This tracker only tracks opponent, robot data always invalid
  SetAngle(Angle(0), false, 0, 0, false);
  SetAngle(Angle(0), true, 0, 0, false);
  SetPosition(cv::Point2f(0, 0), false);  // Robot position invalid
  SetPosition(cv::Point2f(0, 0), true);
  SetVelocity(cv::Point2f(0, 0), false);
  SetVelocity(cv::Point2f(0, 0), true);
  // Ensure robot data is invalid
  _currDataRobot.robotPosValid = false;
  _currDataRobot.InvalidateAngle();
  _lastRespawnTime.markStart();
}

void LKFlowTracker::_StartCalled(void) {
  _initialized = false;
  _prevGray = cv::Mat();
  _tracks.clear();
  _angle = Angle(0);
  _lastRespawnTime.markStart();
}

void LKFlowTracker::SetROI(cv::Rect roi) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  _roi = roi;
  // Re-initialize points if we have a valid ROI and previous frame
  if (!_prevGray.empty() && roi.width > 0 && roi.height > 0) {
    _InitializePoints(_prevGray, roi);
  }
}

void LKFlowTracker::_ProcessNewFrame(cv::Mat currFrame, double frameTime) {
  // Convert to grayscale if needed
  cv::Mat gray;
  if (currFrame.channels() == 1) {
    gray = currFrame;
  } else if (currFrame.channels() == 3) {
    cv::cvtColor(currFrame, gray, cv::COLOR_BGR2GRAY);
  } else if (currFrame.channels() == 4) {
    cv::cvtColor(currFrame, gray, cv::COLOR_BGRA2GRAY);
  } else {
    return;  // Unsupported format
  }

  // If previous image is empty, initialize it to the current frame
  if (_prevGray.empty()) {
    _prevGray = gray;
    if (_roi.width > 0 && _roi.height > 0) {
      _InitializePoints(gray, _roi);
    }
    return;
  }

  // If not initialized and we have a valid ROI, try to initialize
  if (!_initialized && _roi.width > 0 && _roi.height > 0) {
    if (_InitializePoints(gray, _roi)) {
      _initialized = true;
    }
    _prevGray = gray;
    return;
  }

  // If we don't have enough points, skip update
  if (_tracks.size() < 6) {
    _prevGray = gray;
    return;
  }

  // Update tracking
  std::unique_lock<std::mutex> locker(_updateMutex);
  bool success = _UpdateTracking(_prevGray, gray, frameTime);
  locker.unlock();

  if (success) {
    _prevGray = gray;
  }
}

bool LKFlowTracker::_InitializePoints(cv::Mat& gray, cv::Rect roi) {
  // Ensure ROI is within image bounds
  int imgH = gray.rows;
  int imgW = gray.cols;

  int x = (std::max)(0, roi.x);
  int y = (std::max)(0, roi.y);
  int w = (std::min)(roi.width, imgW - x);
  int h = (std::min)(roi.height, imgH - y);

  if (w <= 0 || h <= 0 || x + w > imgW || y + h > imgH) {
    return false;
  }

  cv::Rect validROI(x, y, w, h);
  cv::Mat roiGray = gray(validROI);

  // Find good features to track
  std::vector<cv::Point2f> pts;
  cv::goodFeaturesToTrack(roiGray, pts, LK_MAX_CORNERS, LK_QUALITY_LEVEL,
                          LK_MIN_DISTANCE);

  if (pts.size() < 8) {
    return false;
  }

  // Shift to full-image coordinates and create TrackPt structs
  _tracks.clear();
  _tracks.reserve(pts.size());
  for (auto& pt : pts) {
    pt.x += x;
    pt.y += y;
    _tracks.push_back({pt, 0});  // Initialize with age 0
  }

  _prevCenter = _ComputeCenterFromPoints(pts);
  _angle = Angle(0);
  _targetPointCount = 40;  // Set target once during initialization
  _lastRespawnTime.markStart();
  _initialized = true;

  return true;
}

bool LKFlowTracker::_UpdateTracking(cv::Mat& prevGray, cv::Mat& currGray,
                                    double frameTime) {
  // Extract points from tracks for optical flow
  std::vector<cv::Point2f> prevPts;
  prevPts.reserve(_tracks.size());
  for (const auto& track : _tracks) {
    prevPts.push_back(track.pt);
  }

  // Calculate optical flow
  std::vector<cv::Point2f> nextPts;
  std::vector<uchar> status;
  std::vector<float> err;

  cv::calcOpticalFlowPyrLK(prevGray, currGray, prevPts, nextPts, status, err,
                           LK_WIN_SIZE, LK_MAX_LEVEL,
                           cv::TermCriteria(cv::TermCriteria::COUNT |
                                                cv::TermCriteria::EPS,
                                            30, 0.01));

  if (nextPts.empty()) {
    return false;
  }

  // Compute rotation from point pairs BEFORE compaction
  // (uses original indices that match _tracks)
  double angleDelta = 0.0;
  _ComputeRotationsFromPairs(nextPts, status, angleDelta);

  // Single-pass compaction: keep only good points, increment age
  std::vector<TrackPt> tracksNext;
  tracksNext.reserve(_tracks.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] == 1) {
      tracksNext.push_back({nextPts[i], _tracks[i].age + 1});
    }
  }

  if (tracksNext.size() < 6) {
    return false;
  }

  // Extract points for center computation and respawn
  std::vector<cv::Point2f> goodNextPts;
  goodNextPts.reserve(tracksNext.size());
  for (const auto& track : tracksNext) {
    goodNextPts.push_back(track.pt);
  }

  // Compute center from good points
  cv::Point2f center = _ComputeCenterFromPoints(goodNextPts);

  // Check if we need to respawn points (use compacted size)
  // Respawn when we've lost too many points (below threshold ratio of target)
  double elapsed = _lastRespawnTime.getElapsedTime();
  int respawnThreshold = static_cast<int>(_targetPointCount * RESPAWN_THRESHOLD_RATIO);
  if (elapsed >= RESPAWN_INTERVAL &&
      static_cast<int>(goodNextPts.size()) < respawnThreshold) {
    cv::Size roiSize;
    if (_roi.width > 0 && _roi.height > 0) {
      roiSize = cv::Size(_roi.width, _roi.height);
    } else {
      // Estimate ROI size from point spread
      if (!goodNextPts.empty()) {
        float xMin = goodNextPts[0].x, xMax = goodNextPts[0].x;
        float yMin = goodNextPts[0].y, yMax = goodNextPts[0].y;
        for (const auto& pt : goodNextPts) {
          xMin = (std::min)(xMin, pt.x);
          xMax = (std::max)(xMax, pt.x);
          yMin = (std::min)(yMin, pt.y);
          yMax = (std::max)(yMax, pt.y);
        }
        roiSize = cv::Size(static_cast<int>(xMax - xMin) + 20,
                           static_cast<int>(yMax - yMin) + 20);
      } else {
        roiSize = cv::Size(140, 140);
      }
    }

    if (_RespawnPoints(currGray, center, roiSize, tracksNext,
                       _targetPointCount)) {
      _lastRespawnTime.markStart();
      // Note: _targetPointCount remains fixed at the initial value set during _InitializePoints()
      // This ensures respawn triggers consistently when we drop below half of the target
    }
  }

  // Update angle (angleDelta is already in radians)
  _angle = Angle(_angle + Angle(angleDelta));

  // Update position and velocity
  cv::Point2f deltaPos = center - _prevCenter;
  double deltaTime = frameTime - _currDataRobot.time;

  // Update opponent data (this tracker only tracks opponent, robot data always invalid)
  _currDataOpponent.id++;
  _currDataOpponent.frameID = frameID;
  _currDataOpponent.time = frameTime;
  _currDataOpponent.robotPosValid = true;
  _currDataOpponent.robotPosition = center;

  if (deltaTime > 0.001 && _prevDataOpponent.robotPosValid) {
    cv::Point2f visualVelocity = deltaPos / static_cast<float>(deltaTime);
    _currDataOpponent.robotVelocity = visualVelocity;
  } else {
    _currDataOpponent.robotVelocity = cv::Point2f(0, 0);
  }

   // Update angle (angleDelta is already in radians)
   double angularVelocity = 0.0;
   if (deltaTime > 0.001 && _prevAngleDataOpponent.IsAngleValid()) {
     angularVelocity = angleDelta / deltaTime;
   }
   _currDataOpponent.SetAngle(_angle, angularVelocity, frameTime, true);

  // Always invalidate robot data (this tracker only tracks opponent)
  _currDataRobot.robotPosValid = false;
  _currDataRobot.InvalidateAngle();

  _prevDataOpponent = _currDataOpponent;
  _prevAngleDataOpponent = _currDataOpponent;
  _prevCenter = center;

  // Update tracks with compacted results
  _tracks = tracksNext;

  // Update debug image
  std::unique_lock<std::mutex> debugLock(_mutexDebugImage);
  _debugImage = currGray.clone();
  for (const auto& track : _tracks) {
    cv::circle(_debugImage, track.pt, 2, cv::Scalar(255), -1);
  }
  cv::circle(_debugImage, center, 5, cv::Scalar(255), 2);
  debugLock.unlock();

  return true;
}

void LKFlowTracker::_ComputeRotationsFromPairs(
    const std::vector<cv::Point2f>& nextPts, const std::vector<uchar>& status,
    double& angleDelta) {
  std::vector<RotationResult> rotations;

  // Note: This is called BEFORE compaction, so _tracks.size() should match nextPts.size()
  if (_tracks.size() != nextPts.size() || _tracks.size() < 2) {
    angleDelta = 0.0;
    return;
  }

  // Generate point pairs on-the-fly (no need to store them)
  std::vector<std::pair<int, int>> pairs = _GeneratePointPairs(static_cast<int>(_tracks.size()));

  for (const auto& pair : pairs) {
    int idx1 = pair.first;
    int idx2 = pair.second;

    if (idx1 >= static_cast<int>(_tracks.size()) ||
        idx2 >= static_cast<int>(_tracks.size()) ||
        idx1 >= static_cast<int>(status.size()) ||
        idx2 >= static_cast<int>(status.size()) || status[idx1] != 1 ||
        status[idx2] != 1) {
      continue;
    }

    // Check that both points have been tracked for at least LK_MIN_TRACK_FRAMES
    if (_tracks[idx1].age < LK_MIN_TRACK_FRAMES ||
        _tracks[idx2].age < LK_MIN_TRACK_FRAMES) {
      continue;
    }

    // Get points in prev and next frames
    cv::Point2f p1Prev = _tracks[idx1].pt;
    cv::Point2f p2Prev = _tracks[idx2].pt;
    cv::Point2f p1Next = nextPts[idx1];
    cv::Point2f p2Next = nextPts[idx2];

    // Compute vectors between points
    cv::Point2f vecPrev = p2Prev - p1Prev;
    cv::Point2f vecNext = p2Next - p1Next;

    // Skip if vectors are too short (unreliable angle)
    float normPrev = cv::norm(vecPrev);
    float normNext = cv::norm(vecNext);
    if (normPrev < 3.0f || normNext < 3.0f) {
      continue;
    }

     // Compute angles (in radians)
     float anglePrev = static_cast<float>(std::atan2(vecPrev.y, vecPrev.x));
     float angleNext = static_cast<float>(std::atan2(vecNext.y, vecNext.x));
     float distance = cv::norm(p2Next - p1Next);
     float rot = angle_wrap(angleNext - anglePrev);

     rotations.push_back(RotationResult{rot, distance});
  }

  _UpdateAngleFromRotations(rotations, angleDelta);
}

void LKFlowTracker::_UpdateAngleFromRotations(
    const std::vector<RotationResult>& rotations, double& angleDelta) {
  if (rotations.size() < 3) {
    angleDelta = 0.0;
    return;
  }

  // Weighted circular mean: more robust for circular data than percentile
  // Weight by distance (or distance^2) - longer baselines are more stable
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  const float minWeight = 1.0f;   // Minimum weight to avoid division issues
  const float maxWeight = 100.0f; // Cap weight to avoid outliers dominating

  for (const auto& r : rotations) {
    // Weight by distance squared (longer baselines = more stable angles)
    // Clamp to avoid extreme weights
    float weight = (std::max)(minWeight, (std::min)(maxWeight, r.distance * r.distance));
    
    sumSin += weight * std::sin(r.angleDelta);
    sumCos += weight * std::cos(r.angleDelta);
  }

  // Compute weighted circular mean
  angleDelta = static_cast<double>(std::atan2(sumSin, sumCos));
}

bool LKFlowTracker::_RespawnPoints(const cv::Mat& gray, cv::Point2f center,
                                   cv::Size roiSize,
                                   std::vector<TrackPt>& tracks,
                                   int targetCount) {
  if (center.x < 0 || center.y < 0) {
    return false;
  }

  // Calculate how many points we need to reach the target
  int currentCount = static_cast<int>(tracks.size());
  int needed = targetCount - currentCount;
  
  // If we already have enough points, no need to respawn
  if (needed <= 0) {
    return true;  // Success (no action needed)
  }

  // Cap the request to avoid exceeding target (goodFeaturesToTrack may return more)
  int maxToRequest = (std::min)(needed, LK_MAX_CORNERS);

  // Create a ROI around the center
  int w = roiSize.width;
  int h = roiSize.height;
  int x =
      (std::max)(0, static_cast<int>(center.x - static_cast<float>(w) / 2.0f));
  int y =
      (std::max)(0, static_cast<int>(center.y - static_cast<float>(h) / 2.0f));

  // Ensure ROI is within image bounds
  int imgH = gray.rows;
  int imgW = gray.cols;
  x = (std::min)(x, imgW - w);
  y = (std::min)(y, imgH - h);

  if (x < 0 || y < 0 || x + w > imgW || y + h > imgH) {
    return false;
  }

  cv::Rect spawnROI(x, y, w, h);
  cv::Mat roiGray = gray(spawnROI);

  std::vector<cv::Point2f> newPts;
  cv::goodFeaturesToTrack(roiGray, newPts, maxToRequest, LK_QUALITY_LEVEL,
                          LK_MIN_DISTANCE);

  // Require at least a few points to be useful, but don't fail if we get fewer than requested
  if (newPts.empty()) {
    return false;
  }

  // Shift to full-image coords and create TrackPt structs
  int toAdd = (std::min)(static_cast<int>(newPts.size()), needed);
  for (int i = 0; i < toAdd; ++i) {
    newPts[i].x += x;
    newPts[i].y += y;
    tracks.push_back({newPts[i], 0});  // Initialize with age 0
  }

  return true;
}

cv::Point2f LKFlowTracker::_ComputeCenterFromPoints(
    const std::vector<cv::Point2f>& points) {
  if (points.empty()) {
    return cv::Point2f(0, 0);
  }

  cv::Point2f sum(0, 0);
  for (const auto& pt : points) {
    sum += pt;
  }
   return sum / static_cast<float>(points.size());
 }

 std::vector<std::pair<int, int>> LKFlowTracker::_GeneratePointPairs(
    int nPts) {
  std::vector<std::pair<int, int>> pairs;
  if (nPts < 2) {
    return pairs;
  }

  // Generate all possible pairs
  std::vector<std::pair<int, int>> allPairs;
  for (int i = 0; i < nPts; ++i) {
    for (int j = i + 1; j < nPts; ++j) {
      allPairs.push_back({i, j});
    }
  }

  // Select a random subset if we have more pairs than needed
  int numPairs = (std::min)(LK_NUM_PAIRS, static_cast<int>(allPairs.size()));
  if (static_cast<int>(allPairs.size()) > numPairs) {
    std::shuffle(allPairs.begin(), allPairs.end(), _rng);
    pairs.assign(allPairs.begin(), allPairs.begin() + numPairs);
  } else {
    pairs = allPairs;
  }

  return pairs;
}

void LKFlowTracker::SwitchRobots(void) {
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData temp_Robot = _currDataRobot;
  _currDataRobot = _currDataOpponent;
  _currDataOpponent = temp_Robot;

  _currDataRobot.isUs = true;
  _currDataOpponent.isUs = false;

  temp_Robot = _prevDataRobot;
  _prevDataRobot = _prevDataOpponent;
  _prevDataOpponent = temp_Robot;

  _prevDataRobot.isUs = true;
  _prevDataOpponent.isUs = false;
}

void LKFlowTracker::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
  odoData.robotPosition = newPos;
  odoData.robotVelocity = cv::Point2f(0, 0);
  odoData.robotPosValid = true;
  odoData.time = Clock::programClock.getElapsedTime();
  odoData.id++;

  OdometryData& prevData = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
  prevData.robotPosition = newPos;
  prevData.robotVelocity = cv::Point2f(0, 0);
  prevData.robotPosValid = false;
}

void LKFlowTracker::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
  odoData.robotVelocity = newVel;
  odoData.id++;

  OdometryData& odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
  odoData2.robotVelocity = newVel;
}

void LKFlowTracker::SetAngle(Angle newAngle, bool opponentRobot,
                             double angleFrameTime, double newAngleVelocity,
                             bool valid) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
  odoData.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
  odoData.id++;

  OdometryData& odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
  odoData2.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);

  OdometryData& odoData3 =
      (opponentRobot) ? _prevAngleDataOpponent : _prevAngleDataRobot;
  odoData3.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
}

void LKFlowTracker::GetDebugImage(cv::Mat& debugImage, cv::Point offset) {
  OdometryBase::GetDebugImage(debugImage, offset);

  std::unique_lock<std::mutex> locker(_mutexDebugImage);
  if (_debugImage.empty() || _debugImage.size() != debugImage.size() ||
      _debugImage.type() != debugImage.type()) {
    locker.unlock();
    return;
  }

  cv::add(debugImage, _debugImage, debugImage);
  locker.unlock();
}
