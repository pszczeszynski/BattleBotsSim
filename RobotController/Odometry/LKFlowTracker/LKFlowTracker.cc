#include "LKFlowTracker.h"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "../../Clock.h"
#include "../../MathUtils.h"

// Static member initialization
const cv::Size LKFlowTracker::LK_WIN_SIZE(15, 15);

LKFlowTracker::LKFlowTracker(ICameraReceiver* videoSource)
    : OdometryBase(videoSource),
      _roi(0, 0, 0, 0),
      _imageSize(0, 0),
      _angle(Angle(0)),
      _targetPointCount(40),
      _initialized(false),
      _lastRespawnTime(0.0),
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
}

static void DeduplicateTracks(std::vector<TrackPt>& tracks, float minDist) {
  if (tracks.size() < 2) return;

  const float minDistSq = minDist * minDist;

  // Sort for locality to reduce comparisons (x then y)
  std::sort(tracks.begin(), tracks.end(),
            [](const TrackPt& a, const TrackPt& b) {
              if (a.pt.x != b.pt.x) return a.pt.x < b.pt.x;
              return a.pt.y < b.pt.y;
            });

  std::vector<TrackPt> out;
  out.reserve(tracks.size());

  for (const auto& t : tracks) {
    bool tooClose = false;

    // Because sorted by x, only need to check recent accepted points
    // (this is a cheap heuristic; still correct-ish for small radii)
    for (int i = (int)out.size() - 1; i >= 0; --i) {
      float dx = t.pt.x - out[i].pt.x;
      if (dx * dx > minDistSq) break;  // further back will only be farther in x
      float dy = t.pt.y - out[i].pt.y;
      if (dx * dx + dy * dy < minDistSq) {
        // keep the older one
        if (t.age > out[i].age) out[i] = t;
        tooClose = true;
        break;
      }
    }

    if (!tooClose) out.push_back(t);
  }

  tracks.swap(out);
}

void LKFlowTracker::_StartCalled(void) {
  _initialized = false;
  _prevGray = cv::Mat();
  _imageSize = cv::Size(0, 0);
  _tracks.clear();
  _angle = Angle(0);
  _lastRespawnTime = 0.0;
}

cv::Rect LKFlowTracker::_ClipROIToBounds(cv::Rect roi, cv::Size bounds) {
  if (bounds.width <= 0 || bounds.height <= 0) {
    return roi;  // No valid bounds, return as-is
  }

  int x = (std::max)(0, roi.x);
  int y = (std::max)(0, roi.y);
  int w = (std::min)(roi.width, bounds.width - x);
  int h = (std::min)(roi.height, bounds.height - y);

  // Ensure width and height are non-negative
  if (w < 0) w = 0;
  if (h < 0) h = 0;

  return cv::Rect(x, y, w, h);
}

// We expect this to be called every frame from the other vision algorithms, and
// we trust it.
void LKFlowTracker::SetROI(cv::Rect roi) {
  std::unique_lock<std::mutex> locker(_updateMutex);

  // Clip ROI to image bounds
  _roi = _ClipROIToBounds(roi, _imageSize);

  // Filter out any existing points that are now outside the new ROI
  _FilterPointsByROI(_tracks);
}

void EnforceGrayscale(cv::Mat& currFrame, cv::Mat& gray) {
  if (currFrame.channels() == 1) {
    gray = currFrame;
  } else if (currFrame.channels() == 3) {
    cv::cvtColor(currFrame, gray, cv::COLOR_BGR2GRAY);
  } else if (currFrame.channels() == 4) {
    cv::cvtColor(currFrame, gray, cv::COLOR_BGRA2GRAY);
  } else {
    std::cerr << "Unsupported frame format: " << currFrame.channels()
              << std::endl;
  }
}

void LKFlowTracker::_ProcessNewFrame(cv::Mat currFrame, double frameTime) {
  // Convert to grayscale if needed
  cv::Mat gray;
  EnforceGrayscale(currFrame, gray);

  // Update image size for ROI clipping
  _imageSize = gray.size();

  // Regular interval-based respawn
  if (_roi.width > 0 && _roi.height > 0 &&
      ((frameTime - _lastRespawnTime) >= RESPAWN_INTERVAL) &&
      _RespawnPoints(gray, _roi, _tracks, _targetPointCount)) {
    _lastRespawnTime = frameTime;
  }

  // If previous image is empty, store it for next frame
  // OR if we don't have enough points, skip update and force initialization
  // next frame

  if (_prevGray.empty() || _tracks.size() < 6) {
    _prevGray = gray;
    std::cout << "No points, skipping update" << std::endl;
    return;
  }

  // Update tracking
  std::unique_lock<std::mutex> locker(_updateMutex);
  bool success = _UpdateTracking(_prevGray, gray, frameTime);

  locker.unlock();

  // Always advance _prevGray to prevent failure spiral
  // (if we keep old frame, next LK flow will have larger motion and fail more)
  _prevGray = gray;

  if (!success) {
    // Force re-initialization on next frame if tracking failed
    _initialized = false;
  }
}

bool LKFlowTracker::_InitializePoints(cv::Mat& gray, cv::Rect roi) {
  // Clear existing tracks and initialize with points from ROI
  _tracks.clear();

  // Use _RespawnPoints to find points, requesting all available points
  if (!_RespawnPoints(gray, roi, _tracks, LK_MAX_CORNERS)) {
    return false;
  }

  // Require at least 8 points for initialization
  if (_tracks.size() < 8) {
    _tracks.clear();
    return false;
  }

  // Extract points for center computation
  std::vector<cv::Point2f> pts;
  pts.reserve(_tracks.size());
  for (const auto& track : _tracks) {
    pts.push_back(track.pt);
  }

  // Reset state for initialization
  _prevCenter = _ComputeCenterFromPoints(pts);
  _angle = Angle(0);
  _initialized = true;
  _lastRespawnTime = Clock::programClock.getElapsedTime();

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

  cv::calcOpticalFlowPyrLK(
      prevGray, currGray, prevPts, nextPts, status, err, LK_WIN_SIZE,
      LK_MAX_LEVEL,
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30,
                       0.01));

  if (nextPts.empty()) {
    return false;
  }

  // Compute rotation from point pairs BEFORE compaction
  // (uses original indices that match _tracks)
  double angleDelta = 0.0;
  std::vector<std::pair<int, int>> validPairs;
  _ComputeRotationsFromPairs(nextPts, status, angleDelta, validPairs);

  // Single-pass compaction: keep only good points, increment age
  std::vector<TrackPt> tracksNext;
  tracksNext.reserve(_tracks.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] == 1) {
      tracksNext.push_back({nextPts[i], _tracks[i].age + 1});
    }
  }

  // Filter out points that are outside the ROI using standardized method
  _FilterPointsByROI(tracksNext);
  DeduplicateTracks(tracksNext, LK_MIN_DISTANCE * 0.8f);  // tune: 0.6–1.0

  // If we have too few points after tracking, fail (respawn handled in
  // _ProcessNewFrame)
  if (tracksNext.size() < 6) {
    return false;
  }

  // Extract points for center computation
  std::vector<cv::Point2f> goodNextPts;
  goodNextPts.reserve(tracksNext.size());
  for (const auto& track : tracksNext) {
    goodNextPts.push_back(track.pt);
  }

  // Compute center from good points
  cv::Point2f center = _ComputeCenterFromPoints(goodNextPts);

  // Update angle (angleDelta is already in radians)
  _angle = Angle(_angle + Angle(angleDelta));

  // Update position and velocity
  cv::Point2f deltaPos = center - _prevCenter;
  double deltaTime = frameTime - _currDataOpponent.time;

  // Update opponent data (this tracker only tracks opponent, robot data always
  // invalid)
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
  // initialize the debug image to zeros
  _debugImage = cv::Mat::zeros(currGray.size(), CV_8UC1);
  // _debugImage = currGray.clone();

  // Draw lines connecting pairs used for rotation computation
  // Note: validPairs contains indices into nextPts, so we use nextPts for
  // drawing
  for (const auto& pair : validPairs) {
    int idx1 = pair.first;
    int idx2 = pair.second;
    if (idx1 >= 0 && idx1 < static_cast<int>(nextPts.size()) && idx2 >= 0 &&
        idx2 < static_cast<int>(nextPts.size())) {
      cv::line(_debugImage, nextPts[idx1], nextPts[idx2], cv::Scalar(200), 1);
    }
  }

  for (const auto& track : _tracks) {
    cv::circle(_debugImage, track.pt, 2, cv::Scalar(255), -1);
  }
  cv::circle(_debugImage, center, 5, cv::Scalar(255), 2);
  // Draw ROI rectangle if valid
  if (_roi.width > 0 && _roi.height > 0) {
    cv::rectangle(_debugImage, _roi, cv::Scalar(128), 2);
  }
  debugLock.unlock();

  return true;
}

void LKFlowTracker::_ComputeRotationsFromPairs(
    const std::vector<cv::Point2f>& nextPts, const std::vector<uchar>& status,
    double& angleDelta, std::vector<std::pair<int, int>>& validPairs) {
  std::vector<RotationResult> rotations;
  validPairs.clear();  // Clear output parameter

  // Note: This is called BEFORE compaction, so _tracks.size() should match
  // nextPts.size()
  if (_tracks.size() != nextPts.size() || _tracks.size() < 2) {
    angleDelta = 0.0;
    return;
  }

  // Generate point pairs on-the-fly (no need to store them)
  std::vector<std::pair<int, int>> pairs =
      _GeneratePointPairs(static_cast<int>(_tracks.size()));

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
    // Store this pair as valid for debug drawing
    validPairs.push_back(pair);
  }

  _UpdateAngleFromRotations(rotations, angleDelta);
}

void LKFlowTracker::_UpdateAngleFromRotations(
    const std::vector<RotationResult>& rotations, double& angleDelta) {
  if (rotations.size() < 3) {
    angleDelta = 0.0;
    return;
  }

  // Extract angle deltas
  std::vector<float> angles;
  angles.reserve(rotations.size());
  for (const auto& r : rotations) {
    angles.push_back(r.angleDelta);
  }
  if (angles.empty()) {
    angleDelta = 0.0;
    return;
  }

  // Sort to compute median (sign decision)
  std::sort(angles.begin(), angles.end());

  const size_t n = angles.size();
  const size_t mid = n / 2;

  // Median (for even n, average the two middle values)
  float median = 0.0f;

  if ((n & 1u) == 1u) {
    median = angles[mid];
  } else {
    median = 0.5f * (angles[mid - 1] + angles[mid]);
  }

  // Decide sign from median; treat 0 as "no rotation"
  if (std::abs(median) < 1e-6f) {
    angleDelta = 0.0;
    return;
  }
  const bool wantPositive = (median > 0.0f);

  // Filter to angles with the chosen sign (ignore zeros)
  std::vector<float> sameSign;
  sameSign.reserve(n);
  for (float a : angles) {
    if (wantPositive) {
      if (a > 0.0f) sameSign.push_back(a);
    } else {
      if (a < 0.0f) sameSign.push_back(-a);
    }
  }

  // If filtering leaves too few, fall back to median
  if (sameSign.size() < 3) {
    angleDelta = static_cast<double>(median);
    return;
  }

  // Sort filtered set and take 75th percentile *within that sign*
  std::sort(sameSign.begin(), sameSign.end());
  const size_t m = sameSign.size();
  const size_t p75 = static_cast<size_t>(std::floor(0.66 * (m - 1)));

  angleDelta = static_cast<double>(sameSign[p75]);

  if (wantPositive) {
    angleDelta = angleDelta;
  } else {
    angleDelta = -angleDelta;
  }
}

bool LKFlowTracker::_RespawnPoints(const cv::Mat& gray, cv::Rect roi,
                                   std::vector<TrackPt>& tracks,
                                   int targetCount) {
  // Calculate how many points we need to reach the target
  const int needed = targetCount - tracks.size();

  // If we already have enough points, no need to respawn
  if (needed <= 0) {
    std::cout << "Already have enough points, no need to respawn" << std::endl;
    return true;  // Success (no action needed)
  }

  // Request more points than needed since some may be filtered out for being
  // too close to existing points
  int maxToRequest = (std::min)(needed * 2, LK_MAX_CORNERS);

  // Clip ROI to image bounds (safety check in case image size changed)
  cv::Rect validROI = _ClipROIToBounds(roi, gray.size());

  if (validROI.width <= 0 || validROI.height <= 0) {
    return false;
  }
  cv::Mat roiGray = gray(validROI);

  std::vector<cv::Point2f> newPts;
  cv::goodFeaturesToTrack(roiGray, newPts, maxToRequest, LK_QUALITY_LEVEL,
                          LK_MIN_DISTANCE);

  if (newPts.empty()) {
    return false;
  }

  // Shift to full-image coords
  for (auto& pt : newPts) {
    pt.x += validROI.x;
    pt.y += validROI.y;
  }

  // Filter new points: only add those that are sufficiently far from existing
  // tracks
  const double minDistSq =
      LK_MIN_DISTANCE * LK_MIN_DISTANCE;  // Use squared distance for efficiency
  int added = 0;
  for (const auto& newPt : newPts) {
    if (added >= needed) {
      break;  // We've added enough points
    }

    // Check distance to all existing tracks
    bool tooClose = false;
    for (const auto& track : tracks) {
      cv::Point2f diff = newPt - track.pt;
      double distSq = diff.x * diff.x + diff.y * diff.y;
      if (distSq < minDistSq) {
        tooClose = true;
        break;
      }
    }

    // Only add if sufficiently far from all existing points
    if (!tooClose) {
      tracks.push_back({newPt, 0});  // Initialize with age 0
      added++;
    }
  }

  std::cout << "Respawned " << added << " points (filtered from "
            << newPts.size() << " candidates) for a total of " << tracks.size()
            << " points" << std::endl;

  // Return true if we added at least some points (even if fewer than requested)
  return added > 0;
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

std::vector<std::pair<int, int>> LKFlowTracker::_GeneratePointPairs(int nPts) {
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

void LKFlowTracker::_FilterPointsByROI(std::vector<TrackPt>& tracks) {
  // If ROI is invalid (width or height <= 0), keep all points
  if (_roi.width <= 0 || _roi.height <= 0) {
    return;
  }

  // Remove points that are outside the ROI
  tracks.erase(
      std::remove_if(
          tracks.begin(), tracks.end(),
          [this](const TrackPt& track) {
            // Check ROI bounds
            if (track.pt.x < _roi.x || track.pt.x >= _roi.x + _roi.width ||
                track.pt.y < _roi.y || track.pt.y >= _roi.y + _roi.height) {
              return true;
            }

            return false;
          }),
      tracks.end());
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
