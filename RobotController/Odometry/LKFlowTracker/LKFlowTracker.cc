#include "LKFlowTracker.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <opencv2/imgproc.hpp>

#include "../../Clock.h"
#include "../../MathUtils.h"

namespace {
constexpr int kRoiSize = 60;
constexpr float kPosInterpolationAlpha = 0.00f;
constexpr int kMaxCorners = 200;
constexpr double kQualityLevel = 0.001;
constexpr double kMinCornerDistance = 7.0;
const cv::Size kWindowSize(15, 15);
constexpr int kMaxLevel = 3;
constexpr int kNumRotationPairs = 200;
constexpr int kMinTrackFrames = 2;
constexpr double kRespawnIntervalSeconds = 0.3;
constexpr int kTargetPointCount = 40;
}  // namespace

LKFlowTracker::LKFlowTracker(ICameraReceiver* videoSource)
    : OdometryBase(videoSource),
      _imageSize(0, 0),
      _angle(Angle(0)),
      _initialized(false),
      _lastRespawnTime(0.0),
      _rng(std::random_device{}()) {
  // This tracker only tracks opponent, robot data always invalid
}

void LKFlowTracker::_DeduplicateTracks(std::vector<TrackPt>& tracks,
                                       float minDist) {
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
  _pos = cv::Point2f(0, 0);
  _prevTime.reset();
}

// Clips the ROI to be within the image bounds.
cv::Rect LKFlowTracker::_ClipROIToBounds(cv::Rect roi, cv::Size bounds) const {
  if (bounds.width <= 0 || bounds.height <= 0) {
    return roi;
  }

  int x = (std::max)(0, roi.x);
  int y = (std::max)(0, roi.y);
  int w = (std::min)(roi.width, bounds.width - x);
  int h = (std::min)(roi.height, bounds.height - y);

  if (w < 0) w = 0;
  if (h < 0) h = 0;

  return cv::Rect(x, y, w, h);
}

cv::Rect LKFlowTracker::_GetROI() const {
  cv::Rect roi(static_cast<int>(_pos.x) - kRoiSize / 2,
               static_cast<int>(_pos.y) - kRoiSize / 2, kRoiSize, kRoiSize);
  return _ClipROIToBounds(roi, _imageSize);
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
  cv::Mat gray;
  EnforceGrayscale(currFrame, gray);

  _imageSize = gray.size();

  cv::Rect roi = _GetROI();

  if (roi.width > 0 && roi.height > 0 &&
      ((frameTime - _lastRespawnTime) >= kRespawnIntervalSeconds) &&
      _RespawnPoints(gray, roi, _tracks, kTargetPointCount)) {
    _lastRespawnTime = frameTime;
  }

  if (_prevGray.empty() || _tracks.size() < 6) {
    _prevGray = gray;
    std::cout << "No points, skipping update" << std::endl;
    return;
  }

  bool success = _UpdateTracking(_prevGray, gray, frameTime);

  _prevGray = gray;

  if (!success) {
    _initialized = false;
  }
}

bool LKFlowTracker::_InitializePoints(cv::Mat& gray) {
  _tracks.clear();

  cv::Rect roi = _GetROI();

  if (!_RespawnPoints(gray, roi, _tracks, kMaxCorners)) {
    return false;
  }

  if (_tracks.size() < 8) {
    _tracks.clear();
    return false;
  }

  _initialized = true;
  _lastRespawnTime = Clock::programClock.getElapsedTime();

  return true;
}

bool LKFlowTracker::_UpdateTracking(cv::Mat& prevGray, cv::Mat& currGray,
                                    double frameTime) {
  std::vector<cv::Point2f> prevPts;
  prevPts.reserve(_tracks.size());
  for (const auto& track : _tracks) {
    prevPts.push_back(track.pt);
  }

  std::vector<cv::Point2f> nextPts;
  std::vector<uchar> status;
  std::vector<float> err;

  cv::calcOpticalFlowPyrLK(
      prevGray, currGray, prevPts, nextPts, status, err, kWindowSize, kMaxLevel,
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

  // Compute translation from motion deltas (prev -> next) before compaction
  cv::Point2f deltaPos =
      _ComputeTranslationFromPoints(prevPts, nextPts, status);

  // Single-pass compaction: keep only good points, increment age
  std::vector<TrackPt> tracksNext;
  tracksNext.reserve(_tracks.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] != 1) {
      continue;
    }
    tracksNext.push_back({nextPts[i], _tracks[i].age + 1});
  }

  _FilterPointsByROI(tracksNext);
  _DeduplicateTracks(tracksNext, kMinCornerDistance * 0.8f);  // tune: 0.6–1.0

  // If we have too few points after tracking, fail (respawn handled in
  // _ProcessNewFrame)
  if (tracksNext.size() < 6) {
    return false;
  }

  // Update angle (angleDelta is already in radians)
  _angle = _angle + Angle(angleDelta);

  // Update position and velocity
  _pos += deltaPos;

  // Slowly pull _pos toward centroid of tracked points
  _InterpolatePosTowardCenter(tracksNext);

  cv::Point2f visualVelocity = cv::Point2f(0, 0);
  double deltaTime =
      _prevTime.has_value() ? (frameTime - _prevTime.value()) : 0.0;

  if (deltaTime > 0.001) {
    visualVelocity = deltaPos / static_cast<float>(deltaTime);
  }

  // Angular velocity = angleDelta / deltaTime (same deltaTime as position)
  double angularVelocity = 0.0;
  if (deltaTime > 0.001) {
    angularVelocity = angleDelta / deltaTime;
  }

  // Update tracks with compacted results
  _tracks = tracksNext;

  // Build sample and publish
  OdometryData sample{};
  sample.id = _frameID++;
  sample.pos = PositionData(_pos, visualVelocity, frameTime);
  sample.angle = AngleData(_angle, angularVelocity, frameTime);
  OdometryBase::Publish(sample, true);

  // Update local previous data for next iteration
  _prevTime = frameTime;

  _DrawDebugImage(currGray.size(), nextPts, validPairs);

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
    if (_tracks[idx1].age < kMinTrackFrames ||
        _tracks[idx2].age < kMinTrackFrames) {
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

cv::Point2f LKFlowTracker::_ComputeTranslationFromPoints(
    const std::vector<cv::Point2f>& prevPts,
    const std::vector<cv::Point2f>& nextPts, const std::vector<uchar>& status) {
  if (prevPts.size() != nextPts.size() || nextPts.size() != status.size()) {
    return cv::Point2f(0, 0);
  }
  // Motion deltas for successfully tracked points
  std::vector<cv::Point2f> deltas;
  deltas.reserve(nextPts.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] != 1) continue;
    deltas.push_back(nextPts[i] - prevPts[i]);
  }
  if (deltas.empty()) {
    return cv::Point2f(0, 0);
  }
  // Binned 2D mode: find the densest cell of deltas, return its centroid.
  const float binSize = 1.0f;
  std::map<std::pair<int, int>, std::pair<int, cv::Point2f>> bins;
  for (const auto& pt : deltas) {
    int bx = static_cast<int>(std::floor(pt.x / binSize));
    int by = static_cast<int>(std::floor(pt.y / binSize));
    auto key = std::make_pair(bx, by);
    auto& entry = bins[key];
    entry.first += 1;
    entry.second += pt;
  }
  const auto best = std::max_element(bins.begin(), bins.end(),
                                     [](const auto& a, const auto& b) {
                                       return a.second.first < b.second.first;
                                     });
  return best->second.second / static_cast<float>(best->second.first);
}

void LKFlowTracker::_DrawDebugImage(
    cv::Size imageSize, const std::vector<cv::Point2f>& nextPts,
    const std::vector<std::pair<int, int>>& validPairs) {
  std::unique_lock<std::mutex> debugLock(_mutexDebugImage);
  _debugImage = cv::Mat::zeros(imageSize, CV_8UC1);

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
  cv::circle(_debugImage, _pos, 5, cv::Scalar(255), 2);

  cv::Rect roi = _GetROI();
  if (roi.width > 0 && roi.height > 0) {
    cv::rectangle(_debugImage, roi, cv::Scalar(128), 2);
  }

  // draw an arrow from _pos at the angle
  cv::Point2f arrowEnd = _pos + cv::Point2f(30 * cos(_angle), 30 * sin(_angle));
  cv::arrowedLine(_debugImage, _pos, arrowEnd, cv::Scalar(255), 2);
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
  int maxToRequest = (std::min)(needed * 2, kMaxCorners);

  // Clip ROI to image bounds (safety check in case image size changed)
  cv::Rect validROI = _ClipROIToBounds(roi, gray.size());

  if (validROI.width <= 0 || validROI.height <= 0) {
    return false;
  }
  cv::Mat roiGray = gray(validROI);

  std::vector<cv::Point2f> newPts;
  cv::goodFeaturesToTrack(roiGray, newPts, maxToRequest, kQualityLevel,
                          kMinCornerDistance);

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
      kMinCornerDistance *
      kMinCornerDistance;  // Use squared distance for efficiency
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

void LKFlowTracker::_InterpolatePosTowardCenter(
    const std::vector<TrackPt>& tracks) {
  if (kPosInterpolationAlpha == 0.0f) return;
  if (tracks.empty()) return;
  cv::Point2f sum(0, 0);
  for (const auto& t : tracks) {
    sum += t.pt;
  }
  cv::Point2f center = sum / static_cast<float>(tracks.size());
  _pos += kPosInterpolationAlpha * (center - _pos);
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
  int numPairs =
      (std::min)(kNumRotationPairs, static_cast<int>(allPairs.size()));
  if (static_cast<int>(allPairs.size()) > numPairs) {
    std::shuffle(allPairs.begin(), allPairs.end(), _rng);
    pairs.assign(allPairs.begin(), allPairs.begin() + numPairs);
  } else {
    pairs = allPairs;
  }

  return pairs;
}

void LKFlowTracker::_FilterPointsByROI(std::vector<TrackPt>& tracks) {
  cv::Rect roi = _GetROI();
  // If ROI is invalid (width or height <= 0), keep all points
  if (roi.width <= 0 || roi.height <= 0) {
    return;
  }

  // Remove points that are outside the ROI
  tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                              [roi](const TrackPt& track) {
                                // Check ROI bounds
                                if (track.pt.x < roi.x ||
                                    track.pt.x >= roi.x + roi.width ||
                                    track.pt.y < roi.y ||
                                    track.pt.y >= roi.y + roi.height) {
                                  return true;
                                }

                                return false;
                              }),
               tracks.end());
}

void LKFlowTracker::SwitchRobots() {
  std::cerr << "SwitchRobots not implemented for LKFlowTracker" << std::endl;
}

void LKFlowTracker::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  if (!opponentRobot) return;

  _pos = newPos;
  _FilterPointsByROI(_tracks);
}

void LKFlowTracker::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
  std::cerr << "SetVelocity not implemented for LKFlowTracker" << std::endl;
}

void LKFlowTracker::SetAngle(AngleData angleData, bool opponentRobot) {
  if (!opponentRobot) return;

  std::lock_guard<std::mutex> lk(_updateMutex);
  _angle = angleData.angle;
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
