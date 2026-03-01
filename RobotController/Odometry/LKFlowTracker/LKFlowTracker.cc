#include "LKFlowTracker.h"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

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
constexpr int kTargetPointCount = 30;
}  // namespace

LKFlowTracker::LKFlowTracker(ICameraReceiver* videoSource)
    : OdometryBase(videoSource),
      _imageSize(0, 0),
      _rng(std::random_device{}()) {
  _targets[0] = LKFlowTargetState{};
  _targets[1] = LKFlowTargetState{};
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
  _prevGray = cv::Mat();
  _imageSize = cv::Size(0, 0);
  _targets[0] = LKFlowTargetState{};
  _targets[1] = LKFlowTargetState{};
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

cv::Rect LKFlowTracker::_GetROI(cv::Point2f pos, cv::Size imageSize) const {
  cv::Rect roi(static_cast<int>(pos.x) - kRoiSize / 2,
               static_cast<int>(pos.y) - kRoiSize / 2, kRoiSize, kRoiSize);
  return _ClipROIToBounds(roi, imageSize);
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

  {
    std::unique_lock<std::mutex> debugLock(_mutexDebugImage);
    _debugImage = cv::Mat::zeros(_imageSize, CV_8UC3);
  }

  for (int i = 0; i < 2; ++i) {
    LKFlowTargetState& state = _targets[i];
    cv::Rect roi = _GetROI(state.pos, _imageSize);
    cv::Rect otherRoi = _GetROI(_targets[1 - i].pos, _imageSize);
    bool shouldRespawn =
        (frameTime - state.lastRespawnTime) >= kRespawnIntervalSeconds ||
        state.forceRespawnNextFrame;
    if (roi.width > 0 && roi.height > 0 && shouldRespawn) {
      _RespawnPoints(gray, roi, state.tracks, kTargetPointCount, frameTime,
                     state.lastRespawnTime, otherRoi);
      state.forceRespawnNextFrame = false;
    }
  }

  if (_prevGray.empty()) {
    _prevGray = gray;
    return;
  }

  for (int i = 0; i < 2; ++i) {
    LKFlowTargetState& state = _targets[i];
    if (state.tracks.size() < 6) continue;
    bool success = _UpdateTracking(_prevGray, gray, frameTime, state, i == 1);
    if (!success) state.initialized = false;
  }

  _prevGray = gray;
}

bool LKFlowTracker::_UpdateTracking(cv::Mat& prevGray, cv::Mat& currGray,
                                    double frameTime, LKFlowTargetState& state,
                                    bool isOpponent) {
  std::vector<cv::Point2f> prevPts;
  prevPts.reserve(state.tracks.size());
  for (const auto& track : state.tracks) {
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
  double angleDelta = 0.0;
  std::vector<std::pair<int, int>> validPairs;
  _ComputeRotationsFromPairs(state.tracks, nextPts, status, angleDelta,
                             validPairs);

  // // Compute translation from motion deltas (prev -> next) before compaction
  // cv::Point2f deltaPos =
  //     _ComputeTranslationFromPoints(state.tracks, prevPts, nextPts, status);

  // Single-pass compaction: keep only good points, increment age
  std::vector<TrackPt> tracksNext;
  tracksNext.reserve(state.tracks.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] != 1) {
      continue;
    }
    tracksNext.push_back({nextPts[i], state.tracks[i].age + 1});
  }

  cv::Rect roi = _GetROI(state.pos, _imageSize);
  _FilterPointsByROI(tracksNext, roi);
  _DeduplicateTracks(tracksNext, kMinCornerDistance * 0.8f);

  if (tracksNext.size() < 6) {
    return false;
  }

  state.angle = state.angle + Angle(angleDelta);
  cv::Point2f oldPos = state.pos;
  state.pos = _ComputeCenterFromPoints(tracksNext);
  cv::Point2f deltaPos = state.pos - oldPos;
  cv::Point2f visualVelocity = cv::Point2f(0, 0);
  double deltaTime =
      state.prevTime.has_value() ? (frameTime - state.prevTime.value()) : 0.0;

  if (deltaTime > 0.001) {
    visualVelocity = deltaPos / static_cast<float>(deltaTime);
  }

  double angularVelocity = 0.0;
  if (deltaTime > 0.001) {
    angularVelocity = angleDelta / deltaTime;
  }

  state.tracks = std::move(tracksNext);

  OdometryData sample{};
  sample.id = _frameID++;
  sample.pos = PositionData(state.pos, visualVelocity, frameTime);
  sample.angle = AngleData(state.angle, angularVelocity, frameTime);
  OdometryBase::Publish(sample, isOpponent, OdometryAlg::LKFlow);

  state.prevTime = frameTime;

  const cv::Scalar kUsColor(0, 255, 0);          // Green – us
  const cv::Scalar kOpponentColor(255, 165, 0);  // Orange – opponent
  _DrawDebugImage(currGray.size(), nextPts, status, validPairs, roi,
                  isOpponent ? kOpponentColor : kUsColor);

  return true;
}

void LKFlowTracker::_ComputeRotationsFromPairs(
    const std::vector<TrackPt>& tracks, const std::vector<cv::Point2f>& nextPts,
    const std::vector<uchar>& status, double& angleDelta,
    std::vector<std::pair<int, int>>& validPairs) {
  std::vector<RotationResult> rotations;
  validPairs.clear();

  if (tracks.size() != nextPts.size() || tracks.size() < 2) {
    angleDelta = 0.0;
    return;
  }

  std::vector<std::pair<int, int>> pairs =
      _GeneratePointPairs(static_cast<int>(tracks.size()));

  for (const auto& pair : pairs) {
    int idx1 = pair.first;
    int idx2 = pair.second;

    if (idx1 >= static_cast<int>(tracks.size()) ||
        idx2 >= static_cast<int>(tracks.size()) ||
        idx1 >= static_cast<int>(status.size()) ||
        idx2 >= static_cast<int>(status.size()) || status[idx1] != 1 ||
        status[idx2] != 1) {
      continue;
    }

    if (tracks[idx1].age < kMinTrackFrames ||
        tracks[idx2].age < kMinTrackFrames) {
      continue;
    }

    cv::Point2f p1Prev = tracks[idx1].pt;
    cv::Point2f p2Prev = tracks[idx2].pt;
    cv::Point2f p1Next = nextPts[idx1];
    cv::Point2f p2Next = nextPts[idx2];

    cv::Point2f vecPrev = p2Prev - p1Prev;
    cv::Point2f vecNext = p2Next - p1Next;

    float normPrev = cv::norm(vecPrev);
    float normNext = cv::norm(vecNext);
    if (normPrev < 3.0f || normNext < 3.0f) {
      continue;
    }

    float anglePrev = static_cast<float>(std::atan2(vecPrev.y, vecPrev.x));
    float angleNext = static_cast<float>(std::atan2(vecNext.y, vecNext.x));
    float distance = cv::norm(p2Next - p1Next);
    float rot = angle_wrap(angleNext - anglePrev);

    rotations.push_back(RotationResult{rot, distance});
    validPairs.push_back(pair);
  }

  _UpdateAngleFromRotations(rotations, angleDelta);
}

cv::Point2f LKFlowTracker::_ComputeTranslationFromPoints(
    const std::vector<TrackPt>& tracks, const std::vector<cv::Point2f>& prevPts,
    const std::vector<cv::Point2f>& nextPts, const std::vector<uchar>& status) {
  if (prevPts.size() != nextPts.size() || nextPts.size() != status.size()) {
    return cv::Point2f(0, 0);
  }
  std::vector<std::pair<float, cv::Point2f>> translations;
  translations.reserve(status.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (tracks[i].age < 10 || status[i] != 1) {
      continue;
    }
    cv::Point2f d = nextPts[i] - prevPts[i];
    float mag = static_cast<float>(cv::norm(d));
    translations.emplace_back(mag, d);
  }
  if (translations.empty()) {
    return cv::Point2f(0, 0);
  }
  const size_t topHalfCount =
      (std::max)(size_t(1), static_cast<size_t>(translations.size() * 0.75));
  std::partial_sort(
      translations.begin(),
      translations.begin() + static_cast<std::ptrdiff_t>(topHalfCount),
      translations.end(),
      [](const std::pair<float, cv::Point2f>& a,
         const std::pair<float, cv::Point2f>& b) { return a.first > b.first; });
  cv::Point2f sum(0, 0);
  for (size_t i = 0; i < topHalfCount; ++i) {
    sum += translations[i].second;
  }
  return sum / static_cast<float>(topHalfCount);
}

void LKFlowTracker::_DrawDebugImage(
    cv::Size imageSize, const std::vector<cv::Point2f>& nextPts,
    const std::vector<uchar>& status,
    const std::vector<std::pair<int, int>>& validPairs, cv::Rect roi,
    const cv::Scalar& color) {
  std::unique_lock<std::mutex> debugLock(_mutexDebugImage);
  if (_debugImage.empty() || _debugImage.size() != imageSize) return;

  const cv::Scalar kGoodColor = color;
  const cv::Scalar kBadColor(0, 0, 255);

  for (size_t i = 0; i < nextPts.size(); ++i) {
    cv::Scalar ptColor =
        (i < status.size() && status[i] == 1) ? kGoodColor : kBadColor;
    cv::circle(_debugImage, nextPts[i], 2, ptColor, -1);
  }

  if (roi.width > 0 && roi.height > 0) {
    cv::rectangle(_debugImage, roi, color, 2);
  }
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
                                   int targetCount, double frameTime,
                                   double& lastRespawnTime,
                                   cv::Rect excludeRoi) {
  const int needed = targetCount - tracks.size();

  if (needed <= 0) {
    return true;
  }

  int maxToRequest = (std::min)(needed * 2, kMaxCorners);

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

  for (auto& pt : newPts) {
    pt.x += validROI.x;
    pt.y += validROI.y;
  }

  const double minDistSq = kMinCornerDistance * kMinCornerDistance;
  const bool hasExclude = excludeRoi.width > 0 && excludeRoi.height > 0;
  int added = 0;
  for (const auto& newPt : newPts) {
    if (added >= needed) {
      break;
    }

    if (hasExclude &&
        newPt.x >= excludeRoi.x && newPt.x < excludeRoi.x + excludeRoi.width &&
        newPt.y >= excludeRoi.y && newPt.y < excludeRoi.y + excludeRoi.height) {
      continue;
    }

    bool tooClose = false;
    for (const auto& track : tracks) {
      cv::Point2f diff = newPt - track.pt;
      double distSq = diff.x * diff.x + diff.y * diff.y;
      if (distSq < minDistSq) {
        tooClose = true;
        break;
      }
    }

    if (!tooClose) {
      tracks.push_back({newPt, 0});
      added++;
    }
  }

  if (added > 0) {
    lastRespawnTime = frameTime;
  }
  return added > 0;
}

cv::Point2f LKFlowTracker::_ComputeCenterFromPoints(
    const std::vector<TrackPt>& tracks) {
  if (tracks.empty()) {
    return cv::Point2f(0, 0);
  }

  cv::Point2f sum(0, 0);
  for (const auto& pt : tracks) {
    sum += pt.pt;
  }
  return sum / static_cast<float>(tracks.size());
}

void LKFlowTracker::_InterpolatePosTowardCenter(
    const std::vector<TrackPt>& tracks, cv::Point2f& pos) {
  if (kPosInterpolationAlpha == 0.0f) return;
  if (tracks.empty()) return;
  cv::Point2f sum(0, 0);
  for (const auto& t : tracks) {
    sum += t.pt;
  }
  cv::Point2f center = sum / static_cast<float>(tracks.size());
  pos += kPosInterpolationAlpha * (center - pos);
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

void LKFlowTracker::_FilterPointsByROI(std::vector<TrackPt>& tracks,
                                       cv::Rect roi) {
  if (roi.width <= 0 || roi.height <= 0) {
    return;
  }

  tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                              [roi](const TrackPt& track) {
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

void LKFlowTracker::SetPosition(const PositionData& newPos,
                                bool opponentRobot) {
  if (newPos.algorithm == OdometryAlg::LKFlow) {
    return;
  }
  LKFlowTargetState& state = _targets[opponentRobot ? 1 : 0];
  cv::Point2f newPosPt = newPos.position;
  constexpr float kHardSkipThresholdPx = 10;
  if (cv::norm(newPosPt - state.pos) < kHardSkipThresholdPx) {
    state.pos = InterpolatePoints(state.pos, newPosPt, 0.05f);
    cv::Rect roi = _GetROI(state.pos, _imageSize);
    _FilterPointsByROI(state.tracks, roi);
    return;
  }

  state.pos = newPosPt;
  state.forceRespawnNextFrame = true;
  cv::Rect roi = _GetROI(state.pos, _imageSize);
  _FilterPointsByROI(state.tracks, roi);
}

void LKFlowTracker::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
  // std::cerr << "SetVelocity not implemented for LKFlowTracker" << std::endl;
}

void LKFlowTracker::SetAngle(AngleData angleData, bool opponentRobot) {
  LKFlowTargetState& state = _targets[opponentRobot ? 1 : 0];

  if (angleData.algorithm == OdometryAlg::LKFlow) {
    return;
  }

  std::lock_guard<std::mutex> lk(_updateMutex);

  constexpr float kHardSkipThresholdRad = 10.0f * TO_RAD;
  if (std::abs(angleData.angle - state.angle) < kHardSkipThresholdRad) {
    state.angle = InterpolateAngles(state.angle, angleData.angle, 0.05f);
    return;
  }

  state.angle = angleData.angle;
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
