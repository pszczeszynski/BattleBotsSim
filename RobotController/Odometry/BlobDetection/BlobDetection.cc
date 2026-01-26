#include "BlobDetection.h"

#include <algorithm>

#include "../../Clock.h"
#include "../../Globals.h"
#include "../../RobotConfig.h"
#include "../../SafeDrawing.h"


// *********************************************************************
// NOTE:
// Blob detection is used for position and velocity of us and them.
// It does do path tangent calculation here, but it is not used for anything
// yet.
// **********************************************************************

BlobDetection::BlobDetection(ICameraReceiver* videoSource)
    : OdometryBase(videoSource) {
  // State is initialized with default values
}

void BlobDetection::_ProcessNewFrame(const cv::Mat currFrame,
                                     double frameTime) {
  if (_previousImage.empty()) {
    _previousImage = currFrame.clone();
    _prevFrameTime = frameTime;
    return;
  }

  // Run expensive vision work first (no lock)
  VisionClassification robotData =
      _DoBlobDetection(currFrame, _previousImage, frameTime);

  if (robotData.robot.has_value()) {
    _ProcessStream(robotData.robot.value(), _usState, false, frameTime);
  }
  if (robotData.opponent.has_value()) {
    _ProcessStream(robotData.opponent.value(), _themState, true, frameTime);
  }

  // Move currFrame to previousImage
  // But only if both our robot and opponent robot have been found, or we
  // exceeded a timer
  if (robotData.robot.has_value() || robotData.opponent.has_value() ||
      (frameTime - _prevFrameTime) > (1.0 / BLOBS_MIN_FPS)) {
    // save the current frame as the previous frame
    _previousImage = currFrame.clone();
    _prevFrameTime = frameTime;
  }
}

/**
 * Locates the robots in the frame
 * Note: Incoming images are black and white
 */
VisionClassification BlobDetection::_DoBlobDetection(
    const cv::Mat& currFrame, const cv::Mat& previousFrame, double frameTime) {
  const cv::Size BLUR_SIZE = cv::Size(14, 14);
  const float MIN_DIM = (std::min)(MIN_OPPONENT_BLOB_SIZE, MIN_ROBOT_BLOB_SIZE);
  const float MAX_DIM = (std::max)(MAX_OPPONENT_BLOB_SIZE, MAX_ROBOT_BLOB_SIZE);

  // Compute the absolute difference between the current frame and the previous
  // frame
  cv::Mat diff;
  cv::absdiff(previousFrame, currFrame, diff);

  // Convert the difference to grayscale
  cv::Mat grayDiff;

  if (diff.channels() == 1) {
    grayDiff = diff;
  } else if (diff.channels() == 3) {
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);
  } else if (diff.channels() == 4) {
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGRA2GRAY);
  }

  // Convert the difference to a binary image with a certain threshold
  cv::Mat thresholdImg;
  cv::threshold(grayDiff, thresholdImg, MOTION_LOW_THRESHOLD, 255,
                cv::THRESH_BINARY);

  // blurr and re-thresh to make it more leanient
  cv::blur(thresholdImg, thresholdImg, BLUR_SIZE);
  cv::threshold(thresholdImg, thresholdImg, 15, 255, cv::THRESH_BINARY);

  // Use connectedComponentsWithStats for more efficient blob detection
  cv::Mat labels, stats, centroids;
  int numLabels = cv::connectedComponentsWithStats(thresholdImg, labels, stats,
                                                   centroids, 8, CV_32S);

  std::vector<MotionBlob> motionBlobs = {};
  for (int i = 1; i < numLabels; i++) {  // Skip background label 0
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area = stats.at<int>(i, cv::CC_STAT_AREA);

    // Check size constraints
    if (width >= MIN_DIM && height >= MIN_DIM && width <= MAX_DIM &&
        height <= MAX_DIM) {
      cv::Rect rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                    stats.at<int>(i, cv::CC_STAT_TOP), width, height);
      cv::Point2f center(centroids.at<double>(i, 0),
                         centroids.at<double>(i, 1));
      // MotionBlob doesn't need frame pointer - only rect and center are used
      motionBlobs.emplace_back(MotionBlob{rect, center, nullptr});
    }
  }

  // Create lightweight TrackPrior objects for classifier from current state
  TrackPrior usPrior{false, cv::Point2f(0, 0), cv::Point2f(0, 0)};
  TrackPrior themPrior{false, cv::Point2f(0, 0), cv::Point2f(0, 0)};

  // Read state under lock for classifier
  {
    std::lock_guard<std::mutex> lock(_updateMutex);
    if (_usState.have_pos && _usState.last_position.has_value()) {
      usPrior.valid = true;
      usPrior.pos = _usState.last_position.value();
      usPrior.vel = _usState.last_velocity;
    }

    if (_themState.have_pos && _themState.last_position.has_value()) {
      themPrior.valid = true;
      themPrior.pos = _themState.last_position.value();
      themPrior.vel = _themState.last_velocity;
    }
  }

  // Identify which blobs are our robots (finds robot positions)
  // ClassifyBlobs doesn't mutate frame, so we can pass const reference
  VisionClassification result = _robotClassifier.ClassifyBlobs(
      motionBlobs, currFrame, thresholdImg, usPrior, themPrior, frameTime);

  // Build debug image (build locally, then swap under lock)
  cv::Mat debugLocal;
  thresholdImg.copyTo(debugLocal);

  for (const MotionBlob& blob : motionBlobs) {
    cv::rectangle(debugLocal, blob.rect, cv::Scalar(255), 2);
    safe_circle(debugLocal, blob.center, 5, cv::Scalar(255), 2);
  }

  {
    std::scoped_lock<std::mutex> debuglock(_mutexDebugImage);
    _debugImage = std::move(debugLocal);
  }

  return result;
}

void BlobDetection::GetDebugImage(cv::Mat& debugImage, cv::Point offset) {
  OdometryBase::GetDebugImage(
      debugImage, offset);  // Call base class to add the odometry data

  // Get unique access to _debugImage
  std::unique_lock<std::mutex> locker(_mutexDebugImage);
  if (_debugImage.empty() || _debugImage.size() != debugImage.size() ||
      _debugImage.type() != debugImage.type()) {
    locker.unlock();
    return;  // No valid _debugImage to merge
  }

  // Add _debugImage to debugImage
  cv::add(debugImage, _debugImage,
          debugImage);  // Add images, store result in debugImage

  locker.unlock();  // Unlock mutex after operation
}

void BlobDetection::_ProcessStream(const MotionBlob& blob,
                                   BlobTrackState& state, bool isOpponent,
                                   double timestamp) {
  // Save previous position before updating (needed for velocity calculation)
  std::optional<cv::Point2f> prevPos = state.last_position;
  cv::Point2f newPos = blob.center;

  // Update velocity smoothing BEFORE updating position state
  // (uses prevPos from state before we update it)
  _UpdateSmoothedVelocity(state, newPos, timestamp, prevPos);

  // Update position state
  state.last_position = newPos;
  state.last_rect = blob.rect;
  state.last_position_time = timestamp;
  state.have_pos = true;

  // Update angle-path-tangent
  _UpdateAnglePathTangent(state, newPos, timestamp);

  // Publish the new data
  _PublishFromState(state, isOpponent, timestamp);
}

/**
 * Smooths out the visual velocity so it's not so noisy
 * Uses prevPos (from state before update) and newPos to calculate velocity
 */
constexpr double kNewVisualVelocityTimeWeightMS = 50;
void BlobDetection::_UpdateSmoothedVelocity(
    BlobTrackState& state, cv::Point2f newPos, double timestamp,
    const std::optional<cv::Point2f>& prevPos) {
  // Guard: first sample, no previous position, or last_vel_time not initialized
  if (!prevPos.has_value() || state.last_vel_time == 0) {
    // First position or no previous position - start fresh
    state.last_velocity = cv::Point2f(0, 0);
    state.last_vel_time = timestamp;
    return;
  }

  double elapsedVelTime = timestamp - state.last_vel_time;

  // Guard: invalid/zero/negative elapsed time (reset velocity and time)
  if (elapsedVelTime <= 0 || elapsedVelTime > 0.1f) {
    // Reset velocity if elapsed time is invalid or too large
    state.last_velocity = cv::Point2f(0, 0);
    state.last_vel_time = timestamp;
    return;
  }

  // Calculate visual velocity using previous position
  cv::Point2f visualVelocity = (newPos - prevPos.value()) / elapsedVelTime;

  // compute weight for interpolation
  double weight = elapsedVelTime * 1000 / kNewVisualVelocityTimeWeightMS;
  weight = (std::min)(weight, 1.0);

  // interpolate towards the visual velocity so it's not so noisy
  state.last_velocity =
      InterpolatePoints(state.last_velocity, visualVelocity, weight);
  state.last_vel_time = timestamp;
}

// Number of seconds to average velocity for APT threshold calcs. This sets the
// settling time constant.
constexpr double kAptVelocityAveraging = 0.5;

// the displacement required to update the angle
constexpr double kDistBetweenAngUpdatesPx = 5;

// how fast the robot needs to be moving to update the angle
constexpr double kVelocityThresholdForAngleUpdate = 30 * WIDTH / 720.0;

/**
 * @brief updates the angle of the robot using the velocity
 */
void BlobDetection::_UpdateAnglePathTangent(BlobTrackState& state,
                                            cv::Point2f newPos,
                                            double timestamp) {
  constexpr float kAngleSmoothingTimeConstantMs = 80;
  constexpr float kMinVelocityForAngleWeight =
      0.5 * kVelocityThresholdForAngleUpdate;
  constexpr float kMaxVelocityForAngleWeight =
      2.0 * kVelocityThresholdForAngleUpdate;

  // Initialize APT velocity accumulator if needed
  if (!state.prev_vel_for_apt_set) {
    state.prev_vel_for_apt_set = true;
    state.apt_velocity = cv::Point2f(0, 0);
    state.apt_velocity_time = timestamp;
  }

  // Update APT velocity lowpass accumulator
  double delta_apt_time = timestamp - state.apt_velocity_time;
  // Never add in the full amount due to the noise
  double scaling_apt = (std::min)(delta_apt_time / kAptVelocityAveraging, 0.25);
  state.apt_velocity = state.apt_velocity * (1.0 - scaling_apt) +
                       scaling_apt * state.last_velocity;
  state.apt_velocity_time = timestamp;

  // Calculate elapsed time and delta position for angle update
  double elapsedAngleTime = timestamp - state.prev_angle_ref_time;

  // Initialize angle reference if this is the first update
  if (!state.prev_angle_ref_pos.has_value()) {
    state.prev_angle_ref_pos = newPos;
    state.prev_angle_ref_time = timestamp;
    return;  // Can't calculate angle without previous position
  }

  cv::Point2f delta = newPos - state.prev_angle_ref_pos.value();

  double robotVel = cv::norm(state.last_velocity);
  double aptRobotVel = cv::norm(state.apt_velocity);
  double minRobotVel = (std::min)(robotVel, aptRobotVel);

  // Check both distance and time thresholds
  if (cv::norm(delta) < kDistBetweenAngUpdatesPx || elapsedAngleTime < 0.001 ||
      !state.have_pos) {
    // Don't update angle - keep previous reference state
    return;
  }

  // Update the angle
  Angle newAngle = Angle(atan2(delta.y, delta.x));
  if (std::isnan((double)newAngle)) {
    return;
  }

  // If the angle is closer to 180 degrees to the last angle
  if (state.prev_angle_ref_angle.has_value() &&
      abs(Angle(newAngle + M_PI - state.prev_angle_ref_angle.value())) <
          abs(Angle(newAngle - state.prev_angle_ref_angle.value()))) {
    // Add 180 degrees to the angle
    newAngle = Angle(newAngle + M_PI);
  }

  Angle newAngleInterpolated{0};
  double angularVelocity{0};

  // Calculate angular velocity
  if (state.prev_angle_ref_angle.has_value() &&
      !std::isnan((double)state.prev_angle_ref_angle.value())) {
    // Base time-based weight
    double time_weight =
        (std::min)(elapsedAngleTime * 1000.0 / kAngleSmoothingTimeConstantMs,
                   1.0);

    // Inverse time weight: if time updates exceeds reasonable levels, that
    // means there is a discontinuity in the data
    // Normally we expect 20ms per update or faster.
    // if it approaches 150ms, its junk.
    time_weight *= (std::max)((0.15 - elapsedAngleTime) / 0.15, 0.02);

    // Velocity-based weight: scale based on aptRobotVel
    double velocity_weight = std::clamp(
        (minRobotVel - kMinVelocityForAngleWeight) /
            (kMaxVelocityForAngleWeight - kMinVelocityForAngleWeight),
        0.0, 1.0);

    // Combine time and velocity weights
    // Limit the weight to prevent huge jumps
    double final_weight = (std::max)(time_weight * velocity_weight, 0.25);

    // Interpolate the angle with velocity-modulated weight
    newAngleInterpolated = InterpolateAngles(state.prev_angle_ref_angle.value(),
                                             newAngle, final_weight);
    angularVelocity =
        (newAngleInterpolated - state.prev_angle_ref_angle.value()) /
        elapsedAngleTime;
  } else {
    newAngleInterpolated = newAngle;
    angularVelocity = 0;
  }

  // Update angle reference state for next iteration
  state.prev_angle_ref_pos = newPos;
  state.prev_angle_ref_time = timestamp;
  state.prev_angle_ref_angle = newAngleInterpolated;
  state.prev_angle_ref_angular_velocity = angularVelocity;
}

void BlobDetection::_PublishFromState(BlobTrackState& state, bool isOpponent,
                                      double timestamp) {
  OdometryData sample;
  sample.Clear();

  // Set position data
  if (state.have_pos && state.last_position.has_value()) {
    PositionData posData(state.last_position.value(), state.last_velocity,
                         timestamp);
    if (state.last_rect.has_value()) {
      posData.rect = state.last_rect.value();
    }
    sample.pos = posData;
  }

  // Set angle data (if we have angle reference)
  if (state.prev_angle_ref_angle.has_value()) {
    sample.angle = AngleData(state.prev_angle_ref_angle.value(),
                             state.prev_angle_ref_angular_velocity, timestamp);
  }

  // Publish (base class will increment id from previous value)
  OdometryBase::Publish(sample, isOpponent);
}

void BlobDetection::SwitchRobots(void) {
  // Switch who's who - swap the state structs
  std::unique_lock<std::mutex> locker(_updateMutex);
  std::swap(_usState, _themState);
  locker.unlock();
}

void BlobDetection::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  constexpr float IGNORE_THRESH_PX = 10;

  BlobTrackState& state = opponentRobot ? _themState : _usState;

  std::unique_lock<std::mutex> locker(_updateMutex);

  // ignore the request if our existing position is fine
  if (state.have_pos && state.last_position.has_value() &&
      cv::norm(newPos - state.last_position.value()) < IGNORE_THRESH_PX) {
    return;
  }

  double currTime = Clock::programClock.getElapsedTime();

  // Update state
  state.last_position = newPos;
  state.last_position_time = currTime;
  state.have_pos = true;
  state.last_velocity = cv::Point2f(0, 0);
  state.last_vel_time = currTime;

  locker.unlock();
}

void BlobDetection::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
  // Not implemented - velocity is calculated from position updates
}

void BlobDetection::SetAngle(AngleData angleData, bool opponentRobot) {
  // Not implemented - angle is calculated from path tangent
}