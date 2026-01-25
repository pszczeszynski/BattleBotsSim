#include "BlobDetection.h"

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
  SetVelocity(cv::Point2f(0, 0), false);
  SetVelocity(cv::Point2f(0, 0), true);
}

void BlobDetection::_ProcessNewFrame(cv::Mat currFrame, double frameTime) {
  if (_previousImage.empty()) {
    _previousImage = currFrame;
    _prevFrameTime = frameTime;
    return;
  }

  VisionClassification robotData = DoBlobDetection(
      currFrame, _previousImage, frameTime);  // Locks the locker

  // Now update our standard data
  UpdateData(robotData, frameTime);

  // Move currFrame to previousImage
  // But only if both our robot and opponent robot have been found, or we
  // exceeded a timer

  if (robotData.GetRobotBlob() != nullptr ||
      robotData.GetOpponentBlob() != nullptr ||
      (frameTime - _prevFrameTime) > (1.0 / BLOBS_MIN_FPS)) {
    // save the current frame as the previous frame
    _previousImage = currFrame;
    _prevFrameTime = frameTime;
  }
}

/**
 * Locates the robots in the frame
 * Note: Incoming images are black and white
 */
VisionClassification BlobDetection::DoBlobDetection(
    cv::Mat& currFrame, cv::Mat& previousFrame,
    double frameTime) {
  // hyperparameters
  const cv::Size BLUR_SIZE = cv::Size(14, 14);
  const float MIN_DIM = min(MIN_OPPONENT_BLOB_SIZE, MIN_ROBOT_BLOB_SIZE);
  const float MAX_DIM = max(MAX_OPPONENT_BLOB_SIZE, MAX_ROBOT_BLOB_SIZE);
  const int BLOB_SEARCH_SIZE = 10;

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

  // find big blobs in the image using a blob detector

  // iterate through every pixel in the image and find the largest blob
  std::vector<cv::Rect> potentialRobots = {};
  for (int y = 0; y < thresholdImg.rows; y += BLOB_SEARCH_SIZE) {
    for (int x = 0; x < thresholdImg.cols; x += BLOB_SEARCH_SIZE) {
      // if this pixel is white, then it is part of a blob
      if (thresholdImg.at<uchar>(y, x) == 255) {
        // flood fill, mark the blob as 100 so that we don't flood fill it again
        cv::Rect rect;
        cv::floodFill(thresholdImg, cv::Point(x, y), cv::Scalar(100), &rect);

        // if the blob is a reasonable size, add it to the list
        if (rect.width >= MIN_DIM && rect.height >= MIN_DIM &&
            rect.width <= MAX_DIM && rect.height <= MAX_DIM) {
          // add the rect to the list of potential robots
          potentialRobots.push_back(rect);
        }
      }
    }
  }

  // for each robot, find the EXACT center of the robot by counting the white
  // pixels in the blob and averaging them
  std::vector<MotionBlob> motionBlobs = {};

  for (const cv::Rect& rect : potentialRobots) {
    // find the average of all the white pixels in the blob
    int numWhitePixels = 0;
    cv::Point2f averageWhitePixel = cv::Point2f(0, 0);
    for (int y = rect.y; y < rect.y + rect.height; y++) {
      for (int x = rect.x; x < rect.x + rect.width; x++) {
        // if this pixel is white, then add it to the average
        if (thresholdImg.at<uchar>(y, x) > 0) {
          averageWhitePixel += cv::Point2f(x, y);
          numWhitePixels++;
        }
      }
    }
    // divide by the number of white pixels to get the average
    averageWhitePixel /= numWhitePixels;

    // add the average to the list of robot centers
    motionBlobs.emplace_back(MotionBlob{rect, averageWhitePixel, &currFrame});
  }

  // draw the blobs
  std::unique_lock<std::mutex> debuglock(_mutexDebugImage);  // Locks the mutex
  // _debugImage = cv::Mat::zeros(thresholdImg.size(), CV_8UC1); // Initialize
  // debug image to black

  // cv::cvtColor(thresholdImg, _debugImage, cv::COLOR_BGR2GRAY);
  thresholdImg.copyTo(_debugImage);

  for (const MotionBlob& blob : motionBlobs) {
    cv::rectangle(_debugImage, blob.rect, cv::Scalar(255), 2);
    safe_circle(_debugImage, blob.center, 5, cv::Scalar(255), 2);
  }

  // draw the potential robots
  // motionImageWidget.UpdateMat(blobsImage);

  // static ImageWidget motionImageWidget{"Motion", true};
  debuglock.unlock();  // Unlock the mutex

  // Identify which blobs are our robots (finds robot positions)
  // classify the blobs and save them for later
  // Need previous data to be able to predict this data
  // Should we use extrapolated data? In this case maybe not so that we dont
  // compound a bad reading?
  VisionClassification result = _robotClassifier.ClassifyBlobs(
      motionBlobs, currFrame, thresholdImg, _currDataRobot, _currDataOpponent,
      false, frameTime);

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

void BlobDetection::UpdateData(VisionClassification robotBlobData,
                               double timestamp) {
  // Get unique access (already locked from calling function)
  //  std::unique_lock<std::mutex> locker(_updateMutex);

  MotionBlob* robot = robotBlobData.GetRobotBlob();
  MotionBlob* opponent = robotBlobData.GetOpponentBlob();

  if (robot != nullptr && _IsValidBlob(*robot, _currDataRobot)) {
    // Clear curr data
    _currDataRobot.Clear();
    _currDataRobot.userDataDouble["blobArea"] = robot->rect.area();
    // Update our robot position/velocity/angle
    _SetData(robotBlobData.GetRobotBlob(), _currDataRobot, _prevDataRobot,
             _prevAngleDataRobot, timestamp);
  } else {
    // increase invalid count + mark as invalid
    double invalidCount = _currDataRobot.userDataDouble["invalidCount"];

    if (invalidCount >= 10) {
      _currDataRobot.pos.reset();
      _currDataRobot.angle.reset();
      _prevDataRobot = _currDataRobot;  // Invalidate old data
    }

    _currDataRobot.userDataDouble["invalidCount"]++;
  }

  // Increment id and frame
  _currDataOpponent.id++;  // Increment frame id
  // Note: time is now stored in pos->time or angle->time

  if (opponent != nullptr && _IsValidBlob(*opponent, _currDataOpponent)) {
    // Make a copy of currData for velocity calls
    _prevDataOpponent = _currDataOpponent;

    // Clear curr data
    _currDataOpponent.Clear();

    _currDataOpponent.userDataDouble["blobArea"] = opponent->rect.area();
    // Update opponent position/velocity info
    _SetData(robotBlobData.GetOpponentBlob(), _currDataOpponent,
             _prevDataOpponent, _prevAngleDataOpponent, timestamp);
  } else {
    double invalidCount = _currDataOpponent.userDataDouble["invalidCount"];

    if (invalidCount >= 10) {
      _currDataOpponent.pos.reset();
      _currDataOpponent.angle.reset();
      _prevDataOpponent = _currDataOpponent;  // Invalidate old data
    }

    _currDataOpponent.userDataDouble["invalidCount"]++;
  }
}

// the displacement required to update the angle
constexpr double kDistBetweenAngUpdatesPx = 5;

// 0 = no change, 1 = full change
constexpr double kMovingAverageRate = 1.0;

// how fast the robot needs to be moving to update the angle
constexpr double kVelocityThresholdForAngleUpdate = 30 * WIDTH / 720.0;

// max rotational speed to update the angle radians per second
constexpr double kMaxRotationSpeedToAlign = 250 * TO_RAD;

/**
 * Makes sure the blob didn't shrink too much. Permitted to shrink if
 * we have too many invalid marks
 */
bool BlobDetection::_IsValidBlob(MotionBlob& blobNew, OdometryData& prevData) {
  double blobArea = blobNew.rect.area();
  double numUpdatesInvalid = prevData.userDataDouble["invalidCount"];
  double lastBlobArea = prevData.userDataDouble["blobArea"];
  // it's possible the motion detector's blob shrunk due to detecting only a
  // portion
  bool invalidBlob = blobArea < lastBlobArea * 0.8 && numUpdatesInvalid < 10;

  return !invalidBlob;
}

/**
 * @brief our position and velocity using just visual data
 * @param blob - the MotionBlob to update to
 * @param currData - the current data to update (will set the position and
 * velocity)
 */
void BlobDetection::_SetData(MotionBlob* blob, OdometryData& currData,
                             OdometryData& prevData,
                             OdometryData& prevAngleData, double timestamp) {
  //////////////////////// POS ////////////////////////
  // use the blob's center for the visual position
  // Velocity will be calculated in _GetSmoothedVisualVelocity, so set to 0 for
  // now
  currData.pos = PositionData((*blob).center, cv::Point2f(0, 0), timestamp);
  currData.pos.value().rect = (*blob).rect;

  //////////////////////// VEL ////////////////////////
  _GetSmoothedVisualVelocity(currData, prevData);

  //////////////////////// ANGLE ////////////////////////
  CalcAnglePathTangent(currData, prevAngleData, timestamp);

  prevData = currData;
}

/**
 * Smooths out the visual velocity so it's not so noisy
 */
#define NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS 50
#define NEW_VISUAL_VELOCITY_WEIGHT_DIMINISH_OPPONENT 1
void BlobDetection::_GetSmoothedVisualVelocity(OdometryData& currData,
                                               OdometryData& prevData) {
  if (!prevData.pos.has_value()) {
    // Don't update velocity incase it was set by a setData call
    return;
  }

  // If the previous or current position isnt valid or the total elapsed time is
  // too long, then restart velocity
  double currTime = currData.pos.value().time;
  double elapsedVelTime = currTime - prevData.userDataDouble["lastVelTime"];

  // If elapsed time between vel updates is too big, reset it
  if ((elapsedVelTime > 0.1f) || !currData.pos.has_value() ||
      !prevData.pos.has_value()) {
    currData.userDataDouble["lastVelTime"] = currTime;
    cv::Point2f pos = currData.pos.value().position;
    std::optional<cv::Rect> oldRect = currData.pos.value().rect;
    currData.pos = PositionData(pos, cv::Point2f(0, 0), currTime);
    currData.pos.value().rect = oldRect;
    return;
  }

  // visual velocity
  cv::Point2f visualVelocity =
      (currData.pos.value().position - prevData.pos.value().position) /
      elapsedVelTime;

  // compute weight for interpolation. Reduce weight for opponents
  double weight = elapsedVelTime * 1000 / NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS;

  weight = min(weight, 1.0f);

  // interpolate towards the visual velocity so it's not so noisy
  cv::Point2f newVelocity =
      InterpolatePoints(prevData.pos.value().velocity, visualVelocity, weight);
  cv::Point2f pos = currData.pos.value().position;
  std::optional<cv::Rect> oldRect = currData.pos.value().rect;
  currData.pos = PositionData(pos, newVelocity, currTime);
  currData.pos.value().rect = oldRect;
  currData.userDataDouble["lastVelTime"] = currTime;

  return;
}

// Number of seconds to average velocity for APT threhsold calcs. This sets the
// settling time constant.
constexpr double kAptVelocityAveraging = 0.5;
/**
 * @brief updates the angle of the robot using the velocity
 */
void BlobDetection::CalcAnglePathTangent(OdometryData& currData,
                                         OdometryData& prevAngleData,
                                         double timestamp) {
  constexpr float kAngleSmoothingTimeConstantMs = 80;
  constexpr float kMinVelocityForAngleWeight =
      0.5 * kVelocityThresholdForAngleUpdate;

  constexpr float kMaxVelocityForAngleWeight =
      2.0 * kVelocityThresholdForAngleUpdate;

  double elapsedAngleTime =
      timestamp -
      (prevAngleData.angle.has_value() ? prevAngleData.angle.value().time : 0);
  cv::Point2f delta =
      currData.pos.value().position - prevAngleData.pos.value().position;

  // Calculate a slower averaging on velocity to filter out severe noise
  if (prevAngleData.userDataDouble["prevVelForAPTSet"] < 0.5) {
    prevAngleData.userDataDouble["prevVelForAPTSet"] = 1.0;
    prevAngleData.userDataDouble["APT_Vel_x"] = 0;
    prevAngleData.userDataDouble["APT_Vel_y"] = 0;
    prevAngleData.userDataDouble["APT_Vel_t"] = timestamp;
  }

  cv::Point2f apt_velocity(prevAngleData.userDataDouble["APT_Vel_x"],
                           prevAngleData.userDataDouble["APT_Vel_y"]);
  double apt_old_time = prevAngleData.userDataDouble["APT_Vel_t"];
  double delta_apt_time = timestamp - prevAngleData.userDataDouble["APT_Vel_t"];
  double scaling_apt =
      min(delta_apt_time / kAptVelocityAveraging,
          0.25);  // Never add in the full amount due to the noise
  apt_velocity = apt_velocity * (1.0 - scaling_apt) +
                 scaling_apt * currData.pos.value().velocity;

  // Record back new data
  currData.userDataDouble["APT_Vel_x"] = apt_velocity.x;
  currData.userDataDouble["APT_Vel_y"] = apt_velocity.y;
  currData.userDataDouble["APT_Vel_t"] = timestamp;
  prevAngleData.userDataDouble["APT_Vel_x"] = apt_velocity.x;
  prevAngleData.userDataDouble["APT_Vel_y"] = apt_velocity.y;
  prevAngleData.userDataDouble["APT_Vel_t"] = timestamp;

  double robotVel = cv::norm(currData.pos.value().velocity);
  double aptRobotVel = cv::norm(apt_velocity);

  double minRobotVel = min(robotVel, aptRobotVel);

  // Check both distance and time thresholds
  if (cv::norm(delta) < kDistBetweenAngUpdatesPx || elapsedAngleTime < 0.001 ||
      !currData.pos.has_value())
  // || robotVel < VELOCITY_THRESH_FOR_ANGLE_UPDATE || aptRobotVel <
  // VELOCITY_THRESH_FOR_ANGLE_UPDATE/2.0)
  {
    // Copy previous angle data
    if (prevAngleData.angle.has_value() && currData.pos.has_value()) {
      currData.angle = AngleData(prevAngleData.angle.value().angle,
                                 prevAngleData.angle.value().velocity,
                                 prevAngleData.angle.value().time);
    } else {
      currData.angle.reset();
    }
    return;
  }

  // Update the angle
  Angle newAngle = Angle(atan2(delta.y, delta.x));
  if (std::isnan((double)newAngle)) {
    currData.angle.reset();
    return;
  }

  // If the angle is closer to 180 degrees to the last angle
  if (prevAngleData.angle.has_value() &&
      abs(Angle(newAngle + M_PI - prevAngleData.angle.value().angle)) <
          abs(Angle(newAngle - prevAngleData.angle.value().angle))) {
    // Add 180 degrees to the angle
    newAngle = Angle(newAngle + M_PI);
  }

  Angle newAngleInterpolated{0};
  double angularVelocity{0};

  // Calculate angular velocity
  if (prevAngleData.angle.has_value() &&
      !std::isnan((double)prevAngleData.angle.value().angle)) {
    // Base time-based weight
    double time_weight =
        min(elapsedAngleTime * 1000.0 / kAngleSmoothingTimeConstantMs, 1.0f);

    // Inverse time weight: if time updates exceeds reasonable levels, that
    // means there is a discontinuity in the data
    // Normaly we expect 20ms per update or faster.
    // if it approaches 150ms, its junk.
    time_weight *= (std::max)((0.15 - elapsedAngleTime) / 0.15, 0.02);

    // Velocity-based weight: scale based on aptRobotVel
    double velocity_weight =
        min(max((minRobotVel - kMinVelocityForAngleWeight) /
                    (kMaxVelocityForAngleWeight - kMinVelocityForAngleWeight),
                0.0),
            1.0);

    // Combine time and velocity weights (multiply or take minimum, depending on
    // desired behavior)
    double final_weight = max(time_weight * velocity_weight,
                              0.25);  // Limit the weight to prevent huge jumps

    // Interpolate the angle with velocity-modulated weight
    newAngleInterpolated = InterpolateAngles(prevAngleData.angle.value().angle,
                                             newAngle, final_weight);
    angularVelocity =
        (newAngleInterpolated - prevAngleData.angle.value().angle) /
        elapsedAngleTime;
  }

  currData.angle = AngleData(newAngleInterpolated, angularVelocity, timestamp);
  prevAngleData = currData;
}

void BlobDetection::SwitchRobots(void) {
  // Switch who's who
  std::unique_lock<std::mutex> locker(_updateMutex);
  OdometryData temp_Robot = _currDataRobot;
  _currDataRobot = _currDataOpponent;
  _currDataOpponent = temp_Robot;

  temp_Robot = _prevDataRobot;
  _prevDataRobot = _prevDataOpponent;
  _prevDataOpponent = temp_Robot;
}

void BlobDetection::SetPosition(cv::Point2f newPos, bool opponentRobot) {
  constexpr float IGNORE_THRESH_PX = 10;
  std::unique_lock<std::mutex> locker(_updateMutex);

  OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
  // ignore the request if our existing position is fine
  if (odoData.pos.has_value() &&
      cv::norm(newPos - odoData.pos.value().position) < IGNORE_THRESH_PX) {
    return;
  }

  double currTime = Clock::programClock.getElapsedTime();
  odoData.pos = PositionData(newPos, cv::Point2f(0, 0), currTime);

  OdometryData& prevData = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
  prevData.pos.reset();
}

void BlobDetection::SetVelocity(cv::Point2f newVel, bool opponentRobot) {
}

void BlobDetection::SetAngle(AngleData angleData, bool opponentRobot) {
}
