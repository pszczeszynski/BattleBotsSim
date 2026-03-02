#include "RobotClassifier.h"

#include "../../Clock.h"
#include "../../RobotConfig.h"
#include "BlobDetection.h"  // For VisionClassification definition

RobotClassifier::RobotClassifier(void) {
  robotCalibrationData.meanColor = cv::Scalar(0, 0, 100);
  robotCalibrationData.diameter = 50;

  opponentCalibrationData.meanColor = cv::Scalar(100, 100, 100);
  opponentCalibrationData.diameter = 50;
}

/**
 * Returns a number that is more negative the more we think the robot is us.
 *
 */
double RobotClassifier::ClassifyBlob(MotionBlob &blob, const cv::Mat &frame,
                                     const cv::Mat &motionImage,
                                     const TrackPrior &usPrior,
                                     const TrackPrior &themPrior) {
  // Use default position if prior is invalid
  cv::Point2f usPos = usPrior.valid ? usPrior.pos : cv::Point2f(0, 0);
  cv::Point2f themPos = themPrior.valid ? themPrior.pos : cv::Point2f(0, 0);

  // now let's take their location into account
  double distanceToRobot = cv::norm(usPos - blob.center);
  double distanceToOpponent = cv::norm(themPos - blob.center);

  // normalize the distance score
  double distanceScoreNormalized = (distanceToRobot - distanceToOpponent) /
                                   (distanceToRobot + distanceToOpponent);

  // weight and sum the scores
  return distanceScoreNormalized;
}

/**
 * Returns the closest blob to the given point and removes it from the list
 */
MotionBlob GetClosestBlobAndRemove(std::vector<MotionBlob> &blobs,
                                   cv::Point2f point) {
  int index = 0;
  double minDist = 10000000;

  for (int i = 0; i < blobs.size(); i++) {
    double dist = cv::norm(blobs[i].center - point);
    if (dist < minDist) {
      minDist = dist;
      index = i;
    }
  }

  MotionBlob closest = blobs[index];
  blobs.erase(blobs.begin() + index);
  return closest;
}

/**
 * @brief Classifies the blobs and returns the robot and opponent blobs
 * @param blobs the blobs to classify
 * @param frame the latest frame from the camera
 * @param motionImage a frame with only the motion as white pixels
 */
VisionClassification RobotClassifier::ClassifyBlobs(
    std::vector<MotionBlob> &blobs, const cv::Mat &frame,
    const cv::Mat &motionImage, const TrackPrior &usPrior,
    const TrackPrior &themPrior, double frameTime,
    std::vector<BlobDebugInfo>* outDebugInfo) {
  VisionClassification classificationResult;

  static int lastBlobsSize = 0;
  static int lastNeuralID = -1;
  static Clock noRobotClock;
  static Clock noOpponentClock;

  double matchingDistThresholdRobot = BLOB_MATCHING_DIST_THRESHOLD;
  double matchingDistThresholdOpponent = BLOB_MATCHING_DIST_THRESHOLD;

  // Use default positions if priors are invalid
  cv::Point2f usPos = usPrior.valid ? usPrior.pos : cv::Point2f(0, 0);
  cv::Point2f themPos = themPrior.valid ? themPrior.pos : cv::Point2f(0, 0);

  auto addDebug = [&](const MotionBlob& b, bool valid, const std::string& reason) {
    if (outDebugInfo) {
      outDebugInfo->push_back({b.rect, b.center, valid, reason});
    }
  };

  // if only one blob
  if (blobs.size() == 1) {
    // Note: RobotOdometry gets updates position from other sources, thus
    // GetPosition can change while we are running this code. Is it ok to
    // process this frame's blob detection with a position that may be based on
    // a newer frame we haven't processed yet? This is probably ok since this
    // code is one of the fastest codes run, and if better information on
    // GetPosition arrives from other algorithms then we want to search for
    // blobs near it.
    double distanceToRobot = cv::norm(usPos - blobs[0].center);
    double distanceToOpponent = cv::norm(themPos - blobs[0].center);

    bool isRobot =
        ClassifyBlob(blobs[0], frame, motionImage, usPrior, themPrior) <= 0;

    // if the blob is really close to both robots
    if (distanceToRobot < 10 && distanceToOpponent < 10) {
      // choose the one with more velocity last time
      isRobot = cv::norm(usPrior.vel) > cv::norm(themPrior.vel);
    }

    double blobSize = sqrt(blobs[0].rect.area());

    // if this is the robot (not the opponent)
    if (isRobot) {
      if (distanceToRobot < matchingDistThresholdRobot &&
          blobSize >= MIN_ROBOT_BLOB_SIZE &&
          blobSize <= MAX_ROBOT_BLOB_SIZE) {
        classificationResult.robot = blobs[0];
        noRobotClock.markStart();
        addDebug(blobs[0], true, "");
      } else {
        std::string reason;
        if (distanceToRobot >= matchingDistThresholdRobot)
          reason = "robot too far (" + std::to_string((int)distanceToRobot) + "px)";
        else if (blobSize < MIN_ROBOT_BLOB_SIZE)
          reason = "robot too small (" + std::to_string((int)blobSize) + ")";
        else
          reason = "robot too large (" + std::to_string((int)blobSize) + ")";
        addDebug(blobs[0], false, reason);
      }
    } else {
      // make sure it's close enough and the size is big enough
      if (distanceToOpponent < matchingDistThresholdOpponent &&
          blobSize >= MIN_OPPONENT_BLOB_SIZE &&
          blobSize <= MAX_OPPONENT_BLOB_SIZE) {
        classificationResult.opponent = blobs[0];
        noRobotClock.markStart();
        addDebug(blobs[0], true, "");
      } else {
        std::string reason;
        if (distanceToOpponent >= matchingDistThresholdOpponent)
          reason = "opponent too far (" + std::to_string((int)distanceToOpponent) + "px)";
        else if (blobSize < MIN_OPPONENT_BLOB_SIZE)
          reason = "opponent too small (" + std::to_string((int)blobSize) + ")";
        else
          reason = "opponent too large (" + std::to_string((int)blobSize) + ")";
        addDebug(blobs[0], false, reason);
      }
    }
  }
  // otherwise have more than 2 blobs, only use the first 2 blobs
  else if (blobs.size() >= 2) {
    std::vector<MotionBlob> blobsToChooseFrom = {};
    std::vector<MotionBlob> filtered = {};
    // copy over all the blobs
    std::copy(blobs.begin(), blobs.end(),
              std::back_inserter(blobsToChooseFrom));
    filtered.push_back(GetClosestBlobAndRemove(blobsToChooseFrom, usPos));
    filtered.push_back(GetClosestBlobAndRemove(blobsToChooseFrom, themPos));

    // Add debug for blobs not in top 2
    for (const MotionBlob& b : blobsToChooseFrom) {
      addDebug(b, false, "not in top 2 closest");
    }

    double firstIsRobot =
        ClassifyBlob(filtered[0], frame, motionImage, usPrior, themPrior);
    double secondIsRobot =
        ClassifyBlob(filtered[1], frame, motionImage, usPrior, themPrior);
    double preference = firstIsRobot - secondIsRobot;

    MotionBlob &robot = preference <= 0 ? filtered[0] : filtered[1];
    MotionBlob &opponent = preference <= 0 ? filtered[1] : filtered[0];

    auto checkRobot = [&](const MotionBlob& b) {
      double dist = cv::norm(b.center - usPos);
      double sz = sqrt(b.rect.area());
      if (dist < matchingDistThresholdRobot &&
          sz >= MIN_ROBOT_BLOB_SIZE && sz <= MAX_ROBOT_BLOB_SIZE) {
        addDebug(b, true, "");
        return true;
      }
      std::string reason;
      if (dist >= matchingDistThresholdRobot)
        reason = "robot too far (" + std::to_string((int)dist) + "px)";
      else if (sz < MIN_ROBOT_BLOB_SIZE)
        reason = "robot too small (" + std::to_string((int)sz) + ")";
      else
        reason = "robot too large (" + std::to_string((int)sz) + ")";
      addDebug(b, false, reason);
      return false;
    };
    auto checkOpponent = [&](const MotionBlob& b) {
      double dist = cv::norm(b.center - themPos);
      double sz = sqrt(b.rect.area());
      if (dist < matchingDistThresholdOpponent &&
          sz >= MIN_OPPONENT_BLOB_SIZE && sz <= MAX_OPPONENT_BLOB_SIZE) {
        addDebug(b, true, "");
        return true;
      }
      std::string reason;
      if (dist >= matchingDistThresholdOpponent)
        reason = "opponent too far (" + std::to_string((int)dist) + "px)";
      else if (sz < MIN_OPPONENT_BLOB_SIZE)
        reason = "opponent too small (" + std::to_string((int)sz) + ")";
      else
        reason = "opponent too large (" + std::to_string((int)sz) + ")";
      addDebug(b, false, reason);
      return false;
    };

    // if the robot blob is close to the robot tracker
    if (checkRobot(robot)) {
      classificationResult.robot = robot;
      noRobotClock.markStart();
    }

    // if the opponent blob is close to the opponent tracker
    if (checkOpponent(opponent)) {
      classificationResult.opponent = opponent;
      noRobotClock.markStart();
    }
  }

  lastBlobsSize = blobs.size();

  // return our classification
  return classificationResult;
}
