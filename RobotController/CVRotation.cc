#include "CVRotation.h"

#include <algorithm>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "MathUtils.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "UIWidgets/ClockWidget.h"

CVRotation* CVRotation::_instance = nullptr;

namespace {

// Converts network output [0..1] to XY in [-1..1]
cv::Point2f ConvertNetworkOutputToXY(const cv::Mat& output) {
  float normalized_x = output.at<float>(0, 0);
  float normalized_y = output.at<float>(0, 1);

  float x_component = (normalized_x * 2.0f) - 1.0f;
  float y_component = (normalized_y * 2.0f) - 1.0f;

  return cv::Point2f(x_component, y_component);
}

// Converts network output to rotation radians (model-specific)
float ConvertNetworkOutputToRad(const cv::Mat& output) {
  cv::Point2f xy = ConvertNetworkOutputToXY(output);
  float rotation = std::atan2(xy.y, xy.x);

  // Ensure angle within desired range (your model expects a half-angle)
  rotation = angle_wrap(rotation) * 0.5f;
  return rotation;
}

}  // namespace

CVRotation::CVRotation(ICameraReceiver* videoSource)
    : OdometryBase(videoSource) {
  // Load the model
  _net = cv::dnn::readNetFromONNX(MODEL_PATH);
  _net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
  _net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

  // init to 0,0 so confidence is 0
  _netXY1 = cv::Point2f(0, 0);
  _netXY2 = cv::Point2f(0, 0);
  _lastDisagreementRad = 0;
  _lastRotation = 0;

  _instance = this;
}

CVRotation* CVRotation::GetInstance() { return _instance; }

void CVRotation::_ProcessNewFrame(const cv::Mat frame, double frameTime) {
  // NOTE: This tracker depends on a position prior (from other odometry
  // sources). It does NOT depend on OdometryData objects anymore; it only uses
  // position/angle primitives.

  OdometryData robotData = RobotController::GetInstance().odometry.Robot();
  if (!robotData.pos.has_value()) {
    return;
  }
  cv::Point2f robotPos = robotData.pos.value().position;

  double rotation = ComputeRobotRotation(frame, robotPos, frameTime);
  double conf = GetLastConfidence();

  if (conf < ANGLE_FUSE_CONF_THRESH) {
    return;  // no publish event
  }

  OdometryData sample;
  sample.angle = AngleData(Angle(rotation), 0.0, frameTime);
  sample.pos = std::nullopt;

  Publish(sample, /*isOpponent=*/false);
}

bool CVRotation::_CropImage(const cv::Mat& input, cv::Mat& cropped,
                            cv::Rect roi) {
  // Create an output image of the size of the ROI filled with black
  cropped = cv::Mat(roi.size(), input.type(), cv::Scalar::all(0));

  // Define the image boundaries
  cv::Rect image_rect(0, 0, input.cols, input.rows);

  // Compute the overlapping region between the ROI and the input image
  cv::Rect validROI = roi & image_rect;

  if (validROI.area() <= 0) {
    return false;  // No valid image data within the crop
  }

  // Compute the destination ROI in the cropped image
  cv::Rect destROI(validROI.x - roi.x, validROI.y - roi.y, validROI.width,
                   validROI.height);

  // Copy the overlapping region from the input image to the cropped image
  input(validROI).copyTo(cropped(destROI));
  return true;
}

double CVRotation::ComputeRobotRotation(const cv::Mat& fieldImage,
                                        cv::Point2f robotPos,
                                        double frameTime) {
  constexpr int CROP_SIZE = 128;
  static ClockWidget clock("CVRotation");

  clock.markStart();

  // Convert to grayscale float32 [0..1]
  cv::Mat gray;
  if (fieldImage.channels() == 1) {
    gray = fieldImage;  // shallow copy ok
  } else if (fieldImage.channels() == 3) {
    cv::cvtColor(fieldImage, gray, cv::COLOR_BGR2GRAY);
  } else if (fieldImage.channels() == 4) {
    cv::cvtColor(fieldImage, gray, cv::COLOR_BGRA2GRAY);
  } else {
    // Unexpected channel count; fall back to last rotation
    clock.markEnd();
    return GetLastComputedRotation();
  }

  cv::Mat grayF;
  gray.convertTo(grayF, CV_32FC1, 1.0 / 255.0);

  // crop (zero padded)
  cv::Rect roi(static_cast<int>(robotPos.x - CROP_SIZE / 2),
               static_cast<int>(robotPos.y - CROP_SIZE / 2), CROP_SIZE,
               CROP_SIZE);

  cv::Mat cropped;
  if (!_CropImage(grayF, cropped, roi)) {
    clock.markEnd();
    return GetLastComputedRotation();
  }

  // flipped crop for second prediction
  cv::Mat croppedFlip;
  cv::flip(cropped, croppedFlip, 1);

  // blobs
  cv::Mat blob =
      cv::dnn::blobFromImage(cropped, 1.0, cv::Size(CROP_SIZE, CROP_SIZE),
                             cv::Scalar(0, 0, 0), false, false);
  cv::Mat blobFlip =
      cv::dnn::blobFromImage(croppedFlip, 1.0, cv::Size(CROP_SIZE, CROP_SIZE),
                             cv::Scalar(0, 0, 0), false, false);

  // forward #1
  _net.setInput(blob);
  cv::Mat out1 = _net.forward();
  cv::Point2f netXY1 = ConvertNetworkOutputToXY(out1);
  float rot1 = angle_wrap(ConvertNetworkOutputToRad(out1));

  // forward #2 (flip)
  _net.setInput(blobFlip);
  cv::Mat out2 = _net.forward();
  cv::Point2f netXY2 = ConvertNetworkOutputToXY(out2);
  float rot2 =
      angle_wrap(-ConvertNetworkOutputToRad(out2));  // negate due to flip

  // Disambiguate 180° symmetry using consistency between flipped/unflipped
  float diff = angle_wrap(rot1 - rot2);
  float diff2 = angle_wrap(rot2 - angle_wrap(rot1 - static_cast<float>(M_PI)));

  if (std::abs(diff2) < std::abs(diff)) {
    rot1 = angle_wrap(rot1 - static_cast<float>(M_PI));
  }

  // average
  float avgAngle =
      static_cast<float>(InterpolateAngles(Angle(rot1), Angle(rot2), 0.5));

  // Guiding angle to resolve +/- 180 ambiguity (IMU or fused angle)
  double guidingAngle =
      RobotController::GetInstance().odometry.Robot().GetAngleOrZero();
  if (std::abs(angle_wrap(avgAngle - guidingAngle)) > M_PI / 2) {
    avgAngle = angle_wrap(avgAngle + static_cast<float>(M_PI));
  }

  // If angle jumped too much vs last, blend
  double lastRot = GetLastComputedRotation();
  float deltaAngle = angle_wrap(avgAngle - static_cast<float>(lastRot));
  constexpr float THRESH_LARGE_CHANGE = 15.0f * TO_RAD;
  if (std::abs(deltaAngle) > THRESH_LARGE_CHANGE) {
    avgAngle = static_cast<float>(
        InterpolateAngles(Angle(lastRot), Angle(avgAngle), 0.5f));
  }

  // Update diagnostic/confidence state
  {
    std::lock_guard<std::mutex> lock(_updateMutex);
    _netXY1 = netXY1;
    _netXY2 = netXY2;
    _lastDisagreementRad = std::abs(angle_wrap(rot1 - rot2));
    _lastRotation = avgAngle;
    _lastUpdateTime = frameTime;
  }

  clock.markEnd();
  return avgAngle;
}

double CVRotation::GetLastComputedRotation() {
  std::lock_guard<std::mutex> lock(_updateMutex);
  return _lastRotation;
}

double CVRotation::GetLastConfidence() {
  std::lock_guard<std::mutex> lock(_updateMutex);

  double mag1 = cv::norm(_netXY1);
  double mag2 = cv::norm(_netXY2);
  double magnitudeConfidence = (mag1 + mag2) / 2.0;

  constexpr double MAX_DISAGREEMENT = 15.0 * TO_RAD;
  double angleConfidence = 1.0 - (_lastDisagreementRad / MAX_DISAGREEMENT);
  angleConfidence = (std::max)(0.0, angleConfidence);

  // You currently only use magnitude confidence; keep behavior consistent.
  return magnitudeConfidence;  // + 0.3 * angleConfidence;
}
