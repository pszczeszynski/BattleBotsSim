#include "OdometryData.h"

#include <opencv2/core.hpp>

#include "../Globals.h"
#include "../Odometry/Heuristic1/RobotTracker.h"

const char* OdometryAlgToString(OdometryAlg a) {
  switch (a) {
    case OdometryAlg::Blob:
      return "Blob";
    case OdometryAlg::Heuristic:
      return "Heuristic";
    case OdometryAlg::IMU:
      return "IMU";
    case OdometryAlg::Neural:
      return "Neural";
    case OdometryAlg::Human:
      return "Human";
    case OdometryAlg::NeuralRot:
      return "NeuralRot";
    case OdometryAlg::OpenCV:
      return "OpenCV";
    case OdometryAlg::LKFlow:
      return "LKFlow";
    case OdometryAlg::Gyro:
      return "Gyro";
    case OdometryAlg::ManualOverride:
      return "ManualOverride";
    case OdometryAlg::UnknownAlg:
    default:
      return "Unknown";
  }
}

// Clear all position data
void OdometryData::Clear() {
  pos.reset();
  angle.reset();
  userDataDouble.clear();
}

PositionData PositionData::ExtrapolateBoundedTo(double targetTime,
                                                double maxRelativeTime) const {
  double extrapTime = std::min(targetTime, time + maxRelativeTime);
  double deltaTime =
      extrapTime -
      (extrapolated_time.has_value() ? extrapolated_time.value() : time);
  PositionData result = *this;
  result.position += result.velocity * static_cast<float>(deltaTime);
  result.extrapolated_time = extrapTime;
  return result;
}

AngleData AngleData::ExtrapolateBoundedTo(double targetTime,
                                          double maxRelativeTime) const {
  double extrapTime = std::min(targetTime, time + maxRelativeTime);
  double deltaTime =
      extrapTime -
      (extrapolated_time.has_value() ? extrapolated_time.value() : time);
  AngleData result = *this;
  result.angle = angle + Angle(velocity * deltaTime);
  result.extrapolated_time = extrapTime;
  return result;
}

OdometryData OdometryData::ExtrapolateBoundedTo(double targetTime,
                                                double maxRelativeTime) const {
  OdometryData result = *this;
  if (pos.has_value()) {
    result.pos = pos.value().ExtrapolateBoundedTo(targetTime, maxRelativeTime);
  }
  if (angle.has_value()) {
    result.angle =
        angle.value().ExtrapolateBoundedTo(targetTime, maxRelativeTime);
  }
  return result;
}

// Adds to the passed image odometry data for debugging purposes.
void OdometryData::GetDebugImage(cv::Mat& target, cv::Point offset) {
  if (target.empty()) {
    target = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
  }

  std::stringstream ss;

  ss << std::fixed << std::setprecision(1);

  // ID
  ss << " ID: " << (id == 0 ? "Not set" : std::to_string(id));
  ss << "\n";

  // Position
  if (!pos.has_value()) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << "Pos: ";
  if (pos.has_value()) {
    ss << "(" << pos.value().position.x << ", " << pos.value().position.y
       << ")";
  } else {
    ss << "no value";
  }
  ss << " ["
     << (pos.has_value() ? OdometryAlgToString(pos.value().algorithm)
                         : "not set")
     << "]\n";

  // Velocity
  if (!pos.has_value()) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }

  if (pos.has_value()) {
    double magnitude = cv::norm(pos.value().velocity);  // Euclidean norm
    double velAngle =
        std::atan2(pos.value().velocity.y, pos.value().velocity.x) * 180.0 /
        CV_PI;  // Convert to degrees
    ss << " Vel: " << magnitude << " px/s, " << velAngle << " deg";
  } else {
    ss << " Vel: no value";
  }

  ss << "\n";

  // Angle and Angle Velocity
  if (!angle.has_value()) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << " Angle: ";
  if (angle.has_value()) {
    ss << angle.value().angle.degrees() << " deg";
  } else {
    ss << "no value";
  }
  ss << " ["
     << (angle.has_value() ? OdometryAlgToString(angle.value().algorithm)
                           : "Unknown")
     << "]\n";
  if (!angle.has_value()) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << " AVel: ";
  if (angle.has_value()) {
    ss << angle.value().velocity << " rad/s";
  } else {
    ss << "no value";
  }

  printText(ss.str(), target, offset.y, offset.x);
};
