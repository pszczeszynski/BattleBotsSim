#include "OdometryData.h"

#include <opencv2/core.hpp>

#include "../Globals.h"
#include "../Odometry/Heuristic1/RobotTracker.h"

// ctor for odometry data
OdometryData::OdometryData(int id) : id(id) {}

// Clear all position data
void OdometryData::Clear() {
  // Clear position and angle data
  pos.reset();
  angle.reset();

  // Clear user data
  userDataDouble.clear();
}

OdometryData OdometryData::_ExtrapolateTo(double newtime) const {
  OdometryData result = *this;  // Create a copy of current data

  // Extrapolate position if valid
  if (pos.has_value()) {
    double deltaTime = newtime - pos.value().time;
    PositionData newPos = pos.value();
    newPos.position += newPos.velocity * static_cast<float>(deltaTime);
    newPos.time = newtime;
    result.pos = newPos;
  }

  // Extrapolate angle if valid
  if (angle.has_value()) {
    double deltaTime = newtime - angle.value().time;
    Angle newAngle =
        angle.value().angle + Angle(angle.value().velocity * deltaTime);
    result.angle = AngleData(newAngle, angle.value().velocity, newtime);
  }

  return result;
}

OdometryData OdometryData::ExtrapolateBoundedTo(double targetTime,
                                                double maxRelativeTime) const {
  // Use position time if available, otherwise angle time, otherwise 0
  double baseTime = pos.has_value()
                        ? pos.value().time
                        : (angle.has_value() ? angle.value().time : 0);
  return _ExtrapolateTo((std::min)(targetTime, baseTime + maxRelativeTime));
}

// Adds to the passed image odometry data for debugging purposes.
void OdometryData::GetDebugImage(cv::Mat &target, cv::Point offset) {
  // This should not happen, but in case it does, we will create an empty image
  if (target.empty()) {
    target = cv::Mat(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0));
  }

  std::stringstream ss;

  ss << std::fixed << std::setprecision(1);

  // ID
  ss << " ID: " << ( id == 0 ? "Not set" : std::to_string(id));
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
    ss << "(-1, -1)";
  }
  ss << "\n";

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
    ss << " Vel: 0 px/s, 0 deg";
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
    ss << "0 deg";
  }

  ss << "\n";
  if (!angle.has_value()) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << " AVel: ";
  if (angle.has_value()) {
    ss << angle.value().velocity << " rad/s";
  } else {
    ss << "0 rad/s";
  }

  printText(ss.str(), target, offset.y, offset.x);
};
