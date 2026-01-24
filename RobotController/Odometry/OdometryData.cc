#include "OdometryData.h"

#include <opencv2/core.hpp>

// clock widget
#include "../Odometry/Heuristic1/RobotTracker.h"
#include "../UIWidgets/ClockWidget.h"
#include "../Globals.h"


// ctor for odometry data
OdometryData::OdometryData(int id)
    : id(id), robotPosition(-1.0f, -1.0f), robotVelocity(0, 0) {}

// Clear all position data
void OdometryData::Clear() {
  // Our Position
  robotPosValid = false;
  robotPosition = cv::Point2f(-1.0f, -1.0f);
  robotVelocity = cv::Point2f(0, 0);  // pixels/second

  rect = cv::Rect2i(0, 0, 0, 0);

  // Our Rotation
  _angleValid = false;
  _angle = Angle(0);
  _angleVelocity = 0;  // Clockwise, rads/s

  // Clear user data
  userDataDouble.clear();
}

OdometryData OdometryData::_ExtrapolateTo(double newtime) const {
  // return *this;
  OdometryData result = *this;  // Create a copy of current data

  // Extrapolate only if data is marked as valid
  if (robotPosValid) {
    result.robotPosition += robotVelocity * (newtime - time);
    result.time = newtime;
  }

  if (_angleValid) {
    Angle newAngle =
        GetAngle() + Angle(_angleVelocity * (newtime - _angleFrameTime));
    result._angleFrameTime = newtime;
    result.SetAngle(newAngle, _angleVelocity, newtime, true);
  }

  return result;
}

OdometryData OdometryData::ExtrapolateBoundedTo(double targetTime,
                                                double maxRelativeTime) const {
  return _ExtrapolateTo((std::min)(targetTime, time + maxRelativeTime));
}

/**
 * Returns the age of this data in seconds
 */
double OdometryData::GetAge() const {
  return ClockWidget::programClock.getElapsedTime() - time;
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
  ss << " ID: " << (id == 0 ? "Not set" : std::to_string(id));
  ss << "\n";
  ss << " Frame ID: " << (frameID == -1 ? "Not set" : std::to_string(frameID));
  ss << "\n";
  ss << " Time: " << (time == 0 ? "Not set" : std::to_string(time) + " s");
  ss << "\n";
  ss << " Time Angle: "
     << (_angleFrameTime == -1 ? "Not set"
                               : std::to_string(_angleFrameTime) + " s");
  ss << "\n ";
  if (!robotPosValid) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << "Pos: ";
  ss << "(" << robotPosition.x << ", " << robotPosition.y << ")";
  ss << "\n";

  // Velocity
  if (!robotPosValid) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }

  double magnitude = cv::norm(robotVelocity);  // Euclidean norm
  double angle = std::atan2(robotVelocity.y, robotVelocity.x) * 180.0 /
                 CV_PI;  // Convert to degrees
  ss << " Vel: " << magnitude << " px/s, " << angle << " deg";

  ss << "\n";

  // Angle and Angle Velocity
  if (!_angleValid) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << " Angle: ";
  ss << _angle.degrees() << " deg";

  ss << "\n";
  if (!_angleValid) {
    ss << "Invalid ";
  } else {
    ss << "Valid ";
  }
  ss << " AVel: ";
  ss << _angleVelocity << " deg/s";

  printText(ss.str(), target, offset.y, offset.x);
};

void OdometryData::SetAngle(Angle newAngle, double newAngleVelocity,
                            double angleFrameTime, bool valid) {
  _angle = newAngle;
  _angleFrameTime = angleFrameTime;
  _angleVelocity = newAngleVelocity;
  _angleValid = valid;
}

Angle OdometryData::GetAngle() const { return _angle; }

double OdometryData::GetAngleFrameTime() const { return _angleFrameTime; }

double OdometryData::GetAngleVelocity() const { return _angleVelocity; }

bool OdometryData::IsAngleValid() const { return _angleValid; }

void OdometryData::InvalidatePosition() { robotPosValid = false; }

void OdometryData::InvalidateAngle() { _angleValid = false; }
