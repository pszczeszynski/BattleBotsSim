#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <optional>
#include <unordered_map>

#include "../Clock.h"
#include "../MathUtils.h"

constexpr float kMaxExtrapTimeS = 0.1;

struct AngleData {
  Angle angle;
  double velocity = 0;
  double time;

  double GetAge() const { return Clock::programClock.getElapsedTime() - time; }

  AngleData(Angle a, double v, double t) : angle(a), velocity(v), time(t) {}
};

struct PositionData {
  cv::Point2f position;
  cv::Point2f velocity;
  std::optional<cv::Rect> rect;
  double time = 0;

  PositionData(cv::Point2f pos, cv::Point2f vel, double t)
      : position(pos), velocity(vel), rect(std::nullopt), time(t) {}
  PositionData(cv::Point2f pos, cv::Point2f vel, cv::Rect r, double t)
      : position(pos), velocity(vel), rect(r), time(t) {}

  double GetAge() const { return Clock::programClock.getElapsedTime() - time; }
};

class OdometryData {
 public:
  OdometryData(int id);

  // User data for tracking algorithm internals
  std::unordered_map<std::string, double> userDataDouble;

  // Angle and Position data (nullopt means invalid)
  std::optional<AngleData> angle;
  std::optional<PositionData> pos;
  int id = 0;

  void Clear();  // Clears all position and user data to invalid;

  // returns a new instance of the data extrapolated to the target time
  // the maxRelativeTime is the maximum time to extrapolate forward
  OdometryData ExtrapolateBoundedTo(
      double targetTime, double maxRelativeTime = kMaxExtrapTimeS) const;

  bool IsPointInside(cv::Point2f point) const {
    if (!pos.has_value()) {
      return false;
    }

    if (pos.value().rect.has_value()) {
      return pos.value().rect.value().contains(point);
    } else {
      constexpr float kDefaultCloseEnoughRadius = 30;
      return cv::norm(pos.value().position - point) < kDefaultCloseEnoughRadius;
    }
  }

  void GetDebugImage(
      cv::Mat& target,
      cv::Point offset = cv::Point(
          0, 0));  // Returns an image that is used for debugging purposes.
 private:

  // extrapolates the data to the new time without bound!
  OdometryData _ExtrapolateTo(double newtime) const;
};
