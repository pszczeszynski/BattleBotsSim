#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../MathUtils.h"
#include <unordered_map>

constexpr float kMaxExtrapTimeS = 0.1;

class OdometryData
{
public:
    OdometryData(int id = 0);

    int id = 0; // Increment ID whenever data changes. A value of 0 means it
                // hasn't been initialized yet
    int frameID = -1; // The ID of the last video frame this data is based off.
                      // A value of -1 means it hasn't been initialized yet

    double time = 0; // The time of the last video frame this data is based off
                     // since the start of this program in seconds. A value of 0
                     // means it hasn't been initialized yet
    void Clear();    // Clears all position and user data to invalid;

    bool IsAngleValid() const; // returns true if the angle is valid
    void SetAngle(Angle newAngle, double newAngleVelocity,
                  double angleFrameTime, bool valid);
    Angle GetAngle() const; // gets the last set angle without extrapolation
    double GetAngleFrameTime() const;  // returns the time of the frame the angle was calculated on
    double GetAngleVelocity() const; // returns the last set angle velocity

    // returns a new instance of the data extrapolated to the target time
    // the maxRelativeTime is the maximum time to extrapolate forward
    OdometryData ExtrapolateBoundedTo(double targetTime,
                                      double maxRelativeTime = kMaxExtrapTimeS) const;

    void InvalidatePosition();
    void InvalidateAngle();

    // Set to true for our robot to help generic functions know
    bool isUs = false;

    // Our Position
    bool robotPosValid = false;
    cv::Point2f robotPosition;
    cv::Point2f robotVelocity; // Assume if position is good velocity is good, just starts at 0 if first frame

    // The rectangle to draw around us. only valid if robotPosValid is true
    // This should be optional
    cv::Rect rect;

    // User data for tracking algorithm internals
    std::unordered_map<std::string, double> userDataDouble;


    double GetAge() const; // returns the age of this data in seconds

    bool IsPointInside(cv::Point2f point) const
    {
        return (robotPosValid && rect.contains(point));
    }

    void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0)); // Returns an image that is used for debugging purposes.
private:
    // extrapolates the data to the new time without bound!
    OdometryData _ExtrapolateTo(double newtime) const;

    // Rotation
    bool _angleValid = false;
    double _angleVelocity = 0; // Clockwise
    Angle _angle;
    double _angleFrameTime = -1; // The time of the last angle update
};
