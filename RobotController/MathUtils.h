#pragma once
#include <cmath>
#include <opencv2/opencv.hpp>

#define M_PI 3.141592653589793238
#define TO_RAD (M_PI / 180.0)
#define TO_DEG (180.0 / M_PI)

// TODO: move this to a better place
double AngleBetweenPoints(double x1, double y1, double x2, double y2);

double angle_wrap(double angle_rad);
float angle_wrap(float angle_rad);

cv::Point3f rotate3dPoint(cv::Point3f p, double angle_rad);

void CalculateTangentPoints(const cv::Point2f& center, float radius, const cv::Point2f& externalPoint, cv::Point2f& tangent1, cv::Point2f& tangent2);

cv::Point2f InterpolatePoints(const cv::Point2f& p1, const cv::Point2f& p2, double ratio);

std::vector<cv::Point2f> CirclesIntersect(cv::Point2f center1, float r1, cv::Point2f center2, float r2);

std::vector<cv::Point2f> CircleLineSegmentIntersect(cv::Point2f circleCenter, float circleRadius, cv::Point2f lineStart, cv::Point2f lineEnd);

float distance(cv::Point2f point1, cv::Point2f point2 );

class Point2f
{
public:
    double x;
    double y;
    Point2f(double x, double y)
        : x(x), y(y)
    {
    }

    // define addition operator
    Point2f operator+(const Point2f& p) const
    {
        return Point2f(x + p.x, y + p.y);
    }

    // define multiplication operator
    Point2f operator*(const double& d) const
    {
        return Point2f(x * d, y * d);
    }

    // define subtraction operator
    Point2f operator-(const Point2f& p) const
    {
        return Point2f(x - p.x, y - p.y);
    }

    // define division operator
    Point2f operator/(const double& d) const
    {
        return Point2f(x / d, y / d);
    }
};

double norm(Point2f p);

cv::Point2f rotate_point(cv::Point2f p, double angle);

class Angle
{
public:
    explicit Angle(double value_rad = 0.0, bool wrapAround = true)
    {
        if (wrapAround)
        {
            value_ = angle_wrap(value_rad);
        }
        else
        {
            value_ = value_rad;
        }
    }

    // Overload of operator+
    Angle operator+(const Angle& other) const
    {
        return Angle(value_ + other.value_);
    }

    // Overload of operator-
    Angle operator-(const Angle& other) const
    {
        return Angle(value_ - other.value_);
    }

    Angle operator*(const double& d) const
    {
        return Angle(value_ * d);
    }

    Angle operator/(const double& d) const
    {
        return Angle(value_ / d, false);
    }

    // Conversion operator to double
    operator double() const { return value_; }

    double degrees() const
    {
        return value_ * TO_DEG;
    }

private:
    double value_; // Value in rads
};


Angle InterpolateAngles(Angle a1, Angle a2, double ratio);
double Interpolate(double start, double end, double ratio);

bool SegmentsIntersect(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r);

bool getClosestPointOnSegment(cv::Point2f start, cv::Point2f end, cv::Point2f point, cv::Point2f& closestPoint);