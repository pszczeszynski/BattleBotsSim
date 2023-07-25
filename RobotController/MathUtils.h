#pragma once
#include <cmath>
#include <opencv2/opencv.hpp>

#define M_PI 3.141592653589793238
#define TO_RAD (M_PI / 180.0)
#define TO_DEG (180.0 / M_PI)

// TODO: move this to a better place
double angle_between_points(double x1, double y1, double x2, double y2);

double angle_wrap(double angle);

cv::Point3f rotate3dPoint(cv::Point3f p, double angle_rad);

void calculateTangentPoints(const cv::Point2f& center, float radius, const cv::Point2f& externalPoint, cv::Point2f& tangent1, cv::Point2f& tangent2);

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
    explicit Angle(double value = 0.0, bool wrapAround = true)
    {
        if (wrapAround)
        {
            value_ = angle_wrap(value);
        }
        else
        {
            value_ = value;
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

private:
    double value_;
};

